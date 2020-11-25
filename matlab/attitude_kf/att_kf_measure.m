function att_kf_measure(expected_i, measured_b) %#codegen
%ATT_KF_MEASURE measurement step (measurement update)

global att_kf_q_ref; % reference attitude quaternion q_ref
global att_kf_x; % state vector x
global att_kf_R; % measurement noise covariance matrix R
global att_kf_P; % error covariance matrix P

% skip the measurement step if the vector measured_b is close to zero
if norm(measured_b) < 0.001
    return;
end

% normalize expected vector in the inertial frame
expected_i = expected_i / norm(expected_i);
% normalize measured vector in the body frame
measured_b = measured_b / norm(measured_b);
% Note: The following steps calculate a rotation matrix to transform the
%       vector measured_b from the body frame into the inertial frame and to
%       rotate it onto the vector m (normal vector of the projection plane).
i_to_m = quat_from_vect(expected_i, [1; 0; 0]);
b_to_m = quat_mult(i_to_m, att_kf_q_ref);
b_to_m = rotm_from_quat(b_to_m);
% rotate vector expected_i from the inertial frame into the body frame
expected_b = rotate_by_quat(expected_i, quat_conj(att_kf_q_ref));
% Note: The following steps rotate the vector measured_b using the rotation
%       matrix b_to_m and then calculate the measurement vector z by
%       projecting the rotated vector measured_b onto the y, z plane.
m = b_to_m * measured_b + [1; 0; 0];
% skip the measurement step if the expected and the measured vector are antiparallel
if norm(m) < 0.001
    return;
end
m = m / norm(m);
z = m(2:3) / m(1) * 2;
% calculate observation matrix H
Proj = [0, 1, 0;
        0, 0, 1];
Ha = Proj * b_to_m * cross_prod_mat(expected_b);
H = [Ha, zeros(2, 3)];
% calculate measurement innovation vector y
h = [0; 0];
y = z - h;
% calculate kalman gain matrix K
S = H * att_kf_P * H' + att_kf_R;
K = att_kf_P * H' / S;
% calculate corrected (aposteriori) state vector x(+)
att_kf_x = att_kf_x + K * y;
% calculate corrected (aposteriori) error covariance matrix P(+)
att_kf_P = (eye(6) - K * H) * att_kf_P;
end

function q = quat_from_vect(v1, v2) %#codegen
%QUAT_FROM_VECT quaternion from two 3 element vectors

v1 = v1 / norm(v1);
v2 = v2 / norm(v2);
c = v1' * v2;
if (c > 0.99999)
    q = [1; 0; 0; 0];
elseif (c < -0.99999)
    w = cross(v1, [1, 1, 1]);
    q = [0; w/norm(w)];
else
    ax = cross(v1, v2);
    s = sqrt((1+c)*2);
    q = [s*0.5; ax/s];
end
end

function m = rotm_from_quat(q) %#codegen
%ROTM_FROM_QUAT rotation matrix from quaternion

m1 = rotate_by_quat([1; 0; 0], q);
m2 = rotate_by_quat([0; 1; 0], q);
m3 = rotate_by_quat([0; 0; 1], q);
m = [m1, m2, m3];
end
