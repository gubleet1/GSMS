function att_kf_predict(w) %#codegen
%ATT_KF_PREDICT prediction step (time update)

global att_kf_q_ref; % reference attitude quaternion q_ref
global att_kf_x; % state vector x
global att_kf_Q; % process noise covariance matrix Q
global att_kf_P; % error covariance matrix P

% calculate delta time
dt = 10 / 1000; % [s]
% remove gyroscope bias
omega = w - att_kf_x(4:6);
% calculate angle of rotation
ang = norm(omega) * dt;
% calculate delta quaternion
if ang > 0.000001
    % calculate delata quaternion from angle and axis of rotation
    axis = omega / norm(omega);
    delta_q_ref = [cos(ang/2); axis * sin(ang/2)];
else
    % calculate delta quaternion using small angle approximation
    delta_q_ref = [1; omega * dt / 2];
end
% predict q_ref
att_kf_q_ref = quat_mult(att_kf_q_ref, delta_q_ref);
% calculate linearized state transition matrix Phi
% Note: The EKF linearizes the underlying model to produce matrix Phi.
F = [cross_prod_mat(-omega), eye(3) * -1;
     zeros(3, 6)];
A = [-F, att_kf_Q';
     zeros(6,6), F'];
B = expm(A * dt);
Phi = B(7:12, 7:12)';
% calcualte linearized process noise covariance matrix Qs
% Note: The EKF linearizes the underlying model to produce matrix Qs.
Qs = Phi * B(1:6, 7:12);
% Note: The attitude error is transferred to q_ref after every iteration.
%       This ensures, that the predicted (apriori) state vector x(-) is
%       equal to the state vector x of the previous iteration.
% calculate predicted (apriori) error covariance matrix P(-)
att_kf_P = Phi * att_kf_P * Phi' + Qs;
end
