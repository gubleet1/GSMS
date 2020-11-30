function vel_kf_measure(measured_b) %#codegen
%VEL_KF_MEASURE measurement step (measurement update)

global vel_kf_x; % state vector x
global vel_kf_R; % measurement noise covariance matrix R
global vel_kf_P; % error covariance matrix P

% calculate measurement innovation vector y
y = measured_b - vel_kf_x;
% calculate kalman gain matrix K
S = vel_kf_P + vel_kf_R;
K = vel_kf_P / S;
% calculate corrected (aposteriori) state vector x(+)
vel_kf_x = vel_kf_x + K * y;
% calculate corrected (aposteriori) error covariance matrix P(+)
vel_kf_P = (eye(3) - K) * vel_kf_P;
end
