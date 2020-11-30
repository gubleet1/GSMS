function vel_kf_predict(lin_accel) %#codegen
%VEL_KF_PREDICT prediction step (time update)

global vel_kf_x; % state vector x
global vel_kf_Q; % process noise covariance matrix Q
global vel_kf_P; % error covariance matrix P

% calculate delta time
dt = 10 / 1000; % [s]
% calculate predicted (apriori) state vector x(-)
vel_kf_x = vel_kf_x + (lin_accel * dt);
% calculate predicted (apriori) error covariance matrix P(-)
vel_kf_P = vel_kf_P + vel_kf_Q;
end
