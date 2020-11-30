function vel_kf() %#codegen
%VEL_KF initialize velocity kalman filter

global vel_kf_x; % state vector x
global vel_kf_Q; % process noise covariance matrix Q
global vel_kf_R; % measurement noise covariance matrix R
global vel_kf_P; % error covariance matrix P

% velocity kalman filter constants
accel_white_noise = 150 / 1000000 * 9.8053; % [m/s^2/sqrt(Hz)]
measurement_white_noise = 4 / 180 * pi; % [m/s/sqrt(Hz)]
init_velocity_error_stddev = sqrt(1000); % [m/s]

% initialize kalman filter variables
vel_kf_x = zeros(3, 1);
vel_kf_Q = eye(3) * accel_white_noise^2;
vel_kf_R = eye(3) * measurement_white_noise^2;
vel_kf_P = eye(3) * init_velocity_error_stddev^2;
end
