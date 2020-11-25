function att_kf() %#codegen
%ATT_KF initialize attitude kalman filter

global att_kf_q_ref; % reference attitude quaternion q_ref
global att_kf_x; % state vector x
global att_kf_Q; % process noise covariance matrix Q
global att_kf_R; % measurement noise covariance matrix R
global att_kf_P; % error covariance matrix P

% attitude kalman filter constants
gyro_white_noise = 0.03 / 180 * pi; % [rad/s/sqrt(Hz)]
gyro_bias_white_noise = 0.003 / sqrt(200) / 180 * pi; % [rad/s^2/sqrt(Hz)]
measurement_white_noise = 4 / 180 * pi; % [uT/s/sqrt(Hz)], [m/s^3/sqrt(Hz)]
init_attitude_error_stddev = sqrt(1000); % [-]
init_gyro_bias_stddev = 2 / 180 * pi; % [rad/s]

% initialize kalman filter variables
att_kf_q_ref = [1; 0; 0; 0];
att_kf_x = zeros(6, 1);
att_kf_Q = diag([ones(1, 3) * gyro_white_noise^2, ones(1, 3) * gyro_bias_white_noise^2]);
att_kf_R = eye(2) * measurement_white_noise^2;
att_kf_P = diag([ones(1, 3) * init_attitude_error_stddev^2, ones(1, 3) * init_gyro_bias_stddev^2]);
end
