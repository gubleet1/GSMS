% test attitude kalman filter

% read raw bno055 data
fid = fopen("bno055_raw.txt");
data = textscan(fid, '%4xs16');
data = data{1,1};
data = data';
data = reshape(data, 16, []);
data = double(data);
count = length(data);
% accelerometer
accel = data(1:3,:) / 100; % [m/s^2]
% magnetometer
mag = data(4:6,:) / 16; % [uT]
% gyroscope
gyro = data(7:9,:) / 900; % [rad/s]
% quaternion data
quat = data(13:16,:) / 2^14; % [-]

% delta t
dt = ones(1, count) * 10 / 1000; % [s]
% calculate absolute time
t = cumsum(dt);

% plot actual attitude
figure;
plot(t, quat(1,:));
hold on
plot(t, quat(2,:));
plot(t, quat(3,:));
plot(t, quat(4,:));
hold off
title("Actual Attitude");
xlabel("time [s]");
ylabel("quaternion components [-]");
legend("w", "x", "y", "z");

% kalman filter attitude
quat_kf = zeros(4, count);
% attitude error
diff = zeros(1, count); % [deg]

% initialize attitude kalman filter
att_kf();
% run attitude kalman filter
for i = 1:count
    % prediction step (time update)
    att_kf_predict(gyro(:,i));
    % measurement step (measurement update)
    % measure gravity vector
    att_kf_measure([0; 0; 1], accel(:,i));
    % measure north vector
    att_kf_measure([0; 0.446; -0.895], mag(:,i));
    % propagate attitude error
    att_kf_propagate();
    % store kalman filter attitude
    quat_kf(:,i) = att_kf_q_ref();
    % calculate attitude error
    diff(i) = angle_between_quat(quat_kf(:,i), quat(:,i)) / pi * 180;
end

% plot attitude error
figure;
plot(t, diff);
title("Attitude Error");
xlabel("time [s]");
ylabel("attitude error [deg]");
legend("attitude error");

% plot kalman filter attitude
figure;
plot(t, quat_kf(1,:));
hold on
plot(t, quat_kf(2,:));
plot(t, quat_kf(3,:));
plot(t, quat_kf(4,:));
hold off
title("Kalman Filter Attitude");
xlabel("time [s]");
ylabel("quaternion components [-]");
legend("w", "x", "y", "z");
