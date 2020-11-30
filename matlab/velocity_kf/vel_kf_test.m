% test velocity kalman filter

% read raw bno055 data
fid = fopen("bno055_raw.txt");
data = textscan(fid, '%4xs16');
fclose(fid);
data = data{1,1};
data = data';
data = reshape(data, 19, []);
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
% linear acceleration data
lin_accel = data(17:19,:) / 100; % [m/s^2]

% read raw neo-m9n data
fid = fopen("neom9n_raw.txt");
data = textscan(fid, '%8xs32');
fclose(fid);
data = data{1,1};
data = data';
data = reshape(data, 3, []);
data = double(data);
% mark empty velocity samples using NaN
vel_empty = -2147483648; % 0x80000000
vel = data(1:3,:);
vel(vel == vel_empty) = NaN;
% velocity
vel = vel / 1000; % [m/s]

% read raw attitude kalman filter output data
fid = fopen("attkf_raw.txt");
data = textscan(fid, '%4xs16');
fclose(fid);
data = data{1,1};
data = data';
data = reshape(data, 4, []);
data = double(data);
% kalman filter attitude
quat_kf = data(1:4,:) / 2^14; % [-]

% delta t
dt = ones(1, count) * 10 / 1000; % [s]
% calculate absolute time
t = cumsum(dt);

% -------------------------------------------------------------------------
% attitude
% -------------------------------------------------------------------------

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

% attitude error
diff = zeros(1, count); % [deg]
% calculate attitude error
for i = 1:count
    diff(i) = angle_between_quat(quat_kf(:,i), quat(:,i)) / pi * 180;
end

% plot attitude error
figure;
plot(t, diff);
title("Attitude Error");
xlabel("time [s]");
ylabel("attitude error [deg]");
legend("attitude error");

% -------------------------------------------------------------------------
% linear acceleration
% -------------------------------------------------------------------------

% plot actual linear acceleration
figure;
plot(t, lin_accel(1,:));
hold on
plot(t, lin_accel(2,:));
plot(t, lin_accel(3,:));
hold off
title("Actual Linear Acceleration");
xlabel("time [s]");
ylabel("acceleration components [m/s^2]");
legend("x", "y", "z");

% kalman filter linear acceleration
lin_accel_kf = zeros(3, count); % [m/s^2]
% calculate linear acceleration
gravity = [0; 0; 9.8053]; % [m/s^2]
for i = 1:count
    lin_accel_kf(:,i) = accel(:,i) - rotate_by_quat(gravity, quat_conj(quat_kf(:,i)));
end

% plot kalman filter linear acceleration
figure;
plot(t, lin_accel_kf(1,:));
hold on
plot(t, lin_accel_kf(2,:));
plot(t, lin_accel_kf(3,:));
hold off
title("Kalman Filter Linear Acceleration");
xlabel("time [s]");
ylabel("acceleration components [m/s^2]");
legend("x", "y", "z");

% linear acceleration error
diff_lin_accel_ang = zeros(1, count); % [deg]
diff_lin_accel_norm = zeros(1, count); % [m/s^2]
% calculate linear acceleration error
for i = 1:count
    s = (lin_accel_kf(:,i)' * lin_accel(:,i)) / (norm(lin_accel_kf(:,i)) * norm(lin_accel(:,i)));
    diff_lin_accel_ang(i) = real(acos(s)) / pi * 180;
    diff_lin_accel_norm(i) = abs(norm(lin_accel_kf(:,i)) - norm(lin_accel(:,i)));
end

% plot linear acceleration error
figure;
yyaxis left
plot(t, diff_lin_accel_ang);
ylabel("linear acceleration angle error [deg]");
hold on
yyaxis right
plot(t, diff_lin_accel_norm);
ylabel("linear acceleration norm error [m/s^2]");
hold off
title("Linear Acceleration Error");
xlabel("time [s]");
legend("linear acceleration angle error", "linear acceleration norm error");

% -------------------------------------------------------------------------
% velocity
% -------------------------------------------------------------------------

% kalman filter velocity
velocity_kf = zeros(3, count); % [m/s]
% rotated gps velocity
vel_rot = NaN(3, count); % [m/s]
% velocity error
diff_vel_ang = NaN(1, count); % [deg]
diff_vel_norm = NaN(1, count); % [m/s]
vel_sample = false(1, count);

% initialize velocity kalman filter
vel_kf();
% run velocity kalman filter
for i = 1:count
    % prediction step (time update)
    vel_kf_predict(lin_accel_kf(:,i));
    % check if the velocity sample is not empty
    if not(isnan(vel(:,i)))
        % valid velocity sample
        vel_sample(i) = true;
        % measurement step (measurement update)
        vel_NED = vel(:,i);
        vel_ENU = [vel_NED(2); vel_NED(1); -vel_NED(3)];
        % correct declination
        phi = 2.9425 / 180 * pi;
        measured_i = rotate_z_by_ang(vel_ENU, phi);
        % rotate measurement from the inertial frame into the body frame
        measured_b = rotate_by_quat(measured_i, quat_conj(quat_kf(:,i)));
        % measure velocity vector
        vel_kf_measure(measured_b);
    end
    % store kalman filter velocity
    velocity_kf(:,i) = vel_kf_vel();
    % check if the velocity sample is not empty
    if vel_sample(i)
        % calculate velocity error
        s = (velocity_kf(:,i)' * measured_b) / (norm(velocity_kf(:,i)) * norm(measured_b));
        diff_vel_ang(i) = real(acos(s)) / pi * 180;
        diff_vel_norm(i) = abs(norm(velocity_kf(:,i)) - norm(measured_b));
        % store rotated gps velocity
        vel_rot(:,i) = measured_b;
    end
end

% plot gps velocity
figure;
plot(t(vel_sample), vel(1,vel_sample));
hold on
plot(t(vel_sample), vel(2,vel_sample));
plot(t(vel_sample), vel(3,vel_sample));
hold off
title("GPS Velocity");
xlabel("time [s]");
ylabel("velocity components [m/s]");
legend("x", "y", "z");

% plot rotated gps velocity
figure;
plot(t(vel_sample), vel_rot(1,vel_sample));
hold on
plot(t(vel_sample), vel_rot(2,vel_sample));
plot(t(vel_sample), vel_rot(3,vel_sample));
hold off
title("Rotated GPS Velocity");
xlabel("time [s]");
ylabel("velocity components [m/s]");
legend("x", "y", "z");

% plot kalman filter velocity
figure;
plot(t, velocity_kf(1,:));
hold on
plot(t, velocity_kf(2,:));
plot(t, velocity_kf(3,:));
hold off
title("Kalman Filter Velocity");
xlabel("time [s]");
ylabel("velocity components [m/s]");
legend("x", "y", "z");

% plot velocity error
figure;
yyaxis left
plot(t(vel_sample), diff_vel_ang(vel_sample));
ylabel("velocity angle error [deg]");
hold on
yyaxis right
plot(t(vel_sample), diff_vel_norm(vel_sample));
ylabel("velocity norm error [m/s]");
hold off
title("Velocity Error");
xlabel("time [s]");
legend("velocity angle error", "velocity norm error");
