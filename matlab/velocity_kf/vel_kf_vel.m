function vel = vel_kf_vel() %#codegen
%VEL_KF_VEL velocity

global vel_kf_x; % state vector x

% return velocity
vel = vel_kf_x;
end
