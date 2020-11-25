function att_kf_propagate() %#codegen
%ATT_KF_PROPAGATE propagate attitude error

global att_kf_q_ref; % reference attitude quaternion q_ref
global att_kf_x; % state vector x

% calculate rotation quaternion from attitude error vector a
delta_q_of_a = [2; att_kf_x(1); att_kf_x(2); att_kf_x(3)];
% propagate attitude error to reference attitude quaternion q_ref
att_kf_q_ref = quat_mult(att_kf_q_ref, delta_q_of_a);
% normalize reference attitude quaternion q_ref
att_kf_q_ref = att_kf_q_ref / norm(att_kf_q_ref);
% reset attitude error vector a
att_kf_x(1:3) = zeros(3, 1);
end
