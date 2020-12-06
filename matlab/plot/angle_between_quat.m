function r = angle_between_quat(q1, q2) %#codegen
%ANGLE_BETWEEN_QUAT angle of rotation between two quaternions

u = q1(1)*q2(1) + q1(2)*q2(2) + q1(3)*q2(3) + q1(4)*q2(4);
v = (u.^2).*2-1;
if v > 1
    v = 1;
elseif v < -1
    v = -1;
end
r = acos(v);
end
