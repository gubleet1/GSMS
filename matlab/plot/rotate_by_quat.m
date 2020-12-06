function v_rot = rotate_by_quat(v, q) %#codegen
%ROTATE_BY_QUAT rotate 3 element vector by quaternion

qv = [0; v];
qv_rot = quat_mult(quat_mult(q, qv), quat_conj(q));
v_rot = qv_rot(2:4);
end
