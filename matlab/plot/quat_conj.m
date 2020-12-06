function qc = quat_conj(q) %#codegen
%QUAT_CONJ quaternion conjugate

qc = [q(1); -q(2); -q(3); -q(4)];
end
