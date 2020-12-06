function v_rot = rotate_z_by_ang(v, ang) %#codegen
%ROTATE_Z_BY_ANG rotate 3 element vector around z axis by angle

Rz = [cos(ang) -sin(ang) 0; sin(ang) cos(ang) 0; 0 0 1];
v_rot = Rz * v;
end
