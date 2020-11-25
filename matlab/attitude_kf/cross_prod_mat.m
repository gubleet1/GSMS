function m = cross_prod_mat(v) %#codegen
%CROSS_PROD_MAT cross product matrix of 3 element vector

m = [0, -v(3), v(2);
     v(3), 0, -v(1);
     -v(2), v(1), 0];
end
