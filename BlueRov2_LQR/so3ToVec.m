function omg = so3ToVec(so3mat)
    omg = [so3mat(3, 2); so3mat(1, 3); so3mat(2, 1)];
end