function [omghat, theta] = AxisAng3(expc3)
    theta = norm(expc3);
    omghat = expc3 / theta;
end
