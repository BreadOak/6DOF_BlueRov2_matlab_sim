function traj = CartesianTrajectory(Xstart, Xend, Tf, method, s_time)

    N = Tf/s_time;
    timegap = Tf / (N - 1);
    traj = cell(1, N);
    [Rstart, pstart] = TransToRp(Xstart);
    [Rend, pend] = TransToRp(Xend);
    
    for i = 1: N
        if method == 3
            S = CubicTimeScaling(Tf,timegap * (i - 1));
            s = S(1);
        else
            s = QuinticTimeScaling(Tf,timegap * (i - 1));
        end
        traj{i} ...
        = [Rstart * MatrixExp3(MatrixLog3(Rstart' * Rend) * s), ...
           pstart + s * (pend - pstart); 0, 0, 0, 1];
    end
    
end