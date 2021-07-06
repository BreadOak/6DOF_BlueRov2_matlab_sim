function [s, ds, dds] = CubicTimeScaling(Tf, t)
    s = 3 * (t / Tf) ^ 2 - 2 * (t / Tf) ^ 3;
    ds = (6 * t) / Tf^2 - (6 * t^2) / Tf^3;
    dds = 6 / Tf^2 - (12 * t) / Tf^3;
end