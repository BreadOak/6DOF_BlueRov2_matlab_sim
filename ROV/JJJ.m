
syms x y z ph th ps u v w p q r
syms u1 u2 u3 u4 u5 u6 d
assume([x y z ph th ps u v w p q r],"real")
assume([u1 u2 u3 u4 u5 u6 d],"real")

m = 10;      % kg
W = 98.1;    % N
B = 100.6;   % N
Ix = 0.16;   % kg*m^2
Iy = 0.16;   % kg*m^2
Iz = 0.16;   % kg*m^2

rg = [0, 0, 0];    % m
rb = [0, 0, 0.02]; % m
BG = rg - rb;      % m
xg = BG(1);        % m
yg = BG(2);        % m
zg = BG(3);        % m

Xud = -5.5;   % kg
Yvd = -12.7;  % kg
Zwd = -14.57; % kg
Kpd = -0.12;  % kg*m^2/rad
Mqd = -0.12;  % kg*m^2/rad
Nrd = -0.12;  % kg*m^2/rad

Xu = -4.03;   % N*s/m
Yv = -6.22;   % N*s/m
Zw = -5.18;   % N*s/m
Kp = -0.07;   % N*s/rad
Mq = -0.07;   % N*s/rad
Nr = -0.07;   % N*s/rad

Xuu = -18.18; % N*s^2/m^2
Yvv = -21.66; % N*s^2/m^2
Zww = -36.99; % N*s^2/m^2
Kpp = -1.55;  % N*s^2/rad^2
Mqq = -1.55;  % N*s^2/rad^2
Nrr = -1.55;  % N*s^2/rad^2

% -- [Mass matrix] --
% Rigid-Body System Inertia Matrix
Mrb = [m,     0,     0,     0;
       0,     m,     0,     0;
       0,     0,     m,     0;
       0,     0,     0,    Iz];  

% Hydrodynamic System Inertia Matrix(Added term)
Ma  = - [Xud,   0,   0,   0;
           0, Yvd,   0,   0;
           0,   0, Zwd,   0;
           0,   0,   0, Nrd];

M = Mrb + Ma;

% -- [Coriolis force matrix] --
% Rigid-Body Coriolis and Centripetal Matrix

% 1). Largrangian parameterizations
Crb_L = [ 0,     0,     0,  -m*v;
          0,     0,     0,   m*u;
          0,     0,     0,     0;
        m*v,  -m*u,     0,     0];

% 2). Velocity-independent parameterizations
Crb_V = [ 0,  -m*r,     0,     0;
        m*r,     0,     0,     0;
          0,     0,     0,     0;
          0,     0,     0,     0];

% Hydrodynamic Coriolis-Centripetal Matrix(Added term)
Ca = [  0,      0,      0,  Yvd*v;
        0,      0,      0, -Xud*u;
        0,      0,      0,      0;
   -Yvd*v,  Xud*u,      0,      0];

C = Crb_L + Ca;

% -- [Damping matrix] --
% Linear Part
Dl = - [Xu,   0,   0,   0;
         0,  Yv,   0,   0;
         0,   0,  Zw,   0;
         0,   0,   0,  Nr];

% Nonlinear Part
Dnl = - [Xuu*abs(u),           0,           0,           0;
                  0,  Yvv*abs(v),           0,           0;
                  0,           0,  Zww*abs(w),           0;
                  0,           0,           0,  Nrr*abs(r)];

D = Dl + Dnl;

% -- [Resorting Force] --
g = [       0;
            0; 
     -(W - B);
            0];
                             
% Transformation Matrix of linear velocity
R = [cos(ps), -sin(ps),        0;
     sin(ps),  cos(ps),        0;
           0,        0,        1];

% Transformation of angular velocity
T = 1;

% Transformation Matrix (Body -> World)
J = [         R,  zeros(3,1);
       zeros(1,3),         T];
                       
% Control input
U = [u1;u2;u3;u4];

X = [x; y; z; ps; u; v; w; r];
V = [u; v; w; r];

% Dynamics
f = [J*V; inv(M)*(U - (C+D)*V - g)];

A = jacobian(f,X);
B = jacobian(f,U)

% u01 = 0;
% u02 = 0;
% u03 = 0;
% u04 = 0;
% 
% x01 = 0;
% x02 = 0;
% x03 = 0;
% x04 = 0;
% x05 = 0;
% x06 = 0;
% x07 = 0;
% x08 = 0;
% 
% Equilibrium_point = [x01, x02, x03, x04, x05, x06, x07, x08, u01, u02, u03, u04];
% 
% A = subs(A,{x y z ps u v w r u1 u2 u3 u4},Equilibrium_point);
% B = subs(B,{x y z ps u v w r u1 u2 u3 u4},Equilibrium_point);
