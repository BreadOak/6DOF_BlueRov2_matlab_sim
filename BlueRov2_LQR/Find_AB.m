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

% Rigid-Body System Inertia Matrix
Mrb = [m,     0,     0,     0,  m*zg,     0;
       0,     m,     0, -m*zg,     0,     0;
       0,     0,     m,     0,     0,     0;
       0, -m*zg,     0,    Ix,     0,     0;
    m*zg,     0,     0,     0,    Iy,     0;
       0,     0,     0,     0,     0,    Iz];  
% Hydrodynamic System Inertia Matrix(Added term)
Ma  = - [Xud,   0,   0,   0,   0,   0;
           0, Yvd,   0,   0,   0,   0;
           0,   0, Zwd,   0,   0,   0;
           0,   0,   0, Kpd,   0,   0;
           0,   0,   0,   0, Mqd,   0;
           0,   0,   0,   0,   0, Nrd];
M = Ma+Mrb;

% Hydrodynamic Coriolis-Centripetal Matrix(Added term)
Ca = [  0,      0,      0,       0,  -Zwd*w,  Yvd*v;
        0,      0,      0,   Zwd*w,       0, -Xud*u;
        0,      0,      0,  -Yvd*v,   Xud*u,      0;
        0, -Zwd*w,  Yvd*v,       0,  -Nrd*r,  Mqd*q;
    Zwd*w,      0, -Xud*u,   Nrd*r,       0, -Kpd*p;
   -Yvd*v,  Xud*u,      0,  -Mqd*q,    Kpd*p,     0];
% 1). Largrangian parameterizations
Crb_L = [ 0,     0,     0,       0,     m*w,    -m*v;
          0,     0,     0,    -m*w,       0,     m*u;
          0,     0,     0,     m*v,    -m*u,       0;
          0,   m*w,  -m*v,       0,    Iz*r,   -Iy*q;
       -m*w,     0,   m*u,   -Iz*r,       0,    Ix*p;
        m*v,  -m*u,     0,    Iy*q,   -Ix*p,       0];
C = Crb_L + Ca;

% Linear Part
Dl = - [Xu,   0,   0,   0,  0,  0;
         0,  Yv,   0,   0,  0,  0;
         0,   0,  Zw,   0,  0,  0;
         0,   0,   0,  Kp,  0,  0;
         0,   0    0,   0, Mq,  0;
         0,   0,   0,   0,  0, Nr];

% Nonlinear Part
Dnl = - [Xuu*abs(u),           0,           0,           0,           0,           0;
                  0,  Yvv*abs(v),           0,           0,           0,           0;
                  0,           0,  Zww*abs(w),           0,           0,           0;
                  0,           0,           0,  Kpp*abs(p),           0,           0;
                  0,           0,           0,           0,  Mqq*abs(q),           0;
                  0,           0,           0,           0,           0,  Nrr*abs(r)];
D = Dl + Dnl;

% -- [Resorting Force] --
g = [ (W - B) * sin(th)           ;
     -(W - B) * cos(th) * sin(ph) ; 
     -(W - B) * cos(th) * cos(ph) ;
      zg * W  * cos(th) * sin(ph) ;
      zg * W  * sin(th)           ; 
                                 0];
                             
% Transformation Matrix of linear velocity
R = [cos(ps)*cos(th), -sin(ps)*cos(ph)+sin(ph)*sin(th)*cos(ps),  sin(ps)*sin(ph)+sin(th)*cos(ps)*cos(ph);
     sin(ps)*cos(th),  cos(ps)*cos(ph)+sin(ph)*sin(th)*sin(ps), -cos(ps)*sin(ph)+sin(th)*sin(ps)*cos(ph);
            -sin(th),                          sin(ph)*cos(th),                          cos(ph)*cos(th)];

% Transformation Matrix of angular velocity
T = [1,  sin(ph)*tan(th),  cos(ph)*tan(th);
     0,          cos(ph),         -sin(ph);
     0,  sin(ph)/cos(th),  cos(ph)/cos(th)];

% Transformation Matrix (Body -> World)
J = [         R,  zeros(3);
       zeros(3),         T];
                                                 
U = [u1;u2;u3;u4;u5;u6];
X = [x; y; z; ph; th; ps; u; v; w;  p;  q;  r];
V = [u; v; w;  p;  q;  r];

f1 = [J*V ;inv(M)*(U + d - C*V - D*V - g)];
f2 = [inv(M)*(U + d - C*V - D*V - g)];

A = jacobian(f2,X);
B = jacobian(f2,U);

u01 = 0;
u02 = 0;
u03 = 0;
u04 = 0;
u05 = 0;
u06 = 0;

x01 = 0;
x02 = 0;
x03 = 0;
x04 = 0;
x05 = 0;
x06 = 0;
x07 = 0;
x08 = 0;
x09 = 0;
x10 = 0;
x11 = 0;
x12 = 0;

Equilibrium_point = [x01, x02, x03, x04, x05, x06, x07, x08, x09, x10, x11, x12, u01, u02, u03, u04, u05, u06];

A = subs(A,{x y z ph th ps u v w p q r u1 u2 u3 u4 u5 u6},Equilibrium_point)
B = subs(B,{x y z ph th ps u v w p q r u1 u2 u3 u4 u5 u6},Equilibrium_point)
