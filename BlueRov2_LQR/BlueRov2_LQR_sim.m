clc
clear all
close all

global Tf 

% Control frequency is defined as follows %
s_freq = 100;      % Hz
s_time = 1/s_freq; % sec

% End time
Tf = 10;

% Scaling method
method = 3;

% Desired Position
x = 0;
y = 5;
z = -2;

% Desired Orientation
ph = 0;
th = 0;
ps = 0;

% Desired Linear Velocity
u = 0;
v = 0;
w = 0;

% Desired Angular Velocity
p = 0;
q = 0;
r = 0;

% Initial & Desired value
Initial_eta = [ 0; 0; 0;  0;  0;  0; 0; 0; 0; 0; 0; 0];
Desired_eta = [ x; y; z; ph; th; ps; u; v; w; p; q; r];

% Get LQR Gain
A = [0, 0, 0,            0,          0, 0,           1,          0,       0,          0,        0,    0;
     0, 0, 0,            0,          0, 0,           0,          1,       0,          0,        0,    0;
     0, 0, 0,            0,          0, 0,           0,          0,       1,          0,        0,    0;
     0, 0, 0,            0,          0, 0,           0,          0,       0,          1,        0,    0;
     0, 0, 0,            0,          0, 0,           0,          0,       0,          0,        1,    0;
     0, 0, 0,            0,          0, 0,           0,          0,       0,          0,        0,    1;
     0, 0, 0,            0, 2731/10750, 0, -2821/10750,          0,       0,          0,  -7/2150,    0;
     0, 0, 0,  -2731/15790,          0, 0,           0, -2177/7895,       0,     7/3158,        0,    0;
     0, 0, 0,            0,          0, 0,           0,          0, -74/351,          0,        0,    0;
     0, 0, 0, 225187/31580,          0, 0,           0,   311/1579,       0, -1589/6316,        0,    0;
     0, 0, 0,            0, 30911/4300, 0,   -403/2150,          0,       0,          0, -217/860,    0;
     0, 0, 0,            0,          0, 0,           0,          0,       0,          0,        0, -1/4];
 
B = [     0,        0,        0,         0,      0,    0;
          0,        0,        0,         0,      0,    0;
          0,        0,        0,         0,      0,    0;
          0,        0,        0,         0,      0,    0;
          0,        0,        0,         0,      0,    0;
          0,        0,        0,         0,      0,    0;
     14/215,        0,        0,         0,   2/43,    0;
          0,  70/1579,        0,  -50/1579,      0,    0;
          0,        0, 100/2457,         0,      0,    0;
          0, -50/1579,        0, 5675/1579,      0,    0;
       2/43,        0,        0,         0, 155/43,    0;
          0,        0,        0,         0,      0, 25/7];
      
C = eye(12);
Q = C'*C;
R = 0.0005;
[K,P] = lqr(A,B,Q,R); % P is Riccati Equation Solution
LQR_gain = K; 

% Create desired trajectory
Xstart = eul2rotm(Initial_eta(4:6).');
Xstart = RpToTrans(Xstart, Initial_eta(1:3));
Xend = eul2rotm(Desired_eta(4:6).');
Xend = RpToTrans(Xend, Desired_eta(1:3));
Trajectory = CartesianTrajectory(Xstart, Xend, Tf, method, s_time);

% Get act trajectory
eta_trajectory_LQR = ROV_LQRcontrol(Initial_eta, Trajectory, s_time, LQR_gain);

f1 = figure('Name','ROV LQR control simulation');
Show_Simulation(eta_trajectory_LQR)

function eta_trajectory = ROV_LQRcontrol(Initial_eta, Trajectory, s_time, LQR_gain)

    global Tf M C D g U J
    
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
    
    eta_trajectory = {};
    Current_eta = Initial_eta;

    t = 0;
    i = 0;

    for t_proc = 0 : s_time : Tf - s_time
        
        i = i + 1;
        
        % Trajectory(matrix -> list)
        T = cell2mat( Trajectory(i) );
        [R , p] = TransToRp(T);
        eul = rotm2eul(R).';
        Desired_eta = [p; eul; 0; 0; 0; 0; 0; 0];
 
        % Position & Orientation
        x = Current_eta(1);
        y = Current_eta(2);
        z = Current_eta(3);
        
        ph = Current_eta(4);
        th = Current_eta(5);
        ps = Current_eta(6);

        % Velocity
        u = Current_eta(7);
        v = Current_eta(8);
        w = Current_eta(9);
        
        p = Current_eta(10);
        q = Current_eta(11);
        r = Current_eta(12);

        % Set Error
        Error = Current_eta - Desired_eta;
        Error_X_world = Error(1:6);
        Error_V_body  = Error(7:12);
       
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
           
        % -- [Mass matrix] --
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
        
        M = Mrb + Ma;

        % -- [Coriolis force matrix] --
        % Rigid-Body Coriolis and Centripetal Matrix

        % 1). Largrangian parameterizations
        Crb_L = [ 0,     0,     0,       0,     m*w,    -m*v;
                  0,     0,     0,    -m*w,       0,     m*u;
                  0,     0,     0,     m*v,    -m*u,       0;
                  0,   m*w,  -m*v,       0,    Iz*r,   -Iy*q;
               -m*w,     0,   m*u,   -Iz*r,       0,    Ix*p;
                m*v,  -m*u,     0,    Iy*q,   -Ix*p,       0];

        % 2). Velocity-independent parameterizations
        Crb_V = [ 0,  -m*r,   m*q,       0,       0,       0;
                m*r,     0,  -m*p,       0,       0,       0;
               -m*q,   m*p,     0,       0,       0,       0;
                  0,     0,     0,       0,    Iz*r,   -Iy*q;
                  0,     0,     0,   -Iz*r,       0,    Ix*p;
                  0,     0,     0,    Iy*q,   -Ix*p,       0];

        % Hydrodynamic Coriolis-Centripetal Matrix(Added term)
        Ca = [  0,      0,      0,       0,  -Zwd*w,  Yvd*v;
                0,      0,      0,   Zwd*w,       0, -Xud*u;
                0,      0,      0,  -Yvd*v,   Xud*u,      0;
                0, -Zwd*w,  Yvd*v,       0,  -Nrd*r,  Mqd*q;
            Zwd*w,      0, -Xud*u,   Nrd*r,       0, -Kpd*p;
           -Yvd*v,  Xud*u,      0,  -Mqd*q,    Kpd*p,     0];

        C = Crb_L + Ca;

        % -- [Damping matrix] --
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
                                     
        % Set Body Frame Error (World Frame -> Body Frame)
        Error_X_body = J.'* Error_X_world;
        Error_body = [Error_X_body; Error_V_body];
        
        U = -LQR_gain * Error_body;
        
        X = [x; y; z; ph; th; ps];
        V = [u; v; w;  p;  q;  r];

        % (Word -> Body) 
        X = J.'* X;
        
        State = [X;V];
        
        % Dynamics
        [t,S_list] = ode45(@(t,State) Dynamic_model(t,State), [0  s_time], State);
        State = S_list(end,(1:12)).';
        X = State(1:6);
        V = State(7:12);
    
        % (Body -> World) 
        X = J * X;

        % Update eta
        Current_eta = [X; V]

        eta_trajectory{end+1} = {Current_eta};
    end     
end
      