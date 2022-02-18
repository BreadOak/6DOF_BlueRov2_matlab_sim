clc
clear all
close all

global Tf

% Control frequency is defined as follows %
s_freq = 100;      % Hz
s_time = 1/s_freq; % sec

% End time
Tf = 10;

% Desired Position
x = 4;
y = 3;
z = -2;

% Desired Orientation
ps = 0;

method = 3; % CubicTimeScailing

% Initial & Desired value
Initial_X = [0; 0; 0; 0; 0;  0];
Desired_X = [x; y; z; 0; 0; ps];

% Create desired trajectory
Xstart = eul2rotm(Initial_X(4:6).');
Xstart = RpToTrans(Xstart, Initial_X(1:3));
Xend = eul2rotm(Desired_X(4:6).');
Xend = RpToTrans(Xend, Desired_X(1:3));
Desired_trajectory = CartesianTrajectory(Xstart, Xend, Tf, method, s_time);

% Simulation
%[Input_trajectory, Output_trajectory] = Controller_PD(Initial_X, Desired_trajectory, s_time);
[Input_trajectory, Output_trajectory] = Controller_LQR(Initial_X, Desired_trajectory, s_time);

% Plot
time = 0 : s_time : Tf - s_time;
I = ones(1,length(time));

inTx  = [];
inTy  = [];
inTz  = [];
inTps = [];

outTx  = [];
outTy  = [];
outTz  = [];
outTps = [];

for inT = Input_trajectory
    inT_cell = inT{1};
    inT_list = inT_cell{1};
    inTx(end+1)  = inT_list(1);
    inTy(end+1)  = inT_list(2);
    inTz(end+1)  = inT_list(3);
    inTps(end+1) = inT_list(4);
end

for outT = Output_trajectory
    outT_cell = outT{1};
    outT_list = outT_cell{1};
    outTx(end+1)  = outT_list(1);
    outTy(end+1)  = outT_list(2);
    outTz(end+1)  = outT_list(3);
    outTps(end+1) = outT_list(4);
end

f1 = figure;
plot3(inTx, inTy, inTz,'-.r')
hold on
plot3(outTx, outTy, outTz,'b')
xlabel('X') 
ylabel('Y')
zlabel('Z')
xlim([0 10])
ylim([0 10])
zlim([-10 5])
grid on

f3 = figure;
subplot(4,1,1)
plot(time, x*I - outTx)
ylim([-10 10])
grid on
xlabel('time') 
ylabel('err X')
subplot(4,1,2)
plot(time, y*I - outTy)
ylim([-10 10])
grid on
xlabel('time') 
ylabel('err Y')
subplot(4,1,3)
plot(time, z*I - outTz)
ylim([-10 10])
grid on
xlabel('time') 
ylabel('err Z')
subplot(4,1,4)
plot(time, ps*I - outTps)
ylim([-pi pi])
grid on
xlabel('time') 
ylabel('err ps')