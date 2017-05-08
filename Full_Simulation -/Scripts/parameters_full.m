clc;

%Parameters
simTime = 10.0;
table_height = 1.25; %m 1.25
table_y = 0; %m
ground_y = -5; %m
g = 9.81;
wheelSpeed = 10*2*pi; %rad/sec
motorSaturation = 0.686382838199676; %N-m
wheel_damping = 0.001;
static_friction = 0.9;
kinetic_friction = 0.7;

%Initial Conditions
theta_initial = 90; %deg
dtheta_initial = 0;
x_initial = 0;
dx_initial = 0;
initial_height = table_height+(2.2*0.0254)+0.05; %meters 2.2

%Contact Params
ball.rad = 0.2;
table.l = 5;
table.h = 0.01;
table.w = 5;
ballTable.friction.muk = 0.5;
ballTable.friction.mus = 0.8;

%Control Params
state_desired = [0 0 -0.15 0 0 0]'; %x dx theta dtheta yaw dyaw
P = 1.0;
I = 0.01;
D = 0.2;
Kflight = [200 20];

%Runtime Inputs
t = linspace(0,simTime,simTime*100)';
cutOff = 250;
cutOff2 = 550;
for i=1:cutOff
    x_des(i) = 0;
    yaw_des(i) = 0;
end
for i=cutOff+1:cutOff2
    x_des(i) = x_des(i-1)+0.015;
    yaw_des(i) = 0;
end
for i=cutOff2+1:simTime*100
    x_des(i) = x_des(i-1);
    yaw_des(i) = 0;
end
x_des_input = [t x_des'];
yaw_des_input = [t yaw_des'];
