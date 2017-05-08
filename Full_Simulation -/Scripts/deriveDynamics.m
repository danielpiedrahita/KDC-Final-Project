%symbolic variables
syms M m g l I F;
syms x dx ddx theta dtheta ddtheta yaw dyaw ddyaw;

%Parameters
l = 0.098; %m
m = 5.902; %kg 
M = 0.378; %kg 
Ix = 0.06638; %kg*m^2
Iz = 0.04839; %kg*m^2
g = 9.81;

%state of pole COM
xPole = x + l*sin(theta);
yPole = l*cos(theta);
dxPole = dx + l*cos(theta)*dtheta;
dyPole = -l*sin(theta)*dtheta;

%% Lagrangian %%
%kinetic energy
Tcart = M*dx^2 / 2;
Tpole = ((m*((dxPole^2) + (dyPole^2))) + (Ix*dtheta^2)) / 2;
T = Tcart + Tpole;

%potential energy
Ucart = 0;
Upole = m*g*yPole;
U = Ucart + Upole;

L = simplify(T-U);

%% Lagrangian to State Space
%Define state vectors and parameters
X = {x dx theta dtheta};
Q_i = {0 0}; %internal forces
Q_e = {F 0}; %external forces
R = 0; %dissapation function
par = {M m g l Ix}; %parameters

%Linearize
VF  = EulerLagrange(L,X,Q_i,Q_e,R,par); %vector field representation
states = [x dx theta dtheta];
VFlin = jacobian(VF,states);

%Subsitute in operating point (0,0)
VFlin = subs(VFlin,theta,0);
VFlin = subs(VFlin,dtheta,0);

%Get State Space Representation (symbolic)
Astate = VFlin;
Bstate = diff(VF,F);
Bstate = subs(Bstate,theta,0);
Bstate = subs(Bstate,dtheta,0);

%Get State Space Representation
A = eval(Astate);
B = eval(Bstate);
C = eye(4);
D = 0;

%Rotation State Space Representation [yaw; dyaw]
Arot = [0 1;...
        0 0];
Brot = [0; -1/Iz];
Crot = eye(2);
Drot = 0;

Ddecouple = [0.5 0.5;...
             0.5 -0.5];

%LQR Control Design
sys_balance = ss(A,B,C,D);
sys_turn = ss(Arot,Brot,Crot,Drot);

%Check controllability
if rank(ctrb(sys_balance))==4
    disp('System is controllable!');
else
    disp('System is not controllable :(');
end
if rank(ctrb(sys_turn))==2
    disp('System is controllable!');
else
    disp('System is not controllable :(');
end

%% LQR Controller
%Cost on States
% Q = [10 0 0 0;... %x 10000
%      0 1 0 0; ... %dx 5000
%      0 0 1 0; ... %theta 1
%      0 0 0 1]; %dtheta 1
Q = [30000 0 0 0;... %x 10000
     0 3000 0 0; ... %dx 5000
     0 0 10 0; ... %theta 1
     0 0 0 1]; %dtheta 1

Qrot = [50 0;... %yaw
        0 10]; ... %dyaw
%Cost on Inputs
R = 10;
Rrot = 10;

%Find Controller!
Kbalance = -lqr(sys_balance,Q,R);
Kbalancediscrete = -lqrd(A,B,Q,R,0.02)
Kturn = -lqr(sys_turn,Qrot,Rrot);
Kturndiscrete = -lqrd(Arot,Brot,Qrot,Rrot,0.02)