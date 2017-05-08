%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #1                         %
%  EFK-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
figure;
ylim([0 1]);
%==== TEST: Setup uncertianty parameters (try different values!) ===
sig_ultrasonic = 0.08;
sig_imu = 2.5*pi/180;
sig_enc = 0.005;

%==== Generate sigma^2 from sigma ===
sig_ultrasonic = sig_ultrasonic^2;
sig_imu = sig_imu^2;
sig_enc = sig_enc^2;

%==== dimension of vehicle ===
wheel_base = 10*0.0254;
track_width = 10.5*0.0254; 

%==== Open data file ====
fid = fopen('./Datasets/data_kalman_filter_straight_path.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_enc, sig_enc]);
measure_cov = diag([sig_ultrasonic, sig_ultrasonic, sig_ultrasonic, sig_ultrasonic, sig_imu]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0.4 ; 0];
pose_cov = diag([0.0^2, 0.02^2, 0.1^2]);

%%
%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====

% Write your code here...

% H = zeros(12,3);
k = 0;
landmark = zeros(2*k,1);
landmark_cov = zeros(2*k,2*k);
for i = 1:k
    landmark(2*i-1:2*i) = [pose(1) + arr(2*i)*cos(arr(2*i-1) + pose(3) + 0);...
        pose(2) + arr(2*i)*sin(arr(2*i-1) + pose(3) + 0)];
    A_land = [1 0 -arr(2*i)*sin(arr(2*i-1) + pose(3)); 0 1 arr(2*i)*cos(arr(2*i-1) + pose(3))];
    B_land = [-arr(2*i)*sin(arr(2*i-1) + pose(3)), cos(arr(2*i-1) + pose(3)) ;...
         arr(2*i)*cos(arr(2*i-1) + pose(3)), sin(arr(2*i-1)+ pose(3))];
    landmark_cov(2*i-1:2*i,2*i-1:2*i) = A_land*pose_cov*A_land'  + B_land*measure_cov*B_land';
end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0, 0);

x_total = x;
%%
%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    motion_left = arr(1)*2*3.14*6/2*0.0254;
    motion_right = arr(2)*2*3.14*6/2*0.0254;
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    % Write your code here...
    A = eye(3); %[1 0 -d*sin(last_x(3)); 0 1 d*cos(last_x(3)); 0 0 1];
    rel_disp = (motion_right - motion_left)/track_width;
    B = [cos(last_x(3))/2, cos(last_x(3))/2;...
        sin(last_x(3))/2, sin(last_x(3))/2;...
        -1/sqrt(1-rel_disp^2)/track_width, 1/sqrt(1-rel_disp^2)/track_width];
    
    x_pre = last_x + [eye(3); zeros(2*k, 3)]*...
        [(motion_left + motion_right)*cos(last_x(3))/2;...
        (motion_left + motion_right)*sin(last_x(3))/2;...
        asin(rel_disp)];
    
    P_pre = P;
    
    P_pre(1:3,1:3) = A*P(1:3,1:3)*A' + B*control_cov*B';
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
        
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    distance_left = arr(1)/100;
    distance_right = arr(2)/100;
    yaw_angle = wrapTo180(arr(5))*pi/180;
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...
    Z = [];
    Z_obs = [];
    Z = [x_pre(2) + distance_left*cos(x_pre(3)) + track_width/2*cos(x_pre(3));...
        x_pre(2) - distance_right*cos(x_pre(3)) - track_width/2*cos(x_pre(3));...
        x_pre(3) - yaw_angle];
%     Z = [2*x_pre(2) + distance_left*cos(x_pre(3)) - distance_right*cos(x_pre(3));...
%         x_pre(3) - yaw_angle];
%         x_pre(1) + distance_front*cos(x_pre(3));...
%         x_pre(1) - distance_back*cos(x_pre(3));...
        
    Z_obs = [0.80; 0; 0];
%     Z_obs = [0.80-track_width, 0]';
%     for i = 1:k
%         delta = [x_pre(3+2*(i-1)+1) - x_pre(1); x_pre(3+2*i) - x_pre(2)];
%         q = delta'*delta;
%         z = [wrapToPi(atan2(delta(2), delta(1)) - x_pre(3));sqrt(q)];
%         F = [eye(3),zeros(3, 2*k);zeros(2,3+2*i-2), eye(2), zeros(2,2*k-2*i)];
%         H(2*i-1:2*i,1:3) = 1/q*[delta(2), -delta(1), -q;...
%             -sqrt(q)*delta(1), -sqrt(q)*delta(2), 0];
%         H(2*i-1:2*i, 3+2*i-1:3+2*i) = 1/q*[-delta(2), delta(1);...
%             sqrt(q)*delta(1), sqrt(q)*delta(2)];
%         measurement_cov(2*i-1: 2*i,2*i-1: 2*i) = measure_cov;
%         Z = [Z;z];
%         Z_obs = [Z_obs;[arr(2*(i-1)+1), arr(2*i)]'];
%     end

%     C = [0 1 -distance_left*sin(x_pre(3)); 0 1 distance_right*sin(x_pre(3));...
%         1 0 -distance_front*sin(x_pre(3)); 1 0 distance_back*sin(x_pre(3));...
%         0 0 0];
%     D = [cos(x_pre(3)) 0 0 0 0; 0 -cos(x_pre(3)) 0 0 0; 0 0 cos(x_pre(3)) 0 0;...
%         0 0 0 cos(x_pre(3)) 0; 0 0 0 0 1];

    C = [0 1 -(distance_left + track_width/2)*sin(x_pre(3));...
        0 1 (distance_right + track_width/2)*sin(x_pre(3));...
        0 0 1];
    D = [cos(x_pre(3)) 0 0 0 0;...
        0 -cos(x_pre(3)) 0 0 0;...
        0 0 0 0 -1];
    K = P_pre * C' / (C * P_pre * C' + D*measure_cov*D');
    x = x_pre + K*(Z_obs - Z);
    P = (eye(3) - K*C)*P_pre;

    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t,0);
    last_x = x;
    x_total = [x_total, x];
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end
figure;
subplot(2,2,1);
plot(x_total(1,:),x_total(2,:));
legend('trajectory XY plot');

subplot(2,2,2);
plot(x_total(1,:));
legend('Pose X versus time');

subplot(2,2,3);
plot(x_total(2,:));
legend('Pose Y versus time');

subplot(2,2,4);
plot(x_total(3,:));
legend('pose angle versus time');
%%
%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
landmark_truth = [6 6 6 12 10 6 10 12 14 6 14 12];
hold on;
scatter([6 6 10 10 14 14], [6 12 6 12 6 12], 20, 'filled');

for i = 1:k
    euclidean(i) = norm([landmark_truth(2*(i-1)+1)-x(3+2*(i-1)+1);...
        landmark_truth(2*i)-x(3+2*i)]);
    Mahalanobis(i) = sqrt([landmark_truth(2*(i-1)+1)-x(3+2*(i-1)+1);...
        landmark_truth(2*i)-x(3+2*i)]'/P(3+2*(i-1)+1:3+2*i,3+2*(i-1)+1:3+2*i)*...
        [landmark_truth(2*(i-1)+1)-x(3+2*(i-1)+1);...
        landmark_truth(2*i)-x(3+2*i)]);
end
euclidean
Mahalanobis

%==== Close data file ====
fclose(fid);
