clear;
clc;
close all;

open_system('acceleration_lqr_controller');
mdlWks = get_param('acceleration_lqr_controller','ModelWorkspace');
clear(mdlWks);

%% System Parameters

TUNE = false;

mass = [0.736, 0, 0;
        0, 0.736, 0;
        0, 0, 0.736];
    
inertia = [0.0092, 0, 0;
            0, 0.01134, 0;
            0, 0, 0.0025];

Ixx = inertia(1);
Iyy = inertia(5);
Izz = inertia(9);
        
wing_area = 0.000124; %m^2
rho = 1.1455; %kg/m^3

l_l = 0.154; % distance from rotor to x axis
l_s = 0.19; % distance from rotor to y axis
l_a = 0; % distance from center of lift to wing rotation axis

n = [1, -1, 1, -1]; % sequence of motor torque/force directions
lambda = 0.01; % torque generated from rotor speed
k = 20; % rotor speed to generated force
J_prop = 0.3; % inertia of the propellor

alpha = pi/2;
beta = pi/2;
gamma = [alpha; alpha; beta; beta];

m = 0.736; % mass

%% VTOL GAINS

% Q_x = [15,0;  
%        0,0.5]; 
% R_x = 0.1;
% 
% Q_y = [15,0;
%        0,0.5];
% R_y = 0.1;
% 
% Q_z = [100, 0;
%         0, 10];
% R_z = 0.5;
% 
% Q_roll = [10000, 0;
%            0, 0.1];
% R_roll = 0.1;
%        
% Q_pitch = [10000, 0;
%             0, 0.01];
% R_pitch = 0.1;
%         
% Q_yaw = [10, 0;
%           0, 10];
% R_yaw = 1;


%% TILT ROTOR GAINS

% Q_x = [15,0;  
%        0,0.5]; 
% R_x = 1;
% 
% Q_y = [15,0;
%        0,0.5];
% R_y = 50;
% 
% Q_z = [100, 0;
%         0, 10];
% R_z = 0.5;
% 
% Q_roll = [10000, 0;
%            0, 0.1];
% R_roll = 0.1;
%        
% Q_pitch = [10000, 0;
%             0, 0.01];
% R_pitch = 0.1;
%         
% Q_yaw = [10, 0;
%           0, 10];
% R_yaw = 1;

% ax_gain_plane = 1;
% ay_gain_plane = 1;
% az_gain_plane = 1;
% aphi_gain_plane = 1;
% atheta_gain_plane = 1;
% apsi_gain_plane = 1;
% 
% plane_gains = [ax_gain_plane, 0, 0, 0, 0, 0;
%                0, ay_gain_plane, 0, 0, 0, 0;
%                0, 0, az_gain_plane, 0, 0, 0;
%                0, 0, 0, aphi_gain_plane, 0, 0;
%                0, 0, 0, 0, atheta_gain_plane, 0;
%                0, 0, 0, 0, 0, apsi_gain_plane];

%% MIXED CONTROLLER GAINS

Q_x = [15,0;  
       0,0.5]; 
R_x = 5;

Q_y = [15,0;
       0,0.5];
R_y = 5;

Q_z = [250, 0;
        0, 1];
R_z = 0.05;

Q_roll = [50, 0;
           0, 1];
R_roll = 0.001;
       
Q_pitch = [50, 0;
            0, 1];
R_pitch = 0.001;
        
Q_yaw = [10, 0;
          0, 10];
R_yaw = 1;

%% LQR Gains
% s = [x, dx, y, dy, z, dz, phi, phi_dot, theta, theta_dot, psi, psi_dot]

Q_att = [Q_roll, zeros(2,4);
         zeros(2,2), Q_pitch, zeros(2,2);
         zeros(2,4), Q_yaw];

R_att = [R_roll, 0, 0;
         0, R_pitch, 0;
         0, 0, R_yaw];

Q_acc = [Q_x, zeros(2,10);
         zeros(2,2), Q_y, zeros(2,8);
         zeros(2,4), Q_z, zeros(2,6);
         zeros(6,6), Q_att];

R_acc = [R_x, 0, 0, 0, 0, 0;
         0, R_y, 0, 0, 0, 0;
         0, 0, R_z, 0, 0, 0;
         zeros(3,3), R_att];


Q_g = [1,0,0,0;
       0,0.5,0,0;
       0,0,1,0;
       0,0,0,0.5];

R_g = 0.5;


Q_t_p = [100,0;
         0,100];

R_t_p = 0.1;


Q_g_p = [10,0;
         0,10];

R_g_p = 0.01;

%% LQR K Matricies

% Acceleration
A_acc = [0,1,0,0,0,0,0,0,0,0,0,0;
         0,0,0,0,0,0,0,0,0,0,0,0;
         0,0,0,1,0,0,0,0,0,0,0,0;
         0,0,0,0,0,0,0,0,0,0,0,0;
         0,0,0,0,0,1,0,0,0,0,0,0;
         0,0,0,0,0,0,0,0,0,0,0,0;
         0,0,0,0,0,0,0,1,0,0,0,0;
         0,0,0,0,0,0,0,0,0,0,0,0;
         0,0,0,0,0,0,0,0,0,1,0,0;
         0,0,0,0,0,0,0,0,0,0,0,0;
         0,0,0,0,0,0,0,0,0,0,0,1;
         0,0,0,0,0,0,0,0,0,0,0,0];

B_acc = [0,0,0,0,0,0;
         1/m,0,0,0,0,0;
         0,0,0,0,0,0;
         0,1/m,0,0,0,0;
         0,0,0,0,0,0;
         0,0,1,0,0,0;
         0,0,0,0,0,0;
         0,0,0,1,0,0;
         0,0,0,0,0,0;
         0,0,0,0,1,0;
         0,0,0,0,0,0;
         0,0,0,0,0,1];

C_acc = [1,0,0,0,0,0,0,0,0,0,0,0;
         0,0,1,0,0,0,0,0,0,0,0,0;
         0,0,0,0,1,0,0,0,0,0,0,0;
         0,0,0,0,0,0,1,0,0,0,0,0;
         0,0,0,0,0,0,0,0,1,0,0,0;
         0,0,0,0,0,0,0,0,0,0,1,0];

K_acc = lqr(A_acc, B_acc, Q_acc, R_acc);
k_r_acc = -inv(C_acc/(A_acc-B_acc*K_acc)*B_acc);

% Roll
A_roll = [0,1;
          0,0];
B_roll = [0;1];
C_roll = [1,0];

K_roll = lqr(A_roll, B_roll, Q_roll, R_roll);
k_r_roll = -inv(C_roll/(A_roll-B_roll*K_roll)*B_roll);

% Attitude
A_attitude = [0,1,0,0,0,0;
              0,0,0,0,0,0;
              0,0,0,1,0,0;
              0,0,0,0,0,0;
              0,0,0,0,0,1;
              0,0,0,0,0,0];
B_attitude = [0,0,0;
              1,0,0;
              0,0,0;
              0,1,0;
              0,0,0;
              0,0,1];
C_attitude = [1,0,0,0,0,0;
              0,0,1,0,0,0;
              0,0,0,0,1,0];
          
K_attitude = lqr(A_attitude, B_attitude, Q_att, R_att);
k_r_attitude = -inv(C_attitude/(A_attitude-B_attitude*K_attitude)*B_attitude);

%% Trajectory

% TEST = false;
TEST = true;

wave = 0;

if TEST
% [x,...;
%  y,...;
%  z,...;
%  phi,...;
%  theta,...;
%  psi,...];
%     trajectory = [0, 0, 0;
%                   0, 0, 0;
%                   0,-3, -3;
%                   0, 0, 0;
%                   0, 0, 0;
%                   0, 0, 0];
%     trajectory_time = [0,2,2.9];

    trajectory = [0, 1;
                  0, 0;
                  0, 0;
                  0, 0;
                  0, 0;
                  0, 0];
    trajectory_time = [0,1];

    velocity_bc = zeros(size(trajectory));
    acceleration_bc = zeros(size(trajectory));
    
    simOut = sim('acceleration_lqr_controller', 10);

else

    l_h = -3; % lift height
    x_p_1 = 20; % first x position
    y_p_1 = 0; % first y position
    
    radius = 2; % circle radius
    
    x_p_2 = x_p_1 + 2; % second x position
    y_p_2 = 2; % second y position
    
    lift_vector = [0;0;l_h;0;0;0]; % lift vector
    t_p_1 = [x_p_1;y_p_1;l_h;0;0;0]; % first target point
    t_p_2 = [x_p_2;y_p_2;l_h;0;0;deg2rad(45)]; % second target point
    
    final_vector = [x_p_2;y_p_2;0;0;0;deg2rad(45)];
    
    time_span = 10;
    steps = 100;
    [P, V, A, T] = generate_circle(radius, t_p_1, time_span, steps);

    trajectory = [zeros(6,1), lift_vector, t_p_1, P, t_p_2, final_vector];

    lift_time = 2;
    p_1_time = 2.1;
    circle_time = 6;
    p_2_time = 18;
    land_time = 20;
    
    trajectory_time = [0, lift_time, p_1_time, circle_time + T, p_2_time, land_time];
    
    velocity_bc = [zeros(6,1), zeros(6,1), zeros(6,1), V, zeros(6,1), zeros(6,1)]; 
    acceleration_bc = [zeros(6,1), zeros(6,1), zeros(6,1), A, zeros(6,1), zeros(6,1)]; 

    V_t = 0;
    
    simOut = sim('acceleration_lqr_controller', 40);
end



%% Generate circle
function [P, V, A, T] = generate_circle(radius, starting_pos, time_span, steps)

x = starting_pos(1);
y = starting_pos(2);
z = starting_pos(3);

th = 0:(2*pi)/steps:2*pi;
x_circle = radius * cos(th) + (x - radius);
y_circle = radius * sin(th) + y;

vx_circle = -radius * sin(th);
vy_circle = radius * cos(th);

ax_circle = -radius * cos(th);
ay_circle = -radius * sin(th);

P = [x_circle;
     y_circle;
     z*ones(1,steps+1);
     zeros(1,steps+1);
     zeros(1,steps+1);
     zeros(1,steps+1)];

V = [vx_circle;
     vy_circle;
     zeros(1,steps+1);
     zeros(1,steps+1);
     zeros(1,steps+1);
     zeros(1,steps+1)];
 
A = [ax_circle;
     ay_circle;
     zeros(1,steps+1);
     zeros(1,steps+1);
     zeros(1,steps+1);
     zeros(1,steps+1)];
 
T = 0:time_span/steps:time_span;

end

%% Find Gains

% if TUNE
%     delta = 10;
%     
%     Q_z_i = Q_z;
%     R_z_i = R_z;
%     
%     Q_p_i = Q_p;
%     R_p_i = R_p;
%     
%     Q_a_i = Q_a;
%     R_a_i = R_a;
%     
%     Simulink.sdi.view
%     
%     Simulink.sdi.enablePCTSupport('local');
%     
%     parfor i = 1:10
%         for j = 1:10
%             %         Q_z = Q_z_i + j*delta*eye(2);
%             %         R_z = R_z_i + i*delta*eye(1);
%             
%             %         Q_p = Q_p_i + j*delta*eye(4);
%             %         R_p = R_p_i + i*delta*eye(1);
%             
%             Q_a = Q_a_i + j*delta*eye(6);
%             R_a = R_a_i + i*delta*eye(1);
%             
%             load_system('drone_controller');
%             
%             modelWorkspace = get_param('drone_controller','modelworkspace');
%             modelWorkspace.assignin('Q_z', Q_z);
%             modelWorkspace.assignin('R_z', R_z);
%             modelWorkspace.assignin('Q_p', Q_p);
%             modelWorkspace.assignin('R_p', R_p);
%             modelWorkspace.assignin('Q_a', Q_a);
%             modelWorkspace.assignin('R_a', R_a);
%             sim('drone_controller');
%         end
%     end
% end














