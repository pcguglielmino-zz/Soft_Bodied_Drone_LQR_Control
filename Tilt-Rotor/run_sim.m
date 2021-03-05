clear;
clc;
close all;

open_system('tilt_rotor_drone_controller');
mdlWks = get_param('tilt_rotor_drone_controller','ModelWorkspace');
clear(mdlWks);

%% System Parameters

TUNE = false;

mass = [0.736, 0, 0;
        0, 0.736, 0;
        0, 0, 0.736];
    
inertia = [0.0092, 0, 0;
            0, 0.01134, 0;
            0, 0, 0.0025];

Ix = inertia(1);
Iy = inertia(5);
Iz = inertia(9);
        
wing_area = 0.000124; %m^2
rho = 1.1455; %kg/m^3

l_l = 0.154;
l_s = 0.19;
l_a = 0;
lambda = 0.01;

J_TP = 0.5;

b = 20;
d = 0.5;

alpha = 0;
beta = 0;
% alpha = -pi/2;
% beta = -pi/2;
gamma = [alpha; beta];

m = 0.736;

%% LQR Gains
% s = [x, dx, y, dy, z, dz, phi, p, theta, q, psi, r]

Q_t = [10,0;
       0,10];

R_t = 0.0001;

Q_p = [3,0,0,0;
    0,.5,0,0;
    0,0,3,0;
    0,0,0,.5];

R_p = 3;

Q_a = [100000,0,0,0,0,0;
    0,10,0,0,0,0;
    0,0,100000,0,0,0;
    0,0,0,10,0,0;
    0,0,0,0,10,0;
    0,0,0,0,0,10];

R_a = 10;

Q_g = [1,0,0,0;
       0,0.5,0,0;
       0,0,1,0;
       0,0,0,0.5];

R_g = 0.5;


Q_t_p = [10,0;
         0,10];

R_t_p = 0.0001;


Q_g_p = [3,0;
         0,.5];

R_g_p = 3;

%% LQR K Matricies

% Thrust
A_thrust = [0,1;
            0,0];
B_thrust = [0;1];
C_thrust = [1,0];

K_thrust = lqr(A_thrust, B_thrust, Q_t, R_t);
k_r_thrust = -inv(C_thrust/(A_thrust-B_thrust*K_thrust)*B_thrust);

% Position
a = -b/m;
A_position = [0,1,0,0;
     0,0,0,0;
     0,0,0,1;
     0,0,0,0];
B_position = [0,0;
     a,0;
     0,0;
     0,a];
C_position = [1,0,0,0;
     0,0,1,0];

K_position = lqr(A_position, B_position, Q_p, R_p);
k_r_position = -inv(C_position/(A_position-B_position*K_position)*B_position);

% Attitude
A_attitude = [0,1,0,0,0,0;
              0,0,0,0,0,0;
              0,0,0,1,0,0;
              0,0,0,0,0,0;
              0,0,0,0,0,1;
              0,0,0,0,0,0];
B_attitude = [0,0,0;
             (1/Ix),0,0;
              0,0,0;
              0,(1/Iy),0;
              0,0,0;
              0,0,(1/Iz)];
C_attitude = [1,0,0,0,0,0;
              0,0,1,0,0,0;
              0,0,0,0,1,0];
          
K_attitude = lqr(A_attitude, B_attitude, Q_a, R_a);
k_r_attitude = -inv(C_attitude/(A_attitude-B_attitude*K_attitude)*B_attitude);

% Gamma
A_gamma = [0,1,0,0;
     0,0,0,0;
     0,0,0,1;
     0,0,0,0];
B_gamma = [0,0;
     a,0;
     0,0;
     0,a];
C_gamma = [1,0,0,0;
           0,0,1,0];

K_gamma = lqr(A_gamma, B_gamma, Q_g, R_g);
k_r_gamma = -inv(C_gamma/(A_gamma-B_gamma*K_gamma)*B_gamma);

% Plane
A_thrust_plane = [0,1;
                  0,0];
B_thrust_plane = [0;1];
C_thrust_plane = [1,0];

K_thrust_plane = lqr(A_thrust_plane, B_thrust_plane, Q_t_p, R_t_p);
k_r_thrust_plane = -inv(C_thrust_plane/(A_thrust_plane-B_thrust_plane*K_thrust_plane)*B_thrust_plane);

A_gamma_plane = [0,1;
                 0,0];
B_gamma_plane = [0;1];
C_gamma_plane = [1,0];

K_gamma_plane = lqr(A_gamma_plane, B_gamma_plane, Q_g_p, R_g_p);
k_r_gamma_plane = -inv(C_gamma_plane/(A_gamma_plane-B_gamma_plane*K_gamma_plane)*B_gamma_plane);


K_test = lqr([A_gamma_plane, zeros(2,2); zeros(2,2), A_gamma_plane],...
             [B_gamma_plane, zeros(2,1); zeros(2,1), B_gamma_plane],...
             [Q_g_p, zeros(2,2); zeros(2,2), Q_g_p],...
             [R_g_p]);
%% Trajectory

% [x,...;
%  y,...;
%  z,...;
%  phi,...;
%  theta,...;
%  psi,...];
trajectory = [0,10;
              0,0;
              0,-3;
              0,0;
              0,0;
              0,0];

% trajectory = [0,2,1;
%   0,1,2;
%   0,-3,-5;
%   0,0,0;
%   0,0,0;
%   0,0.7,0.5];
          
trajectory_time = [1;1.1];

velocity_bc = zeros(size(trajectory));
acceleration_bc = zeros(size(trajectory));

V_t = 0;

if ~TUNE
    sim('tilt_rotor_drone_controller', 10);
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














