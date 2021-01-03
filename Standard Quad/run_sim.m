clear;
clc;
close all;

open_system('drone_controller');
mdlWks = get_param('drone_controller','ModelWorkspace');
clear(mdlWks);

%% System Parameters

TUNE = false;

Ix = 0.1;
Iy = 0.1;
Iz = 0.1;

I = [Ix, 0, 0;
    0, Iy, 0;
    0, 0, Iz];

m = 5;

J_TP = 0.5;

l = 0.2;
b = 0.5;
d = 0.5;

%% LQR Gains
% s = [x, dx, y, dy, z, dz, phi, p, theta, q, psi, r]

Q_z = [1000,0;
       0,1000];

R_z = 1;

Q_p = [1,0,0,0;
    0,.5,0,0;
    0,0,1,0;
    0,0,0,.5];

R_p = 20;

Q_a = [100000,0,0,0,0,0;
    0,10,0,0,0,0;
    0,0,100000,0,0,0;
    0,0,0,10,0,0;
    0,0,0,0,10,0;
    0,0,0,0,0,10];

R_a = 10;


%% Setpoints

x_t = 1;
y_t = 2;
z_t = 5; 

phi_t = 0.0;
theta_t = 0.0;
psi_t = 0.7;

if ~TUNE
    sim('drone_controller');
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














