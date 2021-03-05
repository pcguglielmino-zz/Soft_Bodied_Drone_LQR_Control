clc
clear
close all

%% Load data

% phi = simOut.logsout{1}.Values.Data;
% psi = simOut.logsout{2}.Values.Data;
% theta = simOut.logsout{3}.Values.Data;
% X = simOut.logsout{4}.Values.Data;
% Y = simOut.logsout{5}.Values.Data;
% Z = simOut.logsout{6}.Values.Data;

load('VTOL_sim.mat');
phi_VTOL_sim = simOut.logsout{1}.Values.Data;
psi_VTOL_sim = simOut.logsout{2}.Values.Data;
theta_VTOL_sim = simOut.logsout{3}.Values.Data;
X_VTOL_sim = simOut.logsout{4}.Values.Data;
Y_VTOL_sim = simOut.logsout{5}.Values.Data;
Z_VTOL_sim = simOut.logsout{6}.Values.Data;

load('tilt_rotor_sim.mat');
phi_tilt_rotor_sim = simOut.logsout{1}.Values.Data;
psi_tilt_rotor_sim = simOut.logsout{2}.Values.Data;
theta_tilt_rotor_sim = simOut.logsout{3}.Values.Data;
X_tilt_rotor_sim = simOut.logsout{4}.Values.Data;
Y_tilt_rotor_sim = simOut.logsout{5}.Values.Data;
Z_tilt_rotor_sim = simOut.logsout{6}.Values.Data;

load('mixed_sim.mat');
phi_mixed_sim = simOut.logsout{1}.Values.Data;
psi_mixed_sim = simOut.logsout{2}.Values.Data;
theta_mixed_sim = simOut.logsout{3}.Values.Data;
X_mixed_sim = simOut.logsout{4}.Values.Data;
Y_mixed_sim = simOut.logsout{5}.Values.Data;
Z_mixed_sim = simOut.logsout{6}.Values.Data;

load('VTOL_big_step.mat');
phi_VTOL_big_step = simOut.logsout{1}.Values.Data;
psi_VTOL_big_step = simOut.logsout{2}.Values.Data;
theta_VTOL_big_step = simOut.logsout{3}.Values.Data;
X_VTOL_big_step = simOut.logsout{4}.Values.Data;
Y_VTOL_big_step = simOut.logsout{5}.Values.Data;
Z_VTOL_big_step = simOut.logsout{6}.Values.Data;
time_VTOL_big_step = simOut.tout;

load('tilt_rotor_big_step.mat');
phi_tilt_rotor_big_step = simOut.logsout{1}.Values.Data;
psi_tilt_rotor_big_step = simOut.logsout{2}.Values.Data;
theta_tilt_rotor_big_step = simOut.logsout{3}.Values.Data;
X_tilt_rotor_big_step = simOut.logsout{4}.Values.Data;
Y_tilt_rotor_big_step = simOut.logsout{5}.Values.Data;
Z_tilt_rotor_big_step = simOut.logsout{6}.Values.Data;
time_tilt_rotor_big_step = simOut.tout;

load('mixed_big_step.mat');
phi_mixed_big_step = simOut.logsout{1}.Values.Data;
psi_mixed_big_step = simOut.logsout{2}.Values.Data;
theta_mixed_big_step = simOut.logsout{3}.Values.Data;
X_mixed_big_step = simOut.logsout{4}.Values.Data;
Y_mixed_big_step = simOut.logsout{5}.Values.Data;
Z_mixed_big_step = simOut.logsout{6}.Values.Data;
time_mixed_big_step = simOut.tout;

load('VTOL_small_step.mat');
phi_VTOL_small_step = simOut.logsout{1}.Values.Data;
psi_VTOL_small_step = simOut.logsout{2}.Values.Data;
theta_VTOL_small_step = simOut.logsout{3}.Values.Data;
X_VTOL_small_step = simOut.logsout{4}.Values.Data;
Y_VTOL_small_step = simOut.logsout{5}.Values.Data;
Z_VTOL_small_step = simOut.logsout{6}.Values.Data;
time_VTOL_small_step = simOut.tout;

load('tilt_rotor_small_step.mat');
phi_tilt_rotor_small_step = simOut.logsout{1}.Values.Data;
psi_tilt_rotor_small_step = simOut.logsout{2}.Values.Data;
theta_tilt_rotor_small_step = simOut.logsout{3}.Values.Data;
X_tilt_rotor_small_step = simOut.logsout{4}.Values.Data;
Y_tilt_rotor_small_step = simOut.logsout{5}.Values.Data;
Z_tilt_rotor_small_step = simOut.logsout{6}.Values.Data;
time_tilt_rotor_small_step = simOut.tout;

load('mixed_small_step.mat');
phi_mixed_small_step = simOut.logsout{1}.Values.Data;
psi_mixed_small_step = simOut.logsout{2}.Values.Data;
theta_mixed_small_step = simOut.logsout{3}.Values.Data;
X_mixed_small_step = simOut.logsout{4}.Values.Data;
Y_mixed_small_step = simOut.logsout{5}.Values.Data;
Z_mixed_small_step = simOut.logsout{6}.Values.Data;
time_mixed_small_step = simOut.tout;

load('VTOL_vel.mat');
phi_VTOL_vel = simOut.logsout{1}.Values.Data;
psi_VTOL_vel = simOut.logsout{2}.Values.Data;
theta_VTOL_vel = simOut.logsout{3}.Values.Data;
X_VTOL_vel = simOut.logsout{4}.Values.Data;
Y_VTOL_vel = simOut.logsout{5}.Values.Data;
Z_VTOL_vel = simOut.logsout{6}.Values.Data;
time_VTOL_vel = simOut.tout;

load('tilt_rotor_vel.mat');
phi_tilt_rotor_vel = simOut.logsout{1}.Values.Data;
psi_tilt_rotor_vel = simOut.logsout{2}.Values.Data;
theta_tilt_rotor_vel = simOut.logsout{3}.Values.Data;
X_tilt_rotor_vel = simOut.logsout{4}.Values.Data;
Y_tilt_rotor_vel = simOut.logsout{5}.Values.Data;
Z_tilt_rotor_vel = simOut.logsout{6}.Values.Data;
time_tilt_rotor_vel = simOut.tout;

load('mixed_vel.mat');
phi_mixed_vel = simOut.logsout{1}.Values.Data;
psi_mixed_vel = simOut.logsout{2}.Values.Data;
theta_mixed_vel = simOut.logsout{3}.Values.Data;
X_mixed_vel = simOut.logsout{4}.Values.Data;
Y_mixed_vel = simOut.logsout{5}.Values.Data;
Z_mixed_vel = simOut.logsout{6}.Values.Data;
time_mixed_vel = simOut.tout;

%% Step Response Big
figure();
hold on
step_p_vtol = plot(time_VTOL_big_step, X_VTOL_big_step(:), 'Color', 'r');
step_p_vtol.LineWidth = 2;
step_p_tilt = plot(time_tilt_rotor_big_step, X_tilt_rotor_big_step(:), 'Color', 'g');
step_p_tilt.LineWidth = 2;
step_p_mixed = plot(time_mixed_big_step, X_mixed_big_step(:), 'Color', 'b');
step_p_mixed.LineWidth = 2;
step_p_target = plot(time_mixed_big_step,25*ones(1,length(time_mixed_big_step)), '--', 'Color', 'k');
xlabel('time (s)', 'FontSize',20)
ylabel('x position (meters)', 'FontSize', 20)
ax = gca;
ax.FontSize = 16; 
legend('VTOL Controller','Tilt Rotor Controller','Mixed Controller');
grid on
hold off

%% Step Response Small
figure();
hold on
step_p_vtol = plot(time_VTOL_small_step, X_VTOL_small_step(:), 'Color', 'r');
step_p_vtol.LineWidth = 2;
step_p_tilt = plot(time_tilt_rotor_small_step, X_tilt_rotor_small_step(:), 'Color', 'g');
step_p_tilt.LineWidth = 2;
step_p_mixed = plot(time_mixed_small_step, X_mixed_small_step(:), 'Color', 'b');
step_p_mixed.LineWidth = 2;
step_p_target = plot(time_mixed_small_step,1*ones(1,length(time_mixed_small_step)), '--', 'Color', 'k');
xlabel('time (s)', 'FontSize', 20)
ylabel('x position (meters)', 'FontSize', 20)
ax = gca;
ax.FontSize = 16; 
legend('VTOL Controller','Tilt Rotor Controller','Mixed Controller');
grid on
hold off

%% Velocity Response
figure();
hold on
step_p_vtol = plot(time_VTOL_vel, my_diff(time_VTOL_vel, X_VTOL_vel(:)), 'Color', 'r');
step_p_vtol.LineWidth = 2;
step_p_tilt = plot(time_tilt_rotor_vel, my_diff(time_tilt_rotor_vel, X_tilt_rotor_vel(:)), 'Color', 'g');
step_p_tilt.LineWidth = 2;
step_p_mixed = plot(time_mixed_vel, my_diff(time_mixed_vel, X_mixed_vel(:)), 'Color', 'b');
step_p_mixed.LineWidth = 2;
step_p_target = plot(time_mixed_vel,10*ones(1,length(time_mixed_vel)), '--', 'Color', 'k');
xlabel('time (s)', 'FontSize', 20)
ylabel('velocity (meters/second)', 'FontSize', 20)
axis([0,9.5,0,11]);
ax = gca;
ax.FontSize = 16; 
legend('VTOL Controller','Tilt Rotor Controller','Mixed Controller', 'FontSize', 20);
grid on
hold off

%% Sim Plot
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

figure();
hold on
sim_p_vtol = plot3(X_VTOL_sim(:), Y_VTOL_sim(:), Z_VTOL_sim(:), 'Color', 'r');
sim_p_vtol.LineWidth = 2;
sim_p_tilt = plot3(X_tilt_rotor_sim(:), Y_tilt_rotor_sim(:), Z_tilt_rotor_sim(:), 'Color', 'g');
sim_p_tilt.LineWidth = 2;
sim_p_mixed = plot3(X_mixed_sim(:), Y_mixed_sim(:), Z_mixed_sim(:), 'Color', 'b');
sim_p_mixed.LineWidth = 2;
sim_p_target = plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:), '--', 'Color', 'k');
sim_p_target.LineWidth = 3;
axis equal
grid on
ax = gca;
ax.FontSize = 16; 
xlabel('x(t) (meters)', 'FontSize', 20)
ylabel('y(t) (meters)', 'FontSize', 20)
zlabel('z(t) (meters)', 'FontSize', 20)
legend('VTOL Controller','Tilt Rotor Controller','Mixed Controller');

%% Circle Function
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

%% Diff Function
function D = my_diff(time, vector)
D = zeros(1,length(time));

for i = 2:length(time)
    D(i) = (vector(i) - vector(i-1))/(time(i) - time(i-1));
end

end