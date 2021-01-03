clc
clear all;
close all;

%% Parameters and initial conditions
I_x = 1;
I_y = 1;
I_z = 1;
J_r = 0.05;
l = 0.5;
m = 5;
d = 0.5;
b = 1.5;

phi = 0;
dphi = 0;
theta = 0;
dtheta = 0;
psi = 0;
dpsi = 0;
z = 15;
dz = 3;

%% Simulation
tf = 30;
x0 = [phi, dphi, theta, dtheta, psi, dpsi, z, dz];
param = [I_x, I_y, I_z, J_r, l, m, d, b];

% options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) lqr_ode(t, x, param), [0:0.01:tf], x0);

%% Plotting

plot(T,X(:,7));
figure();
plot(T,X(:,8));
