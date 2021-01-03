function [ dx ] = lqr_ode(t, x, param)
% note x is in the form of phi, dphi, theta, dtheta, psi, dpsi, z, dz

[phi, dphi, theta, dtheta, psi, dpsi, z, dz] = deal(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8));
[I_x, I_y, I_z, J_r, l, m, d, b] = deal(param(1), param(2), param(3), param(4), param(5), param(6), param(7), param(8));
% I_n: Moments of inertia, J_r: propeller jyro effect, l: drone arm
% length, m: drone mass, d: drag coefficient, b: thrust coefficient

persistent omega_r
if isempty(omega_r)
    omega_r = 0;
end

a_1 = (I_y - I_z)/I_x;
a_2 = J_r/I_x;
a_3 = dpsi*(I_z-I_x)/I_y;
a_4 = J_r/I_x;
a_5 = (I_z - I_y)/I_z;
b_1 = l/I_x;
b_2 = l/I_y;
b_3 = l/I_z;

% compute the control matricies

A = [0, 1, 0, 0, 0, 0, 0, 0;
    0, 0, 0, dtheta*a_1 - dtheta*a_2*omega_r, 0, 0, 0, 0;
    0, 0, 0, 1, 0, 0, 0, 0;
    0, dpsi*a_3 - dtheta*a_4*omega_r, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, dphi*a_5, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 1;
    0, 0, 0, 0, 0, 0, 0, 0];

B = [0, 0, 0, 0;
    0, b_1, 0, 0;
    0, 0, 0, 0;
    0, 0, b_2, 0;
    0, 0, 0, 0;
    0, 0, 0, b_3;
    0, 0, 0, 0;
    (cos(phi)*cos(theta)*(1/m)), 0, 0, 0];

% LQR control matricies

Q = [0.0005, 0, 0, 0, 0, 0, 0, 0;
    0, 0.0005, 0, 0, 0, 0, 0, 0;
    0, 0, 0.0005, 0, 0, 0, 0, 0;
    0, 0, 0, 0.0005, 0, 0, 0, 0;
    0, 0, 0, 0, 0.0005, 0, 0, 0;
    0, 0, 0, 0, 0, 0.0005, 0, 0;
    0, 0, 0, 0, 0, 0, 0.0625, 0;
    0, 0, 0, 0, 0, 0, 0, 0.625];

R = [0.1, 0, 0, 0;
    0, 0.1, 0, 0;
    0, 0, 0.1, 0;
    0, 0, 0, 0.1];

% Solve Riccati equation

[P,K,L] = icare(A,B,Q,R,[],[],[]);
[K_1, S, e] = lqr(A,B,Q,R);


u = -inv(R)*B'*P*x;

% the control inputs (motor speeds) of the system:
A_u = [-b, -b, -b, -b;
    0, b, 0, -b;
    -b, 0, b, 0;
    d, -d, d, -d];

B_u = -u;

W = linsolve(A_u, B_u)

omega_r = sqrt(W(1)) + sqrt(W(2)) + sqrt(W(3)) + sqrt(W(4));

dx = A*x + B*u;

end

