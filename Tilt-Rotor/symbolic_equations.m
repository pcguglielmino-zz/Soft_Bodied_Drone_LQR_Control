clear all
syms x y z dx dy dz phi theta psi p q r alpha alpha_dot beta beta_dot m Ixx Iyy Izz g J_TP b d l_l l_s O1 O2 O3 O4 c_d c_l rho A wind_vel real

s = [x y z phi theta psi]';
s_dot = [dx dy dz p q r]';
Omega = [O1 O2 O3 O4]';
w_b = s_dot(4:6);

% Inertia and coriolis matricies

mass = m*eye(3);
inertia = [Ixx, 0, 0;
            0, Iyy, 0;
            0, 0, Izz];

M_H = [mass, zeros(3,3);
       zeros(3,3), inertia];

skew = inertia*s_dot(4:6);

C_H = [zeros(3,3), zeros(3,3);
       zeros(1,3), 0, -skew(3), skew(2);
       zeros(1,3), skew(3), 0, -skew(1);
       zeros(1,3), -skew(2), skew(1), 0];

% Gravity   

G = m*[0;0;g];

G_H = [G;0;0;0];

% Torques from angular momentum

alpha_matrix = [cos(alpha),  0, sin(alpha);
                    0,       1,      0;
                -sin(alpha), 0, cos(alpha)];
    
beta_matrix = [cos(beta),  0, sin(beta);
                    0,     1,     0;
               -sin(beta), 0, cos(beta)];

alpha_vector = [-sin(alpha); 0; -cos(alpha)];
beta_vector = [-sin(beta); 0; -cos(beta)];

a_vector = cross(w_b, alpha_vector) + cross([0; alpha_dot; 0], alpha_vector);
b_vector = cross(w_b, beta_vector) + cross([0; beta_dot; 0], beta_vector);
four_vector = [a_vector, -a_vector, -b_vector, b_vector];
O = [zeros(3,4);  four_vector];

O_H = J_TP * O;

% Forces and torques from motor thrust

R = [cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
     sin(psi)*cos(theta), cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
     -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];

E_B = [-b*sin(alpha), -b*sin(alpha), -b*sin(beta), -b*sin(beta);
        0,0,0,0;
        -b*cos(alpha), -b*cos(alpha), -b*cos(beta), -b*cos(beta);
        b*l_l*cos(alpha) + d*sin(alpha), -1*(b*l_l*cos(alpha) + d*sin(alpha)), b*l_l*cos(beta) - d*sin(beta), -1*(b*l_l*cos(beta) - d*sin(beta));
        b*l_s*cos(alpha), b*l_s*cos(alpha), -1*(b*l_s)*cos(beta), -1*(b*l_s)*cos(beta);
        -b*l_l*sin(alpha) + d*cos(alpha), b*l_l*sin(alpha) - d*cos(alpha), -b*l_l*sin(beta) - d*cos(beta), b*l_l*sin(beta) + d*cos(beta)];
       
T = [R, zeros(3,3);
    zeros(3,3), eye(3)];

E_H = T*E_B;

% Lift and drag forces

% v = [dx, dy, dz]';
% vel = dot(v,alpha_vector);


L_pba = alpha_matrix * [1;0;0] * c_l*rho*A * wind_vel;
L_pbb = beta_matrix * [1;0;0] * c_l*rho*A * wind_vel;
L_pb = L_pba + L_pbb;
L_tb = cross([l_s;0;0], L_pba) + cross([-l_s;0;0], L_pbb);

L_H = T*[L_pb;
         L_tb];
L_H = simplify(L_H);

D_pba = alpha_matrix * [0;0;1] * c_d*rho*A * wind_vel;
D_pbb = beta_matrix * [0;0;1] * c_d*rho*A * wind_vel;
D_pb = D_pba + D_pbb;
D_tb = cross([l_s;0;0], D_pba) + cross([-l_s;0;0], D_pbb);

D_H = T*[D_pb;
         D_tb];
D_H = simplify(D_H);


% Final equations
s_ddot = simplify(M_H\(-C_H*s_dot + G_H + O_H*Omega + E_H*Omega.^2 + L_H + D_H));
s_ddot_equal = simplify(subs(s_ddot, [beta, beta_dot], [alpha, alpha_dot]));
s_ddot_sub = simplify(subs(s_ddot, [alpha, beta, alpha_dot, beta_dot], [0,0,0,0]));


