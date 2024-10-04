clear
close all

% on top-twinmotor, using step functions from 0.5 - 0.8 with 0.02 step size
% J = 0.2052;
% b = 1.3877;
% K_t = 3.563;
% F_c = 1.4368;
% F_stat = 1.7102;

% on exo-rightkneemotor
J = 9.1617;
b_2 = -13.8341;
b_1 = 51.0314;
K_t = 0.124 * 120;
F_kinetic = 8.1003;
F_static = 10.4160;
EPSILON = 0.005;

K_p = 2;
K_i = 0;

% first order model
estimated_b = b_1 + 2.5 * b_2;

curr_vel_const = 180 / (pi * 3);
K = K_t / estimated_b;
tau = J / estimated_b;

closed_loop_A = -35.56 / 0.5571;
closed_loop_B = 34.56 / 0.5571;