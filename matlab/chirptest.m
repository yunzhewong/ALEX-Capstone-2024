close all
clear

% sample transer function
sys = tf(3, [1 0.5 30]);
% sys = tf(1, [1 2]);

% the true bode magnitude distribution
figure
bode(sys)

%randomly generated points
POINTS = 10000;
t = linspace(0, 20, POINTS);
chirp_f = linspace(0, 50, POINTS);

u = sin(2 * pi * chirp_f .* t);
y = lsim(sys, u, t)';

chirp_identification(t, u, y)

