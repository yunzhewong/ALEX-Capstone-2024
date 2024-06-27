close all
clear

Kt_b = 1.303562;
J_b = 0.097466;

sys = tf(Kt_b, [J_b 1]);

[times, currents, velocities, positions] = load_data("data/chirp0to25.csv");
[omega, m, theta] = chirp_identification(times, currents', velocities');

[sysM, sysTheta] = bode(sys, omega);
sysM = 20*log10(reshape(sysM(1,1, :), [], 1)');

figure
semilogx(omega, m, omega, sysM)