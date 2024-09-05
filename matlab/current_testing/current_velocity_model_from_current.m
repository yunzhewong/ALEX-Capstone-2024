close all
clear

[J_t, b_2a, b_1a] = extract_current_velocity_relationship('batch0309', 20, 54);
[J_t, b_2b, b_1b] = extract_current_velocity_relationship('batch2808', 20, 50);
[J_t, b_2c, b_1c] = extract_current_velocity_relationship('exo1', 2, 9);
[J_t, b_2d, b_1d] = extract_current_velocity_relationship('exo2', 16, 78);



x = 0.8:0.01:2.5;
figure
plot(x, b_2a * x + b_1a)
hold on
plot(x, b_2b * x + b_1b)
plot(x, b_2c * x + b_1c)
plot(x, b_2d * x + b_1d)
xlabel("Operating Currents (A)")
ylabel("Damping Coefficient")
title("Damping Coefficient vs Current for Various Tests")
ylim([0 70])
xlim([0 3])