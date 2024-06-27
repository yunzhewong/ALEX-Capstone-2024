Kt_b = 1.303562;
J_b = 0.097466;

sys = tf(Kt_b, [J_b 1]);

[t, u, expected_y] = load_data('data/constantCurrent.csv');

% y = lsim(sys, u, t)';


figure
plot(t, u)

figure
% plot(t, y)
% hold on
plot(t, expected_y)