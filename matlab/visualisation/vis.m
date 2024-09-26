close all
[times, currents, velocities, positions] = read_data("./abductor.csv");
[times_2, currents_2, velocities_2, positions_2] = read_data("./abduct.csv");

% extensor
t2 = 23.71;
t1 = 14.43;

% % abductor
% t2 = 23.942
% t1 = 14.60

% % knee
% t2 = 29.061;
% t1 = 19.63;

diff = t2 - t1;
adjust_times = times - diff;

fig = figure;
fig.Position = [50, 50, 1200, 700];
subplot(3, 1, 1)
hold on
plot(adjust_times, currents)
plot(times_2, currents_2)
xlim([0 86])

subplot(3, 1, 2)
hold on
plot(adjust_times, velocities)
plot(times_2, velocities_2)
xlim([0 86])

subplot(3, 1, 3)
hold on
plot(adjust_times, positions)
plot(times_2, positions_2)
xlim([0 86])