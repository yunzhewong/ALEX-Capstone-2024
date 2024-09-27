close all
[times, currents, velocities, positions] = read_data("./knee.csv");
[times_2, currents_2, velocities_2, positions_2] = read_data("./kneereal.csv");

reference = readmatrix('kneetraj.csv');
ref_times = reference(:, 1);
ref_velocities = reference(:, 2);
ref_pos = reference(:, 3);

% % extensor
% t2 = 23.71;
% t1 = 14.43;

% % abductor
% t2 = 23.942
% t1 = 14.60

% knee
t2 = 10.363;
t1 = 0.997;

diff = t2 - t1;
adjust_times = times - diff;

fig = figure;
fig.Position = [50, 50, 900, 750];

tiledlayout(3, 1, "TileSpacing","compact", "Padding","compact");

nexttile
hold on
plot(adjust_times, currents, "Color", "#4DBEEE")
plot(times_2, currents_2, "Color", "#7E2F8E")
xlim([0 40])
ylabel("Current (A)")

nexttile
hold on
plot(ref_times, ref_velocities, "LineWidth", 2, "LineStyle", '- -', "Color", "r")
plot(adjust_times, velocities, "Color", "#4DBEEE")
plot(times_2, velocities_2, "Color", "#7E2F8E")
xlim([0 35])
ylabel("Velocity (rads^{-1})")

nexttile
hold on
plot(ref_times, ref_pos, "LineWidth", 2, "LineStyle", '- -', "Color", "r")
plot(adjust_times, positions, "Color", "#4DBEEE")
plot(times_2, positions_2, "Color", "#7E2F8E")

xlim([0 40])
ylabel("Position (rad)")
xlabel("Time (s)")