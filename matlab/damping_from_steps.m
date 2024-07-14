close all

data = readmatrix("data/0.1.csv");

times = data(:, 1);
currents = data(:, 5);
velocities = data(:, 6);

start_index = 4700;
steady_state_index = 5115;
end_index = 14240;

figure
plot(1:numel(velocities), velocities)
hold on
xline(start_index)
xline(steady_state_index)
xline(end_index)

steady_state_value = mean(velocities(steady_state_index:end_index))
one_time_constant_value = steady_state_value * (1 - exp(-1));

[~, match_index] = closest_match(velocities, one_time_constant_value);

adjusted_times = times(start_index: end_index) - times(start_index);
time_constant = adjusted_times(match_index - start_index)

K_t = 0.1;

radius = 0.06 / 2;
length = 0.055;
density = 7850;

volume = pi * radius * radius * length;
mass = volume * density;

J = 0.5 * mass * radius * radius;
A = 1;

b_steady_state = K_t * A / steady_state_value
b_time_constant = J / time_constant