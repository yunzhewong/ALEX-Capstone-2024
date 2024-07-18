close all

data = readmatrix("data/batch 1/step0.70A.csv");

times = data(:, 1);
currents = data(:, 2);
velocities = data(:, 3);

start_index = 1;
end_index = 2684;

figure
plot(1:numel(velocities), velocities)

sliced_times = times(start_index:end_index) - times(start_index);
sliced_velocities = velocities(start_index:end_index) - velocities(start_index);

figure
plot(sliced_times, sliced_velocities)

p = polyfit(sliced_times, sliced_velocities, 1);

slope = p(1)


A = 1;
K_t = 0.1;

F = K_t * A - slope * J

