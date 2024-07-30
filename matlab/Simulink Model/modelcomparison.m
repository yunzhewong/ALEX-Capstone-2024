clear all

FILENAME = '../data/batch 6/step0.76A.csv';
data = readmatrix(FILENAME);    
times = data(:, 1);

corrected_times = times - times(1);
velocities = data(:, 3);

figure
plot(corrected_times, velocities)
hold on
plot(out.velocity.Time, out.velocity.Data)