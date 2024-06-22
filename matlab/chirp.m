close all

filename = 'chirp025.csv';
data = readmatrix(filename);

times = data(:, 1);
currents = data(:, 2);
velocities = data(:, 3);

frequencies = zeros(numel(times), 1);
initial = 0;
final = 25;
t = 5;
slope = (final - initial) / t;
for i = 1:numel(times)
    time = times(i);
    frequency = slope * time + initial;
    frequencies(i) = frequency;
end

simulated_velocity = sin(2*pi*frequencies.*times);

% velocities = sin(10 * 2 * pi * times);
% positions = data(:, 4);
% 
% figure
% plot(times, currents, "r-")
% 
% figure
% plot(times, velocities, "b-")
% plot(times, simulated_velocity, "g-")
% 
% figure
% plot(times, positions, "r-")

T = 0.003;
Fs = 1 / T;
[pks, locs] = findpeaks(velocities);

magnitudes = zeros(numel(locs) - 1, 1);
frequencies = zeros(numel(locs) - 1, 1);

for i = 1:numel(locs) - 1
    start = locs(i);
    finish = locs(i + 1);
    L = finish - start;
    section = velocities(start:finish);
    ffted  = fft(section);
    
    P2 = abs(ffted/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    
    f = Fs/L*(0:(L/2));
    [M, I] = max(P1);
    frequencies(i) = f(I);
    magnitudes(i) = M;
end

figure
scatter(frequencies, magnitudes);

db = -20* log10(magnitudes);
figure
scatter(frequencies, db)
