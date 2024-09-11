current = 2;


figure
hold on
for i = 1:9
    filename = './samecurrent/' + string(current) + string(i) + ".csv";

    [times, currents, velocities, ~] = read_data(filename);

    rolling = movmean(velocities, 100);

    plot(times, rolling);
end
