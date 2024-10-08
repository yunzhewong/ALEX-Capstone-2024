filename = "./batch6/current2.00A.csv";
[times, currents, ~, ~] = read_data(filename);

rolling = movmean(currents, 100);

figure
plot(times, currents)
hold on
plot(times, rolling);
yline(2, "LineWidth", 3, "LineStyle", ":")

title("Current Command Results")
ylabel("Current (A)")
xlabel("Time (s)")
legend("Measured Current", "Low-Passed Current", "Reference Current")