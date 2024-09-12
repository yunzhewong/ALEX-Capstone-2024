close all
clear

names = ["batch1", "batch2", "batch3", "batch4", "batch5", "batch6", "batch0309", "batch2808", "exo1", "exo2"];

rangestarts = [8, 5, 5, 3, 5, 3, 20, 20, 2, 16];
rangeends = [40, 30, 28, 34, 30, 33, 54, 50, 9, 78];

x = 0.8:0.01:2.5;
figure
hold on
for i = 1:numel(names)
    batchName = names(i);
    [~, b_2, b_1] = extract_current_velocity_relationship(batchName, rangestarts(i), rangeends(i));

    plot(x, b_2 * x + b_1, "DisplayName", batchName)
end
xlabel("Operating Currents (A)")
ylabel("Damping Coefficient")
title("Damping Coefficient vs Current for Various Tests")
ylim([0 70])
xlim([0 3])
legend