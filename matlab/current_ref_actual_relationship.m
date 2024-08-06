close all
current_commands = 0.6:0.02:2.2;
resulting_currents = zeros(1, numel(current_commands));

figure
hold on
for i = 1:numel(current_commands)
    data = readmatrix("./data/exo batch 2/step" + sprintf('%0.2f',current_commands(i)) + "A.csv");
    
    times = data(:, 1);
    corrected_times = times - times(1);
    currents = data(:, 2);
    resulting_currents(i) = mean(currents);

    plot(corrected_times, currents);
end

figure
plot(current_commands, resulting_currents)

bestfit = polyfit(current_commands, resulting_currents, 1);
disp(bestfit)

% there is pretty much a 1 to 1 ratio
% y = 1.0082x + -0.0103
