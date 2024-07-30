close all


amperage = 0.20:0.02:1.00;


commands = zeros(1, numel(amperage));
resulting_currents = zeros(1, numel(amperage));

figure
hold on
for i = 1:numel(amperage)
    data = readmatrix("./data/batch 1/step" + sprintf('%0.2f',amperage(i)) + "A.csv");
    
    times = data(:, 1);
    corrected_times = times - times(1);
    currents = data(:, 2);
    avg_current = mean(currents);

    commands(i) = amperage(i);
    resulting_currents(i) = avg_current;

    plot(corrected_times, currents);
end

figure
plot(commands, resulting_currents)

bestfit = polyfit(commands, resulting_currents, 1)

% there is pretty much a 1 to 1 ratio
% y = 1.0082x + -0.0103
