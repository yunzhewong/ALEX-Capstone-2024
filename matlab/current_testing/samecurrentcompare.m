current = 2;


figure
hold on
for i = 1:9
    filename = './samecurrent/' + string(current) + string(i) + ".csv";

    [times, currents, velocities, ~] = read_data(filename);

    rolling = movmean(currents, 100);

    plot(times, rolling, "DisplayName", string(i));
end
legend("Location", "southeast")
title("Current vs Time for Repeated Current = 2A")
xlabel("Time (s)")
ylabel("Current (rads^{-1})")

dcm = datacursormode(gcf);
set(dcm, 'UpdateFcn', @myUpdateFcn);

% Custom update function to display plot name
function output_txt = myUpdateFcn(~, event_obj)
    % Get the target (the plot line)
    target = get(event_obj, 'Target');
    
    % Get the display name
    displayName = get(target, 'DisplayName');
    
    % Get the position of the data cursor
    pos = get(event_obj, 'Position');
    
    % Construct the output text
    output_txt = {['X: ', num2str(pos(1))], ...
                  ['Y: ', num2str(pos(2))], ...
                  ['Name: ', displayName]};
end
