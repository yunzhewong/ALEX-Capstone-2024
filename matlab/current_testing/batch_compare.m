names = ["batch1", "batch2", "batch3", "batch4", "batch5", "batch6"];

current = 1;
name = "current" + compose("%1.2f", current) + "A.csv";

figure
hold on
for i = 1:numel(names)
    batchName = names(i);
    filename = "./" + batchName + "/" + name;
    [times, currents, velocities, positions] = read_data(filename);

    rolling = movmean(currents, 100);
    plot(times, rolling)
    % % plot(times, currents)
    % plot(times, positions)
    % [c, v] = steady_state_current_velocity(batchName);
    % plot(c, v, "DisplayName", batchName)
end
legend("Location", "southeast")

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
