close all

fig = figure;
hold on
[names, count] = get_files('batch6');

for i = 1:numel(names)
    data = readmatrix(names(i));
    
    times = data(:, 1);
    startTime = times(1);
    currents = data(:, 2);
    velocities = data(:, 3);

    plot(times - startTime, velocities)
end

dcm = datacursormode(gcf);
set(dcm, 'UpdateFcn', @myUpdateFcn);

title("Batch Results")
ylabel("Measured Velocity (rads^{-1})")
xlabel("Time (s)")


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
