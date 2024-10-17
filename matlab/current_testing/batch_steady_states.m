names = ["exo2"];

fig = figure;
hold on
for i = 1:numel(names)
    batchName = names(i);
    [c, v] = steady_state_current_velocity(batchName);
    plot(c, v)
end
% title("Steady State Velocity vs Steady State Current for Various Batches")
ylabel("Steady State Velocity (rads^{-1})")
xlabel("Steady State Current (A)")


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

function [currents, velocities] = steady_state_current_velocity(directory)
    [names, count] = get_files(directory);
    
    steady_state_currents = zeros(1, count);
    steady_state_velocities = zeros(1, count);

    for i=1:count
        [times, currents, velocities, ~] = read_data(names(i));
    
        steady_state_index = floor(numel(times) / 2);
    
        steady_state_currents(i) = mean(currents(steady_state_index:end));
        steady_state_velocities(i) = mean(velocities(steady_state_index:end));
    end

    currents = steady_state_currents;
    velocities = steady_state_velocities;
end
