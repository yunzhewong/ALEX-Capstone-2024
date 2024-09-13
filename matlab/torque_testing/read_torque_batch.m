function [average_currents, final_positions] = read_torque_batch(directory)
    [names, count] = get_files(directory);
    
    average_currents = zeros(1, count);
    final_positions = zeros(1, count);
    
    figure
    hold on
    title("Position vs Time for Incrementing Currents")
    xlabel("Test Time (s)")
    ylabel("Position (degrees)")

    dcm = datacursormode(gcf);
    set(dcm, 'UpdateFcn', @myUpdateFcn);
    
    for i = 1:count
        [times, currents, ~, positions] = read_data(string(names(i)));
    
        position_degrees = positions * 180 / pi;
    
        average_currents(i) = mean(currents);
        final_positions(i) = position_degrees(end);
        
        plot(times, position_degrees, "DisplayName", string(average_currents(i)) + ":" + string(names(i)))
    end
    legend
end

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