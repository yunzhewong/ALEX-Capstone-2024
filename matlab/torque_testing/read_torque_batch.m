function [average_currents, final_positions] = read_torque_batch(directory)
    [names, count] = get_files(directory);
    
    average_currents = zeros(1, count);
    final_positions = zeros(1, count);
    
    figure
    hold on
    title("Position vs Time for Incrementing Currents")
    xlabel("Test Time (s)")
    ylabel("Position (degrees)")
    
    for i = 1:count
        [times, currents, ~, positions] = read_data(string(names(i)));
    
        position_degrees = positions * 180 / pi;
    
        average_currents(i) = mean(currents);
        final_positions(i) = position_degrees(end);
        
        plot(times, position_degrees, "DisplayName", string(average_currents(i)))
    end
    legend
end
