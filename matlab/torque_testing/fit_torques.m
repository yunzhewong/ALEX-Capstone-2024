function fit_torques(average_currents, final_positions, cutoff_index)
    cutoff_current = average_currents(cutoff_index);
   
    y = sind(final_positions(cutoff_index:end));
    x = average_currents(cutoff_index:end);
    p = polyfit(x, y, 1);
    
    Kt = 0.124 * 120;
    g = 9.81;

    
    % m = 5.77;
    m_times_l = Kt / (g*p(1));
    F_kinetic = -p(2) * m_times_l * g;
    F_static = cutoff_current * Kt;
    
    fprintf("Effective M times Length: %.4f\n", m_times_l)
    fprintf("Effective Kinetic Friction: %.4f\n", F_kinetic)
    fprintf("Effective Static Friction: %.4f\n", F_static)
    
    input_torques = Kt * average_currents;
    
    expected_sin_positions = zeros(1, numel(average_currents));
    for cutoff_index = 1:numel(average_currents)
        if input_torques(cutoff_index) >= F_static
            expected_sin_positions(cutoff_index) = (input_torques(cutoff_index) - F_kinetic) / (m_times_l * g);
        end
    end
    
    
    
    figure
    hold on
    plot(average_currents, sind(final_positions))
    plot(average_currents, expected_sin_positions);
    title("Current vs Sine of Final Calf Position")
    legend("Measured Results", "Modelled Results", "Location", "southeast")
    xlabel("Mean Current")
    ylabel("sin(\theta)")
    
    figure
    hold on
    plot(average_currents, final_positions);
    plot(average_currents, asind(expected_sin_positions))
    title("Current vs Angle Final Calf Position")
    legend("Measured Results", "Modelled Results", "Location", "southeast")
    xlabel("Mean Current")
    ylabel("Angle (degrees)")
end