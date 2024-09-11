function [m_times_l, F_kinetic, F_static] = fit_torques(average_currents, final_positions, static_index, cutoff_start, cutoff_end)
    figure
    hold on
    plot(final_positions)
    xline(cutoff_start)
    xline(cutoff_end)

    %K_t*i = mgl*sin(theta) + F_kinetic
    % sin(theta) = (K_t/mgl)*i - F_kinetic/(m*g*l)
   
    y = sind(final_positions(cutoff_start:cutoff_end));
    x = average_currents(cutoff_start:cutoff_end);
    p = polyfit(x, y, 1);
    
    Kt = 0.124 * 120;
    g = 9.81;

    figure
    plot(x, y)
    hold on
    plot(x, p(1)*x + p(2))

    
    % m = 5.77;
    m_times_l = Kt / (g*p(1));
    F_kinetic = -p(2) * m_times_l * g;
    F_static = average_currents(static_index) * Kt;
    
    fprintf("Effective M times Length: %.4f\n", m_times_l)
    fprintf("Effective Kinetic Friction: %.4f\n", F_kinetic)
    fprintf("Effective Static Friction: %.4f\n", F_static)
    
    input_torques = Kt * average_currents;
    
    expected_sin_positions = zeros(1, numel(average_currents));
    for i = 1:numel(average_currents)
        if input_torques(i) >= F_static
            expected_sin_positions(i) = (input_torques(i) - F_kinetic) / (m_times_l * g);
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