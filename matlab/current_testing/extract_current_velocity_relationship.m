function [J_t, b_2, b_1] = extract_current_velocity_relationship(directory, start_index, end_index)
    [names, count] = get_files(directory);
    
    steady_state_currents = zeros(1, count);
    steady_state_velocities = zeros(1, count);
    first_accels = zeros(1, count);

    for i=1:count
        [times, currents, velocities, ~] = read_data(names(i));
    
        steady_state_index = floor(numel(times) / 2);
    
        steady_state_currents(i) = mean(currents(steady_state_index:end));
        steady_state_velocities(i) = mean(velocities(steady_state_index:end));
    
        FIRST_COUNT = 20;
        first_velocities = velocities(1:FIRST_COUNT, 1);
        first_times = times(1:FIRST_COUNT, 1);
        accels = zeros(1, FIRST_COUNT);
        for j = 1:(FIRST_COUNT-1)
            v_diff = first_velocities(j + 1) - first_velocities(j);
            t_diff = first_times(j + 1) - first_times(j);
            accels(j) = v_diff/t_diff;
        end
        first_accels(i) = mean(accels);
    end
    
    % figure
    % plot(steady_state_velocities)
    % 
    %assumption
    Kt = 0.124 * 120;
    F_kinetic = 8.1003;
    F_static = 10.4160;
    
    moving_currents = steady_state_currents(start_index:end_index);
    moving_velocities = steady_state_velocities(start_index:end_index);
    moving_accelerations = first_accels(start_index:end_index);
    
    expected_torques = Kt * moving_currents - F_kinetic;
    
    % J_t*a_i = tau_i
    J_t_measurements = expected_torques ./ moving_accelerations;
    
    % figure
    % plot(J_t_measurements)
    
    J_t = mean(J_t_measurements);
    fprintf("J_t = %.4f\n", J_t);
    
    b_predictions = expected_torques ./ moving_velocities;
    
    % figure
    % plot(moving_currents, b_predictions)
    % title("Observed Current vs Predicted Damping Coefficient")
    % ylabel("Damping Coefficient")
    % xlabel("Current (A)")
    
    %looks like b is a function of i, such that b(i) = b_2*i + b_1
    pb = polyfit(moving_currents, b_predictions, 1);
    b_2 = pb(1);
    b_1 = pb(2);
    fprintf("b(i) = %.4f*i + %.4f\n", b_2, b_1);
    
    sim_currents = moving_currents(1):0.01:moving_currents(end);
    sim_b = b_2 * sim_currents + b_1;
    
    % figure
    % plot(moving_currents, b_predictions)
    % hold on
    % plot(sim_currents, sim_b)   
    % title("Current vs Predicted Damping Coefficient")
    % ylabel("Damping Coefficient")
    % xlabel("Current (A)")
    % legend("Observed Results", "Line of Best Fit")
    
    expected_velocities = zeros(1, numel(sim_currents));
    for i = 1:numel(sim_currents)
        motor_torque = Kt*sim_currents(i);
        if motor_torque > F_static
            expected_velocities(i) = (motor_torque - F_kinetic)/sim_b(i);
        end
    end
    
    % figure
    % plot(sim_currents, expected_velocities)
    % hold on
    % plot(steady_state_currents, steady_state_velocities)
    % title("Current vs Velocity")
    % ylabel("Velocity (rad/s)")
    % xlabel("Current (A)")
    % legend("Observed Results", "Modelled Results")
end
