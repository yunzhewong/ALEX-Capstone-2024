clear

% on top-twinmotor, using step functions from 0.5 - 0.8 with 0.02 step size
J = 0.2052;
b = 1.3877;
K_t = 3.563;
F_c = 1.4368;
F_stat = 1.7102;

iters = 0.02;
low = 0.5;
high = 1.2;

amperage = low:iters:high;

for a = amperage
    FILENAME = "../data/batch 6/step" + compose("%1.2f", a) + "A.csv";;
    data = readmatrix(FILENAME);    
    times = data(:, 1);
    currents = data(:, 2);
    velocities = data(:, 3);
    
    diffs = zeros(1, numel(times) - 1);
    for i = 1:numel(diffs)
        diffs(i) = times(i + 1) - times(i);
    end
    dt = mean(diffs);
    
    EPSILON = 0.005;
    
    model = 'friction_model';
    load_system(model)
    simout = sim(model);
    
    sim_times = simout.velocity.Time;
    sim_velocities = simout.velocity.Data;

    corrected_times = times - times(1);
    
    synchronised_velocities = zeros(numel(sim_velocities), 1);
    synchronised_velocities(1) = sim_velocities(1);
    
    last_match = 2;
    for i = 1:numel(corrected_times)
        real_time = corrected_times(i);
        if real_time >= sim_times(last_match)
            synchronised_velocities(last_match) = velocities(i);
            last_match = last_match + 1;
        end
    end
    
    figure
    plot(sim_times, synchronised_velocities)
    hold on
    plot(sim_times, sim_velocities)
    
    error = mape(sim_velocities, synchronised_velocities, "omitzero");
    
    fprintf("(%1.2fA)MAPE: %f%%\n", a, error)
end
