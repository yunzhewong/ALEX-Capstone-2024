
iters = 0.02;
low = 0.6;
high = 2.4;

amperage = low:iters:high;

model = 'friction_model';
open_system(model)

for i = 1:numel(amperage)
    [times, currents, velocities] = loadFile(amperage(i));
    
    diffs = zeros(1, numel(times) - 1);
    for j = 1:numel(diffs)
        diffs(j) = times(j + 1) - times(j);
    end
    dt = mean(diffs);
    
    model = 'friction_model';
    load_system(model)
    simout = sim(model);

    sim_times = simout.velocity.Time;
    sim_velocities = simout.velocity.Data;

    corrected_times = times - times(1);
    synchronised_velocities = zeros(numel(sim_velocities), 1);
    synchronised_velocities(1) = sim_velocities(1);
    
    last_match = 2;
    for j = 1:numel(corrected_times)
        real_time = corrected_times(j);
        if real_time >= sim_times(last_match)
            synchronised_velocities(last_match) = velocities(j);
            last_match = last_match + 1;
        end
    end
    
    figure
    plot(sim_times, synchronised_velocities)
    hold on
    plot(sim_times, sim_velocities)
    
    mape_error = mape(sim_velocities, synchronised_velocities, "omitzero");
    fprintf("(%1.2fA)MAPE: %f%%\n", amperage(i), mape_error)

    steady_state_index = floor(numel(sim_times) / 2);
    steady_state_true = mean(synchronised_velocities(steady_state_index:end));
    steady_state_sim = mean(sim_velocities(steady_state_index:end));

    yline(steady_state_true)
    yline(steady_state_sim)
    steady_state_error = (steady_state_true / steady_state_sim) * 100 - 100;

    fprintf("(%1.2fA)Steady State: %f%%\n", amperage(i), steady_state_error)
end

function [times, currents, velocities] = loadFile(current)
    FILENAME = "../data/exo batch 2/step" + compose("%1.2f", current) + "A.csv";
    data = readmatrix(FILENAME);    
    times = data(:, 1);
    currents = data(:, 2);
    velocities = data(:, 3);
    
    
end