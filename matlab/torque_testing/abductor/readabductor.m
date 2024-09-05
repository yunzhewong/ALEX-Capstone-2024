close all
addpath("..")

[average_currents, final_positions] = read_torque_batch("straightleg");
fit_torques(average_currents, final_positions, 58, 58, 61);

%% 
[average_currents, final_positions] = read_torque_batch("withknee");
fit_torques(average_currents, final_positions, 17, 17, 40);


%% 
[average_currents, final_positions] = read_torque_batch("withoutkneeleg");
fit_torques(average_currents, final_positions, 36, 36, 55);