close all
addpath("..")

[average_currents, final_positions] = read_torque_batch("withknee");
fit_torques(-1 * average_currents, -1 * final_positions, 14, 21, 91)

%% 

[average_currents, final_positions] = read_torque_batch("withoutknee");
fit_torques(-1 * average_currents, -1 * final_positions, 13, 13, 37)