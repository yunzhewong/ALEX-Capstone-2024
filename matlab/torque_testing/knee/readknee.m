close all
addpath("..")

[average_currents, final_positions] = read_torque_batch("test1");
[mxl_1, ~, ~] = fit_torques(average_currents, final_positions, 1, 1, 9);

%% 

[average_currents, final_positions] = read_torque_batch("test2");
[mxl_2, ~, ~] = fit_torques(average_currents, final_positions, 15, 15, 33);

mxl_knee = (mxl_1 + mxl_2) / 2