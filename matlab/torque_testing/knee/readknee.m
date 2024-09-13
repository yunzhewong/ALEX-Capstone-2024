close all
addpath("..")

[average_currents, final_positions] = read_torque_batch("test1");
fit_torques(average_currents, final_positions, 1, 1, 9);

%% 
[average_currents, final_positions] = read_torque_batch("test2");
[mxl_knee, ~, ~] = fit_torques(average_currents, final_positions, 15, 15, 33);

m_knee = 5.77;
lgy_knee = mxl_2 / m_knee;