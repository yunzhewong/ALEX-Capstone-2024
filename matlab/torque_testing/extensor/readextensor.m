close all
addpath("..")

[average_currents, final_positions] = read_torque_batch("withknee");
[mxl_with, ~, ~] = fit_torques(-1 * average_currents, -1 * final_positions, 14, 21, 91);

%% 

[average_currents, final_positions] = read_torque_batch("withoutknee");
[mxl_without, ~, ~] = fit_torques(-1 * average_currents, -1 * final_positions, 13, 13, 37);

mxl_knee = 1.5194;
m_knee = 5.77;

hip_to_knee = 0.44;


mxl_without + m_knee * hip_to_knee + mxl_knee