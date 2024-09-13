close all
addpath("..")

[average_currents, final_positions] = read_torque_batch("withknee");
[mxl_with, ~, ~] = fit_torques(-1 * average_currents, -1 * final_positions, 14, 21, 91);

%% 

[average_currents, final_positions] = read_torque_batch("withoutknee");
[mxl_without, ~, ~] = fit_torques(-1 * average_currents, -1 * final_positions, 13, 13, 37);

l_ext = 0.44;
component1_mass = 4.543;
component2_mass = 5.044;
m_ext = component1_mass + component2_mass;
lgy_ext = mxl_without / m_ext;

measured = mxl_with;
estimated = lgy_ext * m_ext + (l_ext + lgy_knee) * m_knee;

fprintf("Measured Lumped Leg Mass Length: %.4f\n", measured);
fprintf("Estimated Lumped Leg Mass Length: %.4f\n", estimated);