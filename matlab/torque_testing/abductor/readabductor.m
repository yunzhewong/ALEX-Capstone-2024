close all
addpath("..")

component1_mass = 4.543;
component2_mass = 5.044;
extensor_mass = component1_mass + component2_mass;

% [average_currents, final_positions] = read_torque_batch("straightleg");
% fit_torques(average_currents, final_positions, 58, 58, 61);

%% 
[average_currents_withknee, final_positions_withknee] = read_torque_batch("withknee");
[mxl_full] = fit_torques(average_currents_withknee, final_positions_withknee, 17, 17, 40);

%% 
[average_currents, final_positions] = read_torque_batch("withoutkneeleg");
[mxl_abductor_extensor, ~, ~] = fit_torques(average_currents, final_positions, 36, 36, 55);

%% 
[average_currents, final_positions] = read_torque_batch("justend");
[mxl_end, ~, ~] = fit_torques(average_currents, final_positions, 55, 55, 61);

%%
full_leg_flat_current = 2.6655;
full_leg_flat_torque = 0.124 * 120 * full_leg_flat_current - 8.1003;
m_leg = m_knee + m_ext;
lgx_leg = full_leg_flat_torque / (m_leg * 9.81);

without_knee_flat_current = 2.1479;
without_knee_flat_torque = 0.124 * 120 * without_knee_flat_current - 8.1003;
lgx_leg_from_noknee = without_knee_flat_torque / (m_ext * 9.81);

lgy_leg = (m_ext * lgy_ext + m_knee * (l_ext + lgy_knee)) / m_leg;
adjust_angle_leg = atand(lgy_leg / lgx_leg);

adjusted_angles = final_positions_withknee - adjust_angle_leg + 90;
fit_torques(average_currents_withknee, adjusted_angles, 17, 17, 40);
