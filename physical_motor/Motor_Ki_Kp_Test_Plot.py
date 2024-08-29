import matplotlib.pyplot as plt
import pandas as pd
import os
import math
import numpy as np
from scipy.integrate import cumulative_trapezoid


# Kp_pos = 1
# Kp_vel = 0.5
# Ki_vel = 0.1

# fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
# file_n = f'Vel_C Kp_pos={Kp_pos}, Kp_vel={Kp_vel}, Ki_vel={Ki_vel}'
# file_p = file_n + '.csv'
# df = pd.read_csv(file_p, on_bad_lines='skip')
# df = df.sort_values(by='Time')

# t = df['Time'].to_numpy()
# c = df[' Motor Current'].to_numpy()
# v = df[' Motor Velocities'].to_numpy()
# p = df[' Motor Positions'].to_numpy()
# # ax1.plot(t, c, label=f'Position gain = {gain}')
# # ax1.set_ylabel("Current (A)")

# ax1.plot(t, v, label=f'Pos_gain={Kp_pos}, Vel_gain={Kp_vel}, vel_i={Ki_vel}')
# ax1.set_ylabel("Velocity (rad/s)")

# ax2.plot(t, p, label=f'Pos_gain={Kp_pos}, Vel_gain={Kp_vel}, vel_i={Ki_vel}')
# ax2.set_ylabel("Position (rad)")
    
# t = np.arange(0, 7, 0.01)
# v_r = np.sin(t)
# p_r = cumulative_trapezoid(v_r, t, initial=0)
# ax1.plot(t, v_r, label='Reference', color='k')
# ax2.plot(t, p_r, label='Reference', color='k')
# plt.legend()
# plt.show()

Kp_pos = 1
Kp_vel = 0.5
Ki_vel_list = [0.0002, 0.002, 0.02, 0.1, 0.2]


fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
for Ki_vel in Ki_vel_list:
    file_n = f'Vel_C Kp_pos={Kp_pos}, Kp_vel={Kp_vel}, Ki_vel={Ki_vel}'
    file_p = file_n + '.csv'
    df = pd.read_csv(file_p, on_bad_lines='skip')
    df = df.sort_values(by='Time')

    t = df['Time'].to_numpy()
    c = df[' Motor Current'].to_numpy()
    v = df[' Motor Velocities'].to_numpy()
    p = df[' Motor Positions'].to_numpy()
    # ax1.plot(t, c, label=f'Position gain = {gain}')
    # ax1.set_ylabel("Current (A)")

    ax1.plot(t, v, label=f'Pos_gain={Kp_pos}, Vel_gain={Kp_vel}, vel_i={Ki_vel}')
    ax1.set_ylabel("Velocity (rad/s)")

    ax2.plot(t, p, label=f'Pos_gain={Kp_pos}, Vel_gain={Kp_vel}, vel_i={Ki_vel}')
    ax2.set_ylabel("Position (rad)")
        
    t = np.arange(0, 7, 0.01)
    v_r = np.sin(t)
    p_r = cumulative_trapezoid(v_r, t, initial=0)
    ax1.plot(t, v_r, label='Reference', color='k')
    ax2.plot(t, p_r, label='Reference', color='k')
plt.legend()
plt.show()


# gain_values = [1, 5, 8]
# plt.figure()
# for gain in gain_values:
    
#     file_n = 'Pos_gain=10, Vel_gain = ' + str(gain)
#     file_p = file_n + '.csv'


#     df = pd.read_csv(file_p, on_bad_lines='skip')
#     df = df.sort_values(by='Time')

#     x = df['Time'].to_numpy()
#     y = df[' Motor Positions'].to_numpy()
#     plt.plot(x, y, label=f'Position gain = {gain}')


# file_n = 'Pos_gain=1, Vel_gain = 10'
# file_p = file_n + '.csv'
# df = pd.read_csv(file_p, on_bad_lines='skip')
# df = df.sort_values(by='Time')

# x = df['Time'].to_numpy()
# y = df[' Motor Positions'].to_numpy()
# plt.figure
# plt.plot(x, y, label='Pos_gain=1, Vel_gain = 10')

# file_n = 'Pos gain = 1'
# file_p = file_n + '.csv'
# df = pd.read_csv(file_p, on_bad_lines='skip')
# df = df.sort_values(by='Time')

# x = df['Time'].to_numpy()
# y = df[' Motor Positions'].to_numpy()
# plt.figure
# plt.plot(x, y, label='Pos_gain=1, Vel_gain = 1')

# x = np.arange(0, 7, 0.01)
# y = np.sin(x)
# plt.plot(x, y, label='Reference', color='k')
# plt.legend()
# plt.show()

# gain_values = [1, 5, 10, 15, 20, 50]
# fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
# for gain in gain_values:
    
#     file_n = 'Pos gain = ' + str(gain)
#     file_p = file_n + '.csv'
#     df = pd.read_csv(file_p, on_bad_lines='skip')
#     df = df.sort_values(by='Time')

#     t = df['Time'].to_numpy()
#     c = df[' Motor Current'].to_numpy()
#     v = df[' Motor Velocities'].to_numpy()
#     p = df[' Motor Positions'].to_numpy()
#     ax1.plot(t, c, label=f'Position gain = {gain}')
#     ax1.set_ylabel("Current (A)")

#     ax2.plot(t, v, label=f'Position gain = {gain}')
#     ax1.set_ylabel("Velocity (rad/s)")

#     ax3.plot(t, p, label=f'Position gain = {gain}')
#     ax1.set_ylabel("Current (rad)")
    

# t = np.arange(0, 7, 0.01)
# p = np.sin(t)
# plt.plot(t, p, label='Reference', color='k')
# plt.legend()
# plt.show()
