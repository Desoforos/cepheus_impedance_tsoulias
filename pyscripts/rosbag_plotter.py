#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import math
pi = math.pi
from scipy.signal import butter, filtfilt



def smoothlist(rawlist):
    incontact = False
    for i in range(len(rawlist)):
        if(i == 0):
            pass
        if(i>62.25*200):
            pass
        else:
            if(abs(rawlist[i])>0.1):
                incontact = True
            if(incontact):
                if(abs(rawlist[i])<0.01):
                    rawlist[i] = rawlist[i-1]
    return rawlist

def simplefilter(rawlist, lim):
    for i in range(1, len(rawlist)):
        if(abs(rawlist[i]-rawlist[i-1]) > lim):
            rawlist[i] = rawlist[i-1]
    
    return rawlist




# Bag file path
path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/"
bag_file = path + sys.argv[1] + ".bag"

# Initialize lists for storing data
time_stamps = []
xt_x, xd_x, xee_x = [], [], []
xt_y, xd_y, xee_y = [], [], []
xt_theta, xd_theta, xee_theta = [], [], []
xt_theta0, xd_theta0, xee_theta0 = [], [], []
fext_x, fext_x_raw = [], []
xt_x_raw, xt_y_raw, xt_theta_raw = [], [], []
xt_x_dot_raw, xt_y_dot_raw, xt_theta_dot_raw = [], [], []

xt_x_dot, xd_x_dot, xee_x_dot = [], [], []
xt_y_dot, xd_y_dot, xee_y_dot = [], [], []
xt_theta_dot, xd_theta_dot, xee_theta_dot = [], [], []
xt_theta0_dot, xd_theta0_dot, xee_theta0_dot = [], [], []

torquerw, torqueq1, torqueq2, torqueq3 = [], [], [], []






# Open bag file
bag = rosbag.Bag(bag_file)

# Read the bag file
for topic, msg, t in bag.read_messages(topics=['/cepheus/xt_x', '/cepheus/xd_x', '/cepheus/xee_x',
                                               '/cepheus/xt_y', '/cepheus/xd_y', '/cepheus/xee_y',
                                               '/cepheus/xt_theta', '/cepheus/xd_theta', '/cepheus/xee_theta',
                                               '/cepheus/xt_theta0', '/cepheus/xd_theta0', '/cepheus/xee_theta0',
                                               '/cepheus/ft_sensor_topic', '/cepheus/fextx_raw',
                                               '/cepheus/xt_x_dot', '/cepheus/xd_x_dot', '/cepheus/xee_x_dot',
                                               '/cepheus/xt_y_dot', '/cepheus/xd_y_dot', '/cepheus/xee_y_dot',
                                               '/cepheus/xt_theta_dot', '/cepheus/xd_theta_dot', '/cepheus/xee_theta_dot',
                                               '/cepheus/xt_theta0_dot', '/cepheus/xd_theta0_dot', '/cepheus/xee_theta0_dot',
                                               '/cepheus/torquerw', '/cepheus/torqueq1', '/cepheus/torqueq2', '/cepheus/torqueq3',
                                               '/cepheus/xt_x_raw','/cepheus/xt_y_raw','/cepheus/xt_theta_raw',
                                               '/cepheus/xt_x_dot_raw','/cepheus/xt_y_dot_raw','/cepheus/xt_theta_dot_raw']):
    # Time
    time_stamps.append(t.to_sec())

    # X values
    if topic == "/cepheus/xt_x":
        xt_x.append(msg.data)
    elif topic == "/cepheus/xd_x":
        xd_x.append(msg.data)
    elif topic == "/cepheus/xee_x":
        xee_x.append(msg.data)

    # Y values
    elif topic == "/cepheus/xt_y":
        xt_y.append(msg.data)
    elif topic == "/cepheus/xd_y":
        xd_y.append(msg.data)
    elif topic == "/cepheus/xee_y":
        xee_y.append(msg.data)

    # Theta values
    elif topic == "/cepheus/xt_theta":
        xt_theta.append(msg.data*180/pi)
    elif topic == "/cepheus/xd_theta":
        xd_theta.append(msg.data*180/pi)
    elif topic == "/cepheus/xee_theta":
        xee_theta.append(msg.data*180/pi)
    
    # Theta0 values
    elif topic == "/cepheus/xt_theta0":
        xt_theta0.append(msg.data*180/pi)
    elif topic == "/cepheus/xd_theta0":
        xd_theta0.append(msg.data*180/pi)
    elif topic == "/cepheus/xee_theta0":
        xee_theta0.append(msg.data*180/pi)


    # Xdot values
    if topic == "/cepheus/xt_x_dot":
        xt_x_dot.append(msg.data)
    elif topic == "/cepheus/xd_x_dot":
        xd_x_dot.append(msg.data)
    elif topic == "/cepheus/xee_x_dot":
        xee_x_dot.append(msg.data)

    # Ydot values
    elif topic == "/cepheus/xt_y_dot":
        xt_y_dot.append(msg.data)
    elif topic == "/cepheus/xd_y_dot":
        xd_y_dot.append(msg.data)
    elif topic == "/cepheus/xee_y_dot":
        xee_y_dot.append(msg.data)

    # Thetadot values
    elif topic == "/cepheus/xt_theta_dot":
        xt_theta_dot.append(msg.data*180/pi)
    elif topic == "/cepheus/xd_theta_dot":
        xd_theta_dot.append(msg.data*180/pi)
    elif topic == "/cepheus/xee_theta_dot":
        xee_theta_dot.append(msg.data*180/pi)
    
    #Theta0dot values
    elif topic == "/cepheus/xt_theta0_dot":
        xt_theta0_dot.append(msg.data*180/pi)
    elif topic == "/cepheus/xd_theta0_dot":
        xd_theta0_dot.append(msg.data*180/pi)
    elif topic == "/cepheus/xee_theta0_dot":
        xee_theta0_dot.append(msg.data*180/pi)    
    
    #Fextx values
    elif topic == "/cepheus/ft_sensor_topic":
        fext_x.append(-msg.data)
    
    elif topic == "/cepheus/fextx_raw":
        fext_x_raw.append(-msg.data)
    
    #Torque values
    # elif topic == "/cepheus/torquerw":
    #     torquerw.append(msg.data)
    # elif topic == "/cepheus/torqueq1":
    #     torqueq1.append(msg.data)
    # elif topic == "/cepheus/torqueq2":
    #     torqueq2.append(msg.data)
    # elif topic == "/cepheus/torqueq3":
    #     torqueq3.append(msg.data)

    # gia actual motors (*186 gia actual roph)
    elif topic == "/cepheus/torquerw":
        torquerw.append(msg.data)
    elif topic == "/cepheus/torqueq1":
        torqueq1.append(-186*msg.data)
    elif topic == "/cepheus/torqueq2":
        torqueq2.append(186*msg.data)
    elif topic == "/cepheus/torqueq3":
        torqueq3.append(-186*msg.data)
    
    elif topic == "/cepheus/xt_x_raw":
        xt_x_raw.append(msg.data)
    elif topic == "/cepheus/xt_y_raw":
        xt_y_raw.append(msg.data)
    elif topic == "/cepheus/xt_theta_raw":
        xt_theta_raw.append(msg.data)





bag.close()

# Convert time stamps to relative time
# time_stamps = np.array(time_stamps)

#diorthosh
sampling_frequency = 100  # Hz
time_stamps= np.arange(0, len(xt_x)) / sampling_frequency
time_stamps -= time_stamps[0]  # Start from 0

desired_secs = 60

# des_len = sampling_frequency*desired_secs
des_len = len(xee_x)


print("xstep[0] is: ",xd_x[0])

print("xstep[1] is: ",xd_x[1])
print("xee[100] is: ",xee_x[100])

print("xstepdot[100] is: ",xd_x_dot[100])
print("xeedot[100] is: ",xee_x_dot[100])

print("error[100] is: ",xd_x[1] - xee_x[100])
print("errordot[100] is: ",xd_x_dot[1] - xee_x_dot[100])

print("torque3[0] is: ",torqueq3[0])
print("torque3[1] is: ",torqueq3[1])
print("torque3[100] is: ",torqueq3[100])

print("theta[1] is: ", xee_theta[200])
print("theta0[1] is: ", xee_theta0[200])


error_x = []
error_xdot = []

error_y = []
error_ydot = []

error_theta = []
error_thetadot = []

for i in range(len(xee_x)):
    error_x.append(xd_x[i]-xee_x[i])
    error_xdot.append(xd_x_dot[i]-xee_x_dot[i])

    error_y.append(xd_y[i]-xee_y[i])
    error_ydot.append(xd_y_dot[i]-xee_y_dot[i])

    error_theta.append(xd_theta[i]-xee_theta[i])
    error_thetadot.append(xd_theta_dot[i]-xee_theta_dot[i])

# simplefilter(xee_x_dot, 0.01)
# simplefilter(xee_y_dot, 0.01)
# simplefilter(xee_theta_dot, 3)




#smoothen the plots:
# xee_x = savgol_filter(xee_x, window_length=51, polyorder=3)
# xee_y = savgol_filter(xee_y, window_length=51, polyorder=3)
# xee_theta = savgol_filter(xee_theta, window_length=51, polyorder=3)
# xee_theta0 = savgol_filter(xee_theta0, window_length=51, polyorder=3)
# # # fext_x = savgol_filter(fext_x, window_length=51, polyorder=3)  #mou xalaei ta arnhtika
# xee_x_dot = savgol_filter(xee_x_dot, window_length=51, polyorder=3)
# xee_y_dot = savgol_filter(xee_y_dot, window_length=51, polyorder=3)
# xee_theta_dot = savgol_filter(xee_theta_dot, window_length=51, polyorder=3)
# xee_theta0_dot = savgol_filter(xee_theta0_dot, window_length=51, polyorder=3)
# torquerw = savgol_filter(torquerw, window_length=51, polyorder=3)
# torqueq1 = savgol_filter(torqueq1, window_length=51, polyorder=3)
# torqueq2 = savgol_filter(torqueq2, window_length=51, polyorder=3)
# torqueq3 = savgol_filter(torqueq3, window_length=51, polyorder=3)

# 1. Threshold to detect the contact phase
# threshold = -0.5
# fext_xarr = np.array(fext_x)
# contact_indices = np.where(fext_xarr < threshold)[0]

# 2. Interpolate only within the contact phase
# fext_interp = fext_xarr.copy()
# if len(contact_indices) > 0:
#     # Replace non-contact values within the contact phase with interpolated values
#     contact_times = np.arange(len(fext_xarr))[contact_indices]
#     interpolator = interp1d(contact_times, fext_xarr[contact_indices], kind='linear', fill_value="extrapolate")
#     non_contact_indices = np.where((fext_xarr >= threshold) & (np.arange(len(fext_xarr)) > contact_indices[0]))[0]
#     fext_interp[non_contact_indices] = interpolator(non_contact_indices)


# # Create plots

# # X values (Target, Desired, Actual)
# plt.figure()
# plt.plot(time_stamps[:des_len], xt_x, label='Target X', color='r')
# plt.plot(time_stamps[:des_len], xd_x, label='Desired X', color='g')
# plt.plot(time_stamps[:des_len], xee_x, label='Actual X', color='b')
# plt.grid()
# plt.title('X values')
# plt.xlabel('Time [s]')
# plt.ylabel('X Position [m]')
# plt.legend()

# # Y values (Target, Desired, Actual)
# plt.figure()
# plt.plot(time_stamps[:des_len], xt_y, label='Target Y', color='r')
# plt.plot(time_stamps[:des_len], xd_y, label='Desired Y', color='g')
# plt.plot(time_stamps[:des_len], xee_y, label='Actual Y', color='b')
# plt.grid()
# plt.title('Y values')
# plt.xlabel('Time [s]')
# plt.ylabel('Y Position [m]')
# plt.legend()

# # Theta values (Target, Desired, Actual)
# plt.figure()
# plt.plot(time_stamps[:des_len], xt_theta, label='Target Theta', color='r')
# plt.plot(time_stamps[:des_len], xd_theta, label='Desired Theta', color='g')
# plt.plot(time_stamps[:des_len], xee_theta, label='Actual Theta', color='b')
# plt.grid()
# plt.title('Theta values')
# plt.xlabel('Time [s]')
# plt.ylabel('Theta [deg]')
# plt.legend()

# # External Force (X-axis)
# plt.figure()
# plt.plot(time_stamps[:des_len], fext_x, label='Fext X', color='k')
# plt.grid()
# plt.title('External Force (X-axis)')
# plt.xlabel('Time [s]')
# plt.ylabel('Force [N]')
# plt.legend()

# # Show all plots
# plt.show()

# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot X values (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:des_len], xt_x[:des_len], label='Target X', color='r')
axs[0, 0].plot(time_stamps[:des_len], xd_x[:des_len], label='Desired X', color='g',linestyle='--')
axs[0, 0].plot(time_stamps[:des_len], xee_x[:des_len], label='Actual X', color='b')
# axs[0, 0].scatter(time_stamps[:des_len], xee_x[:des_len], label='Actual X', color='y')
# axs[0, 0].plot(time_stamps[:des_len], xt_x_raw[:des_len], label='Raw target X', color='y')
axs[0, 0].grid()
axs[0, 0].set_title('X values')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X Position [m]')
axs[0, 0].legend()

# Plot Y values (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:des_len], xt_y[:des_len], label='Target Y', color='r')
axs[0, 1].plot(time_stamps[:des_len], xd_y[:des_len], label='Desired Y', color='g',linestyle='--')
axs[0, 1].plot(time_stamps[:des_len], xee_y[:des_len], label='Actual Y', color='b')
# axs[0, 1].scatter(time_stamps[:des_len], xee_y[:des_len], label='Actual Y', color='y')
# axs[0, 1].plot(time_stamps[:des_len], xt_y_raw[:des_len], label='Raw target Y', color='y')
axs[0, 1].grid()
axs[0, 1].set_title('Y values')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y Position [m]')
axs[0, 1].legend()

# Plot Theta values (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:des_len], xt_theta[:des_len], label='Target Theta', color='r')
axs[1, 0].plot(time_stamps[:des_len], xd_theta[:des_len], label='Desired Theta', color='g',linestyle='--')
axs[1, 0].plot(time_stamps[:des_len], xee_theta[:des_len], label='Actual Theta', color='b')
# axs[1, 0].scatter(time_stamps[:des_len], xee_theta[:des_len], label='Actual Theta', color='y')
# axs[1, 0].plot(time_stamps[:des_len], xt_theta_raw[:des_len], label='Raw target Theta', color='y')
axs[1, 0].grid()
axs[1, 0].set_title('Theta values')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Theta [deg]')
axs[1, 0].legend()

# Plot Theta0 values (Target, Desired, Actual)
# axs[1, 1].plot(time_stamps[:des_len], xt_theta0[:des_len], label='Target Theta0', color='r')
axs[1, 1].plot(time_stamps[:des_len], xd_theta0[:des_len], label='Desired Theta0', color='g',linestyle='--')
axs[1, 1].plot(time_stamps[:des_len], xee_theta0[:des_len], label='Actual Theta0', color='b')
# axs[1, 1].scatter(time_stamps[:des_len], xee_theta0[:des_len], label='Actual Theta0', color='y')
axs[1, 1].grid()
axs[1, 1].set_title('Theta0 values')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Theta0 [deg]')
axs[1, 1].legend()


# # Plot External Force (X-axis)
# axs[1, 1].plot(time_stamps[:des_len], fext_x, label='Fext X', color='k')
# axs[1, 1].grid()
# axs[1, 1].set_title('External Force (X-axis)')
# axs[1, 1].set_xlabel('Time [s]')
# axs[1, 1].set_ylabel('Force [N]')
# axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()


fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot Xdot values (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:des_len], xt_x_dot[:des_len], label='Target Xdot', color='r')
axs[0, 0].plot(time_stamps[:des_len], xd_x_dot[:des_len], label='Desired Xdot', color='g',linestyle='--')
axs[0, 0].plot(time_stamps[:des_len], xee_x_dot[:des_len], label='Actual Xdot', color='b')
# axs[0, 0].scatter(time_stamps[:des_len], xee_x_dot[:des_len], label='Actual Xdot', color='y')

axs[0, 0].grid()
axs[0, 0].set_title('Xdot ')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X Velcotiy [m/sec]')
axs[0, 0].legend()

# Plot Ydot values (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:des_len], xt_y_dot[:des_len], label='Target Ydot', color='r')
axs[0, 1].plot(time_stamps[:des_len], xd_y_dot[:des_len], label='Desired Ydot', color='g',linestyle='--')
axs[0, 1].plot(time_stamps[:des_len], xee_y_dot[:des_len], label='Actual Ydot', color='b')
# axs[0, 1].scatter(time_stamps[:des_len], xee_y_dot[:des_len], label='Actual Ydot', color='y')
axs[0, 1].grid()
axs[0, 1].set_title('Ydot')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y Velocity [m/sec]')
axs[0, 1].legend()

# Plot Thetadot values (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:des_len], xt_theta_dot[:des_len], label='Target Thetadot', color='r')
axs[1, 0].plot(time_stamps[:des_len], xd_theta_dot[:des_len], label='Desired Thetadot', color='g',linestyle='--')
axs[1, 0].plot(time_stamps[:des_len], xee_theta_dot[:des_len], label='Actual Thetadot', color='b')
# axs[1, 0].scatter(time_stamps[:des_len], xee_theta_dot[:des_len], label='Actual Thetadot', color='y')
axs[1, 0].grid()
axs[1, 0].set_title('Thetadot')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Thetadot [deg/sec]')
axs[1, 0].legend()

# Plot Theta0dot values (Target, Desired, Actual)
# axs[1, 1].plot(time_stamps[:des_len], xt_theta0_dot[:des_len], label='Target Theta0dot', color='r')
axs[1, 1].plot(time_stamps[:des_len], xd_theta0_dot[:des_len], label='Desired Theta0dot', color='g',linestyle='--')
axs[1, 1].plot(time_stamps[:des_len], xee_theta0_dot[:des_len], label='Actual Theta0dot', color='b')
# axs[1, 1].scatter(time_stamps[:des_len], xee_theta0_dot[:des_len], label='Actual Theta0dot', color='y')
axs[1, 1].grid()
axs[1, 1].set_title('Theta0dot')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Theta0dot [deg/sec]')
axs[1, 1].legend()


# # Plot External Force (X-axis)
# axs[1, 1].plot(time_stamps[:des_len], fext_x, label='Fext X', color='k')
# axs[1, 1].grid()
# axs[1, 1].set_title('External Force (X-axis)')
# axs[1, 1].set_xlabel('Time [s]')
# axs[1, 1].set_ylabel('Force [N]')
# axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()





fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot torquerw
axs[0, 0].plot(time_stamps[:des_len], torquerw[:des_len], label='Base torque', color='b')
axs[0, 0].grid()
axs[0, 0].set_title('Base torque ')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('Base torque [Nm]')
axs[0, 0].legend()

# Plot torqueq1
axs[0, 1].plot(time_stamps[:des_len], torqueq1[:des_len], label='Joint1 torque', color='b')
axs[0, 1].grid()
axs[0, 1].set_title('Joint1 torque')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Joint1 torque [Nm]')
axs[0, 1].legend()

# Plot torqueq2
axs[1, 0].plot(time_stamps[:des_len], torqueq2[:des_len], label='Joint2 torque', color='b')
axs[1, 0].grid()
axs[1, 0].set_title('Joint2 torque')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Joint2 torque [Nm]')
axs[1, 0].legend()

# Plot torqueq3
axs[1, 1].plot(time_stamps[1:des_len], torqueq3[1:des_len], label='Joint3 torque', color='b')
axs[1, 1].grid()
axs[1, 1].set_title('Joint3 torque')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Joint3 torque [Nm]')
axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()
# fext_smoothed = fext_x.copy()
# fext_smoothed = smoothlist(fext_smoothed)

# for i in range(0,200*10):
#     fext_x[i] = 0

# External Force (X-axis)
plt.figure()
plt.plot(time_stamps[:des_len], fext_x[:des_len], label='Fext X', color='k')
plt.plot(time_stamps[:des_len], fext_x_raw[:des_len], label='Fext X RAW', color='g')
# plt.plot(time_stamps[:des_len], fext_smoothed[:des_len], label='Fext_x')
plt.grid()
plt.title('External Force (X-axis)')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()

# Show all plots
plt.show()




# time_before_contact = time_stamps[:des_len]
# torque_before_contact = torquerw[:des_len]

# plt.figure(figsize=(10, 6))

# plt.plot(time_before_contact, torque_before_contact, label="Before Contact (Detailed)")
# plt.grid()

# plt.xlabel("Time (s)")
# plt.ylabel("Torque (Nm)")
# plt.title("Torque of Joint Over Time")
# plt.legend()
# plt.show()

# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot X values (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:des_len], error_x[:des_len], label='error X', color='r')
axs[0, 0].plot(time_stamps[:des_len], error_xdot[:des_len], label='error Xdot', color='b')
axs[0, 0].grid()
axs[0, 0].legend()
axs[0, 0].set_title("errors X")

# Plot Y values (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:des_len], error_y[:des_len], label='error Y', color='r')
axs[0, 1].plot(time_stamps[:des_len], error_ydot[:des_len], label='error Ydot', color='b')
axs[0, 1].grid()
axs[0, 1].legend()
axs[0, 1].set_title("errors Y")

# Plot Theta values (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:des_len], error_theta[:des_len], label='error Theta', color='r')
axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='b')
axs[1, 0].grid()
axs[1, 0].legend()
axs[1, 0].set_title("errors Theta")

# # Plot Theta0 values (Target, Desired, Actual)
# # axs[1, 1].plot(time_stamps[:des_len], xt_theta0[:des_len], label='Target Theta0', color='r')
# axs[1, 1].plot(time_stamps[:des_len], xd_theta0[:des_len], label='Desired Theta0', color='g',linestyle='--')
# axs[1, 1].plot(time_stamps[:des_len], xee_theta0[:des_len], label='Actual Theta0', color='b')
# axs[1, 1].grid()
# axs[1, 1].set_title('Theta0 values')
# axs[1, 1].set_xlabel('Time [s]')
# axs[1, 1].set_ylabel('Theta0 [deg]')
# axs[1, 1].legend()
plt.show()


ydot = xee_y_dot
fs = 100
n = len(ydot)
frequencies = np.fft.fftfreq(n, d=1/fs)
fft_magnitude = np.abs(np.fft.fft(ydot))

# Plot FFT
plt.figure(figsize=(8,4))
plt.plot(frequencies[:n//2], fft_magnitude[:n//2])  # Only positive frequencies
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("FFT of ydot signal")
plt.grid()
plt.show()

def butter_lowpass_filter(data, cutoff, fs, order=2):
    nyquist = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyquist  # Normalize cutoff frequency
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)  # Apply filter

filtered_ydot = butter_lowpass_filter(ydot, cutoff=0.8, fs=100, order=2)

plt.plot(time_stamps[:des_len], filtered_ydot[:des_len], label='ydot', color='r')
plt.plot(time_stamps[:des_len], xd_y_dot[:des_len], label='Desired Ydot', color='g',linestyle='--')
plt.title("Butterworth 2nd order filter with fc = 0.8Hz")
plt.xlabel("Time (sec) ")
plt.ylabel("Ydot (m/s)")
plt.grid()
plt.show()


y = xee_y
fs = 100
n = len(y)
frequencies = np.fft.fftfreq(n, d=1/fs)
fft_magnitude = np.abs(np.fft.fft(y))

# Plot FFT
plt.figure(figsize=(8,4))
plt.plot(frequencies[:n//2], fft_magnitude[:n//2])  # Only positive frequencies
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("FFT of y signal")
plt.grid()
plt.show()

filtered_y = butter_lowpass_filter(ydot, cutoff=5, fs=100, order=2)

plt.plot(time_stamps[:des_len], filtered_y[:des_len], label='y', color='r')
plt.plot(time_stamps[:des_len], xd_y[:des_len], label='Desired Ydot', color='g',linestyle='--')
plt.title("Butterworth 2nd order filter with fc = 5Hz")
plt.xlabel("Time (sec) ")
plt.ylabel("Y (m) ")
plt.grid()
plt.show()