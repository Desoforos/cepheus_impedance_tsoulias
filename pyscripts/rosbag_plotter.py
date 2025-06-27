#!/usr/bin/env python
# afto xrisimopoihsa sti diplomatiki, ostoso meta gia to ioant tou evala ki alla plots pou den yparxoun sto .bag tis diplomaitkis (13-2-imp6.bag)
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import math
pi = math.pi
from scipy.signal import butter, filtfilt
import scipy.signal as signal
import numpy as np
import scipy.signal as signal
import scipy.ndimage as ndimage
import scipy.signal as signal
import scipy.ndimage as ndimage
import scipy.io
import os



def smooth_signal(y, method="savgol", window_size=21, polyorder=3, sigma=2, alpha=0.1):
    """
    Smooths the entire time series.

    Parameters:
    - y: numpy array, the time series of position y.
    - method: str, smoothing method ("savgol", "gaussian", "ema").
    - window_size: int, window size for Savitzky-Golay filter (must be odd).
    - polyorder: int, polynomial order for Savitzky-Golay filter.
    - sigma: float, standard deviation for Gaussian filter.
    - alpha: float, smoothing factor for Exponential Moving Average (EMA).

    Returns:
    - y_smooth: numpy array, smoothed signal.
    """

    if method == "savgol":
        y_smooth = signal.savgol_filter(y, window_size, polyorder)
    elif method == "gaussian":
        y_smooth = ndimage.gaussian_filter1d(y, sigma)
    elif method == "ema":
        y_smooth = np.zeros_like(y)
        y_smooth[0] = y[0]
        for i in range(1, len(y)):
            y_smooth[i] = alpha * y[i] + (1 - alpha) * y_smooth[i - 1]
    else:
        raise ValueError("Invalid method. Choose 'savgol', 'gaussian', or 'ema'.")

    return y_smooth



def smooth_sudden_movement(y, sample_rate=100, smooth_range=(7, 8), window_size=21, polyorder=3):
    """
    Smooths a sudden movement in the specified time range while preserving the overall trajectory.

    Parameters:
    - y: numpy array, time series of position y.
    - sample_rate: int, sample rate in Hz (default 100 Hz).
    - smooth_range: tuple, time range (start, end) in seconds to apply smoothing.
    - window_size: int, size of the smoothing window (must be odd for Savitzky-Golay filter).
    - polyorder: int, polynomial order for Savitzky-Golay filter.

    Returns:
    - y_filtered: numpy array, smoothed signal.
    """

    # Convert time range to indices
    start_idx = int(smooth_range[0] * sample_rate)
    end_idx = int(smooth_range[1] * sample_rate)

    # Apply Savitzky-Golay filter only in the given range
    y_smoothed = y.copy()
    y_smoothed[start_idx:end_idx] = signal.savgol_filter(y[start_idx:end_idx], window_size, polyorder)

    return y_smoothed



    


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

def moving_average_filter(data, window_size):
    """
    Smoothens a time series using a moving average filter.
    
    Parameters:
    - data: list or numpy array, the time series data.
    - window_size: int, the size of the moving average window.
    
    Returns:
    - smoothed_data: numpy array, the smoothened time series.
    """
    if window_size < 1:
        raise ValueError("Window size must be at least 1.")
    
    # Use numpy's convolution function for a moving average
    kernel = np.ones(window_size) / window_size
    smoothed_data = np.convolve(data, kernel, mode='same')  # 'same' keeps the same length as input
    
    return smoothed_data



# Bag file path
path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/"
bag_file = path + sys.argv[1] + ".bag"

path2 = "/home/desoforos/Desktop/IOANT_proti_fash/gia_rek/"



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

error_x = []
error_xdot = []

error_y = []
error_ydot = []

error_theta = []
error_thetadot = []

error_theta0 = []
error_theta0dot =[]

acc1_x , acc1_y, acc1_z = [], [], []
acc2_x , acc2_y, acc2_z = [], [], []
acc3_x , acc3_y, acc3_z = [], [], []

imu_acc_x , imu_acc_y, imu_vel_theta = [], [] ,[]







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
                                               '/cepheus/xt_x_dot_raw','/cepheus/xt_y_dot_raw','/cepheus/xt_theta_dot_raw',
                                               '/cepheus/acc1_x', '/cepheus/acc1_y', '/cepheus/acc1_z','/cepheus/acc2_x', '/cepheus/acc2_y', '/cepheus/acc2_z',
                                                '/cepheus/acc3_x', '/cepheus/acc3_y', '/cepheus/acc3_z',
                                                '/cepheus/imu_acc_x','/cepheus/imu_acc_y', '/cepheus/imu_vel_theta']):
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

    # gia Measured motors (*186 gia Measured roph)
    elif topic == "/cepheus/torquerw":
        torquerw.append(msg.data)
    elif topic == "/cepheus/torqueq1":
        torqueq1.append(-msg.data)
    elif topic == "/cepheus/torqueq2":
        torqueq2.append(msg.data)
    elif topic == "/cepheus/torqueq3":
        torqueq3.append(-msg.data)
    
    elif topic == "/cepheus/xt_x_raw":
        xt_x_raw.append(msg.data)
    elif topic == "/cepheus/xt_y_raw":
        xt_y_raw.append(msg.data)
    elif topic == "/cepheus/xt_theta_raw":
        xt_theta_raw.append(msg.data)
    
    elif topic == "/cepheus/acc1_x":
        acc1_x.append(msg.data)
    elif topic == "/cepheus/acc1_y":
        acc1_y.append(msg.data)
    elif topic == "/cepheus/acc1_z":
        acc1_z.append(msg.data)

    elif topic == "/cepheus/acc2_x":
        acc2_x.append(msg.data)
    elif topic == "/cepheus/acc2_y":
        acc2_y.append(msg.data)
    elif topic == "/cepheus/acc2_z":
        acc2_z.append(msg.data)

    elif topic == "/cepheus/acc3_x":
        acc3_x.append(msg.data)
    elif topic == "/cepheus/acc3_y":
        acc3_y.append(msg.data)
    elif topic == "/cepheus/acc3_z":
        acc3_z.append(msg.data)
    
    elif topic == "/cepheus/imu_acc_x":
        imu_acc_x.append(msg.data)
    elif topic == "/cepheus/imu_acc_y":
        imu_acc_y.append(msg.data)
    elif topic == "/cepheus/imu_vel_theta":
        imu_vel_theta.append(msg.data)
        
              



bag.close()

# Convert time stamps to relative time
# time_stamps = np.array(time_stamps)

#diorthosh
sampling_frequency = 100  # Hz
time_stamps= np.arange(0, len(torqueq1)) / sampling_frequency
time_stamps -= time_stamps[0]  # Start from 0

desired_secs = 19

des_len = sampling_frequency*desired_secs
# des_len = len(xee_x)


# print("xee_x[0] is ",xee_x[0])
# print("xee_y[0] is ",xee_y[0])
# print("xee_theta[0] is ",xee_theta[0])
# print("theta0[0] is ",xee_theta0[0])

# print("xt_x[0] is",xt_x[0])
# print("xt_y[0] is",xt_y[0])
# print("xt_theta[0] is",xt_theta[0])



# print("xstep[0] is: ",xd_x[0])

# print("xstep[1] is: ",xd_x[1])
# print("xee[100] is: ",xee_x[100])

# print("xstepdot[100] is: ",xd_x_dot[100])
# print("xeedot[100] is: ",xee_x_dot[100])

# print("error[100] is: ",xd_x[1] - xee_x[100])
# print("errordot[100] is: ",xd_x_dot[1] - xee_x_dot[100])

# print("torque3[0] is: ",torqueq3[0])
# print("torque3[1] is: ",torqueq3[1])
# print("torque3[100] is: ",torqueq3[100])

# print("theta[1] is: ", xee_theta[200])
# print("theta0[1] is: ", xee_theta0[200])



# simplefilter(xee_x_dot, 0.01)
# simplefilter(xee_y_dot, 0.01)
# simplefilter(xee_theta_dot, 3)

# xee_y = moving_average_filter(xee_y,10)
# xee_y_filt = smooth_sudden_movement(xee_y)


# simplesmooth(xee_y,7.2,8.2,100) #gia 13-2-imp6
# xee_y_smoothed = xee_y.copy()
# xee_y_smoothed = smooth_signal(xee_y_smoothed, method ="gaussian") #gia 13-2-imp6

# xee_y = xee_y_smoothed
###############na ta ksananoikso##############33
# xee_y_dot = smooth_signal(xee_y_dot,method="ema")
# xee_x_dot = smooth_signal(xee_x_dot,method="ema")

# xee_theta_dot = smooth_signal(xee_theta_dot,method="ema")


# makezeros(xd_x_dot)
# makezeros(xd_y_dot)
# makezeros(xd_theta_dot)

# for i in range(len(xee_x)):
#     error_x.append(-(xd_x[i]-xee_x[i]))
#     error_xdot.append(-(xd_x_dot[i]-xee_x_dot[i]))

#     error_y.append(-(xd_y[i]-xee_y[i]))
#     error_ydot.append(-(xd_y_dot[i]-xee_y_dot[i]))

#     error_theta.append(-(xd_theta[i]-xee_theta[i]))
#     error_thetadot.append(-(xd_theta_dot[i]-xee_theta_dot[i]))

#     error_theta0.append(-(xd_theta0[i]-xee_theta0[i]))
#     error_theta0dot.append(-(xd_theta0_dot[i]-xee_theta0_dot[i]))
####################na ta ksananoikso##############################

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

# # X values (Target, Desired, Measured)
# plt.figure()
# plt.plot(time_stamps[:des_len], xt_x, label='Target X', color='r')
# plt.plot(time_stamps[:des_len], xd_x, label='Desired X', color='g')
# plt.plot(time_stamps[:des_len], xee_x, label='Measured X', color='b')
# plt.grid()
# plt.title('X values')
# plt.xlabel('Time [s]')
# plt.ylabel('X Position [m]')
# plt.legend()

# # Y values (Target, Desired, Measured)
# plt.figure()
# plt.plot(time_stamps[:des_len], xt_y, label='Target Y', color='r')
# plt.plot(time_stamps[:des_len], xd_y, label='Desired Y', color='g')
# plt.plot(time_stamps[:des_len], xee_y, label='Measured Y', color='b')
# plt.grid()
# plt.title('Y values')
# plt.xlabel('Time [s]')
# plt.ylabel('Y Position [m]')
# plt.legend()

# # Theta values (Target, Desired, Measured)
# plt.figure()
# plt.plot(time_stamps[:des_len], xt_theta, label='Target Theta', color='r')
# plt.plot(time_stamps[:des_len], xd_theta, label='Desired Theta', color='g')
# plt.plot(time_stamps[:des_len], xee_theta, label='Measured Theta', color='b')
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

#apo edo
# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot X values (Target, Desired, Measured)
# axs[0, 0].plot(time_stamps[:des_len], xt_x[:des_len], label='Target X', color='r')
# axs[0, 0].plot(time_stamps[:des_len], xd_x[:des_len], label='Desired X', color='g',linestyle='--') #na to anoikso
axs[0, 0].plot(time_stamps[:des_len], xee_x[:des_len], label='Measured X', color='b')
# axs[0, 0].scatter(time_stamps[:des_len], xee_x[:des_len], label='Measured X', color='y')
# axs[0, 0].plot(time_stamps[:des_len], xt_x_raw[:des_len], label='Raw target X', color='y')
axs[0, 0].grid()
axs[0, 0].set_title('X values')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X Position [m]')
axs[0, 0].legend()

# Plot Y values (Target, Desired, Measured)
# axs[0, 1].plot(time_stamps[:des_len], xt_y[:des_len], label='Target Y', color='r')
# axs[0, 1].plot(time_stamps[:des_len], xd_y[:des_len], label='Desired Y', color='g',linestyle='--') 
axs[0, 1].plot(time_stamps[:des_len], xee_y[:des_len], label='Measured Y', color='b')
# axs[0, 1].plot(time_stamps[:des_len], xee_y_smoothed[:des_len], label='Smoothed Y', color='r')
# axs[0, 1].scatter(time_stamps[:des_len], xee_y[:des_len], label='Measured Y', color='y')
# axs[0, 1].plot(time_stamps[:des_len], xt_y_raw[:des_len], label='Raw target Y', color='y')
axs[0, 1].grid()
axs[0, 1].set_title('Y values')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y Position [m]')
axs[0, 1].legend()

# Plot Theta values (Target, Desired, Measured)
# axs[1, 0].plot(time_stamps[:des_len], xt_theta[:des_len], label='Target Theta', color='r')
# axs[1, 0].plot(time_stamps[:des_len], xd_theta[:des_len], label='Desired Theta', color='g',linestyle='--')
axs[1, 0].plot(time_stamps[:des_len], xee_theta[:des_len], label='Measured Theta', color='b')
# axs[1, 0].scatter(time_stamps[:des_len], xee_theta[:des_len], label='Measured Theta', color='y')
# axs[1, 0].plot(time_stamps[:des_len], xt_theta_raw[:des_len], label='Raw target Theta', color='y')
axs[1, 0].grid()
axs[1, 0].set_title('Theta values')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Theta [deg]')
axs[1, 0].legend()

# Plot Theta0 values (Target, Desired, Measured)
# axs[1, 1].plot(time_stamps[:des_len], xt_theta0[:des_len], label='Target Theta0', color='r')
# axs[1, 1].plot(time_stamps[:des_len], xd_theta0[:des_len], label='Desired Theta0', color='g',linestyle='--')
axs[1, 1].plot(time_stamps[:des_len], xee_theta0[:des_len], label='Measured Theta0', color='b')
# axs[1, 1].scatter(time_stamps[:des_len], xee_theta0[:des_len], label='Measured Theta0', color='y')
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

# Plot Xdot values (Target, Desired, Measured)
# axs[0, 0].plot(time_stamps[:des_len], xt_x_dot[:des_len], label='Target Xdot', color='r')
# axs[0, 0].plot(time_stamps[:des_len], xd_x_dot[:des_len], label='Desired Xdot', color='g',linestyle='--')
axs[0, 0].plot(time_stamps[:des_len], xee_x_dot[:des_len], label='Measured Xdot', color='b')
# axs[0, 0].scatter(time_stamps[:des_len], xee_x_dot[:des_len], label='Measured Xdot', color='y')
axs[0, 0].grid()
axs[0, 0].set_title('Xdot ')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X Velcotiy [m/sec]')
axs[0, 0].legend()

# Plot Ydot values (Target, Desired, Measured)
# axs[0, 1].plot(time_stamps[:des_len], xt_y_dot[:des_len], label='Target Ydot', color='r')
# axs[0, 1].plot(time_stamps[:des_len], xd_y_dot[:des_len], label='Desired Ydot', color='g',linestyle='--')
axs[0, 1].plot(time_stamps[:des_len], xee_y_dot[:des_len], label='Measured Ydot', color='b')
# axs[0, 1].scatter(time_stamps[:des_len], xee_y_dot[:des_len], label='Measured Ydot', color='y')
axs[0, 1].grid()
axs[0, 1].set_title('Ydot')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y Velocity [m/sec]')
axs[0, 1].legend()

# Plot Thetadot values (Target, Desired, Measured)
# axs[1, 0].plot(time_stamps[:des_len], xt_theta_dot[:des_len], label='Target Thetadot', color='r')
# axs[1, 0].plot(time_stamps[:des_len], xd_theta_dot[:des_len], label='Desired Thetadot', color='g',linestyle='--')
axs[1, 0].plot(time_stamps[:des_len], xee_theta_dot[:des_len], label='Measured Thetadot', color='b')
# axs[1, 0].scatter(time_stamps[:des_len], xee_theta_dot[:des_len], label='Measured Thetadot', color='y')
axs[1, 0].grid()
axs[1, 0].set_title('Thetadot')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Thetadot [deg/sec]')
axs[1, 0].legend()

# Plot Theta0dot values (Target, Desired, Measured)
# axs[1, 1].plot(time_stamps[:des_len], xt_theta0_dot[:des_len], label='Target Theta0dot', color='r')
# axs[1, 1].plot(time_stamps[:des_len], xd_theta0_dot[:des_len], label='Desired Theta0dot', color='g',linestyle='--')
axs[1, 1].plot(time_stamps[:des_len], xee_theta0_dot[:des_len], label='Measured Theta0dot', color='b')
# axs[1, 1].scatter(time_stamps[:des_len], xee_theta0_dot[:des_len], label='Measured Theta0dot', color='y')
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

#OS EDOOOO



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
# plt.plot(time_stamps[:des_len], fext_x[:des_len], label='Fext X', color='k')
plt.plot(time_stamps[:des_len], fext_x_raw[:des_len], label='Fext X ', color='g')
# plt.plot(time_stamps[:des_len], fext_smoothed[:des_len], label='Fext_x')
plt.grid()
plt.title('External Force (X-axis)')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()

# Show all plots
plt.show()



# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# # Plot errorX values (Target, Desired, Measured)
# axs[0, 0].plot(time_stamps[:des_len], error_x[:des_len], label='error X', color='r')
# # axs[0, 0].plot(time_stamps[:des_len], error_xdot[:des_len], label='error Xdot', color='b')
# axs[0, 0].grid()
# axs[0, 0].legend()
# axs[0, 0].set_title("Error X ")
# axs[0, 0].set_xlabel('Time [s]')
# axs[0, 0].set_ylabel('Error [m]')

# # Plot Y values (Target, Desired, Measured)
# axs[0, 1].plot(time_stamps[:des_len], error_y[:des_len], label='error Y', color='r')
# # axs[0, 1].plot(time_stamps[:des_len], error_ydot[:des_len], label='error Ydot', color='b')
# axs[0, 1].grid()
# axs[0, 1].legend()
# axs[0, 1].set_title("Error Y ")
# axs[0, 1].set_xlabel('Time [s]')
# axs[0, 1].set_ylabel('Error [m]')

# # Plot Theta values (Target, Desired, Measured)
# axs[1, 0].plot(time_stamps[:des_len], error_theta[:des_len], label='error Theta', color='r')
# # axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='b')
# axs[1, 0].grid()
# axs[1, 0].legend()
# axs[1, 0].set_title("Error Theta ")
# axs[1, 0].set_xlabel('Time [s]')
# axs[1, 0].set_ylabel('Error [deg]')

# # # Plot Theta0 values (Target, Desired, Measured)
# axs[1, 1].plot(time_stamps[:des_len], error_theta0[:des_len], label='error Theta0', color='r')
# # axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='b')
# axs[1, 1].grid()
# axs[1, 1].legend()
# axs[1, 1].set_title("Error Theta0")
# axs[1, 1].set_xlabel('Time [s]')
# axs[1, 1].set_ylabel('Error [deg]')
# plt.show()


# # Create a figure and a 2x2 grid of subplots
# fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# # Plot errorX values (Target, Desired, Measured)
# axs[0, 0].plot(time_stamps[:des_len], error_xdot[:des_len], label='error Xdot', color='r')
# axs[0, 0].grid()
# axs[0, 0].legend()
# axs[0, 0].set_title("Error Xdot ")
# axs[0, 0].set_xlabel('Time [s]')
# axs[0, 0].set_ylabel('Error [m/sec]')

# # Plot Y values (Target, Desired, Measured)
# axs[0, 1].plot(time_stamps[:des_len], error_ydot[:des_len], label='error Ydot', color='r')
# axs[0, 1].grid()
# axs[0, 1].legend()
# axs[0, 1].set_title("Error Ydot")
# axs[0, 1].set_xlabel('Time [s]')
# axs[0, 1].set_ylabel('Error [m/sec]')

# # Plot Theta values (Target, Desired, Measured)
# axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='r')
# axs[1, 0].grid()
# axs[1, 0].legend()
# axs[1, 0].set_title("Error Thetadot ")
# axs[1, 0].set_xlabel('Time [s]')
# axs[1, 0].set_ylabel('Error [deg/sec]')

# # # Plot Theta0 values (Target, Desired, Measured)
# axs[1, 1].plot(time_stamps[:des_len], error_theta0dot[:des_len], label='error Theta0dot', color='r')
# axs[1, 1].grid()
# axs[1, 1].legend()
# axs[1, 1].set_title("Error Theta0dot")
# axs[1, 1].set_xlabel('Time [s]')
# axs[1, 1].set_ylabel('Error [deg/sec]')
# plt.show()


# ydot = xee_y_dot
# fs = 100
# n = len(ydot)
# frequencies = np.fft.fftfreq(n, d=1/fs)
# fft_magnitude = np.abs(np.fft.fft(ydot))

# # Plot FFT
# plt.figure(figsize=(8,4))
# plt.plot(frequencies[:n//2], fft_magnitude[:n//2])  # Only positive frequencies
# plt.xlabel("Frequency (Hz)")
# plt.ylabel("Magnitude")
# plt.title("FFT of ydot signal")
# plt.grid()
# plt.show()

# def butter_lowpass_filter(data, cutoff, fs, order=2):
#     nyquist = 0.5 * fs  # Nyquist frequency
#     normal_cutoff = cutoff / nyquist  # Normalize cutoff frequency
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return filtfilt(b, a, data)  # Apply filter

# filtered_ydot = butter_lowpass_filter(ydot, cutoff=0.8, fs=100, order=2)

# plt.plot(time_stamps[:des_len], filtered_ydot[:des_len], label='ydot', color='r')
# plt.plot(time_stamps[:des_len], xd_y_dot[:des_len], label='Desired Ydot', color='g',linestyle='--')
# plt.title("Butterworth 2nd order filter with fc = 0.8Hz")
# plt.xlabel("Time (sec) ")
# plt.ylabel("Ydot (m/s)")
# plt.grid()
# plt.show()


# y = xee_y
# fs = 100
# n = len(y)
# frequencies = np.fft.fftfreq(n, d=1/fs)
# fft_magnitude = np.abs(np.fft.fft(y))

# # Plot FFT
# plt.figure(figsize=(8,4))
# plt.plot(frequencies[:n//2], fft_magnitude[:n//2])  # Only positive frequencies
# plt.xlabel("Frequency (Hz)")
# plt.ylabel("Magnitude")
# plt.title("FFT of y signal")
# plt.grid()
# plt.show()

# filtered_y = butter_lowpass_filter(ydot, cutoff=5, fs=100, order=2)

# plt.plot(time_stamps[:des_len], filtered_y[:des_len], label='y', color='r')
# plt.plot(time_stamps[:des_len], xd_y[:des_len], label='Desired Ydot', color='g',linestyle='--')
# plt.title("Butterworth 2nd order filter with fc = 5Hz")
# plt.xlabel("Time (sec) ")
# plt.ylabel("Y (m) ")
# plt.grid()
# plt.show()


# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot errorX values (Target, Desired, Measured)
axs[0, 0].plot(time_stamps[:des_len], acc1_x[:des_len], label='acc1 X', color='r')
# axs[0, 0].plot(time_stamps[:des_len], error_xdot[:des_len], label='error Xdot', color='b')
axs[0, 0].grid()
axs[0, 0].legend()
axs[0, 0].set_title("acc1 X ")
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('acceleration [m/s^2]')

# Plot Y values (Target, Desired, Measured)
axs[0, 1].plot(time_stamps[:des_len], acc1_y[:des_len], label='acc1  Y', color='r')
# axs[0, 1].plot(time_stamps[:des_len], error_ydot[:des_len], label='error Ydot', color='b')
axs[0, 1].grid()
axs[0, 1].legend()
axs[0, 1].set_title("acc1 Y ")
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('acceleration [m/s^2]')

# Plot Theta values (Target, Desired, Measured)
axs[1, 0].plot(time_stamps[:des_len], acc1_z[:des_len], label='acc1 Z', color='r')
# axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='b')
axs[1, 0].grid()
axs[1, 0].legend()
axs[1, 0].set_title("acc1 Z ")
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('acceleration [m/s^2]')

plt.show()

# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot errorX values (Target, Desired, Measured)
axs[0, 0].plot(time_stamps[:des_len], acc2_x[:des_len], label='acc2 X', color='r')
# axs[0, 0].plot(time_stamps[:des_len], error_xdot[:des_len], label='error Xdot', color='b')
axs[0, 0].grid()
axs[0, 0].legend()
axs[0, 0].set_title("acc2 X ")
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('acceleration [m/s^2]')

# Plot Y values (Target, Desired, Measured)
axs[0, 1].plot(time_stamps[:des_len], acc2_y[:des_len], label='acc2  Y', color='r')
# axs[0, 1].plot(time_stamps[:des_len], error_ydot[:des_len], label='error Ydot', color='b')
axs[0, 1].grid()
axs[0, 1].legend()
axs[0, 1].set_title("acc2 Y ")
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('acceleration [m/s^2]')

# Plot Theta values (Target, Desired, Measured)
axs[1, 0].plot(time_stamps[:des_len], acc2_z[:des_len], label='acc2 Z', color='r')
# axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='b')
axs[1, 0].grid()
axs[1, 0].legend()
axs[1, 0].set_title("acc2 Z ")
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('acceleration [m/s^2]')

plt.show()

# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot errorX values (Target, Desired, Measured)
axs[0, 0].plot(time_stamps[:des_len], acc3_x[:des_len], label='acc3 X', color='r')
# axs[0, 0].plot(time_stamps[:des_len], error_xdot[:des_len], label='error Xdot', color='b')
axs[0, 0].grid()
axs[0, 0].legend()
axs[0, 0].set_title("acc3 X ")
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('acceleration [m/s^2]')

# Plot Y values (Target, Desired, Measured)
axs[0, 1].plot(time_stamps[:des_len], acc3_y[:des_len], label='acc3  Y', color='r')
# axs[0, 1].plot(time_stamps[:des_len], error_ydot[:des_len], label='error Ydot', color='b')
axs[0, 1].grid()
axs[0, 1].legend()
axs[0, 1].set_title("acc3 Y ")
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('acceleration [m/s^2]')

# Plot Theta values (Target, Desired, Measured)
axs[1, 0].plot(time_stamps[:des_len], acc3_z[:des_len], label='acc3 Z', color='r')
# axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='b')
axs[1, 0].grid()
axs[1, 0].legend()
axs[1, 0].set_title("acc3 Z ")
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('acceleration [m/s^2]')

plt.show()

# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot errorX values (Target, Desired, Measured)
axs[0, 0].plot(time_stamps[:des_len], imu_acc_x[:des_len], label='imu acc X', color='r')
# axs[0, 0].plot(time_stamps[:des_len], error_xdot[:des_len], label='error Xdot', color='b')
axs[0, 0].grid()
axs[0, 0].legend()
axs[0, 0].set_title("IMU acc X")
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('Acceleration [m]')

# Plot Y values (Target, Desired, Measured)
axs[0, 1].plot(time_stamps[:des_len], imu_acc_y[:des_len], label='imu acc  Y', color='r')
# axs[0, 1].plot(time_stamps[:des_len], error_ydot[:des_len], label='error Ydot', color='b')
axs[0, 1].grid()
axs[0, 1].legend()
axs[0, 1].set_title("IMU acc Y ")
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Acceleration [m/s^2]')

# Plot Theta values (Target, Desired, Measured)
axs[1, 0].plot(time_stamps[:des_len], imu_vel_theta[:des_len], label='imu vel theta ', color='r')
# axs[1, 0].plot(time_stamps[:des_len], error_thetadot[:des_len], label='error Thetadot', color='b')
axs[1, 0].grid()
axs[1, 0].legend()
axs[1, 0].set_title("IMU vel Theta ")
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Velocity [rad/sec]')

plt.show()

# Assume all the lists below are already populated somewhere in your code
# time_stamps, xt_x, xd_x, xee_x, ..., imu_acc_x, imu_acc_y, imu_vel_theta

data_dict = {
    "time": time_stamps,

    "xd_x": xd_x, "xee_x": xee_x,
    "xd_y": xd_y, "xee_y": xee_y,
    "xd_theta": xd_theta, "xee_theta": xee_theta,
    "xd_theta0": xd_theta0, "xee_theta0": xee_theta0,

    "xd_x_dot": xd_x_dot, "xee_x_dot": xee_x_dot,
    "xd_y_dot": xd_y_dot, "xee_y_dot": xee_y_dot,
    "xd_theta_dot": xd_theta_dot, "xee_theta_dot": xee_theta_dot,
    "xd_theta0_dot": xd_theta0_dot, "xee_theta0_dot": xee_theta0_dot,

    "fext_x_raw": fext_x_raw,

    "torquerw": torquerw,
    "torqueq1": torqueq1,
    "torqueq2": torqueq2,
    "torqueq3": torqueq3,

    "error_x": error_x, "error_xdot": error_xdot,
    "error_y": error_y, "error_ydot": error_ydot,
    "error_theta": error_theta, "error_thetadot": error_thetadot,
    "error_theta0": error_theta0, "error_theta0dot": error_theta0dot,

    "acc1_x": acc1_x, "acc1_y": acc1_y, "acc1_z": acc1_z,
    "acc2_x": acc2_x, "acc2_y": acc2_y, "acc2_z": acc2_z,
    "acc3_x": acc3_x, "acc3_y": acc3_y, "acc3_z": acc3_z,

    "imu_acc_x": imu_acc_x, "imu_acc_y": imu_acc_y, "imu_vel_theta": imu_vel_theta
}

# Convert all lists to numpy arrays to ensure compatibility with MATLAB
for key in data_dict:
    data_dict[key] = np.array(data_dict[key])

# Ask user for filename
filename = input("Enter the filename to save (without .mat extension): ").strip() + ".mat"
file_path = os.path.join(path2, filename)

# Save to .mat file
scipy.io.savemat(file_path, data_dict)
print(f"Data saved to {filename}")