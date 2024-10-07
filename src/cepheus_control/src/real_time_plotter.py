#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import WrenchStamped

# Initialize global variables to store the data
xt_x, xd_x, xee_x = [], [], []
xt_y, xd_y, xee_y = [], [], []
xt_theta, xd_theta, xee_theta = [], [], []
fext_x = []
time_stamps = []

# Callback functions for each topic
def xt_x_callback(msg):
    xt_x.append(msg.data)
    time_stamps.append(rospy.get_time())



def xd_x_callback(msg):
    xd_x.append(msg.data)

def xee_x_callback(msg):
    xee_x.append(msg.data)

def xt_y_callback(msg):
    xt_y.append(msg.data)

def xd_y_callback(msg):
    xd_y.append(msg.data)

def xee_y_callback(msg):
    xee_y.append(msg.data)

def xt_theta_callback(msg):
    xt_theta.append(msg.data)

def xd_theta_callback(msg):
    xd_theta.append(msg.data)

def xee_theta_callback(msg):
    xee_theta.append(msg.data)

def fext_x_callback(msg):
    fext_x.append(msg.wrench.force.x)

# Function to update the plot in real time
# def update_plot(i):
#     plt.clf()

#     # Plot X Coordinates
#     plt.subplot(2, 2, 1)
#     plt.plot(time_stamps[:len(xt_x)], xt_x, label='Target X', color='r')
#     plt.plot(time_stamps[:len(xd_x)], xd_x, label='Desired X', color='g')
#     plt.plot(time_stamps[:len(xee_x)], xee_x, label='Actual X', color='b')
#     plt.grid()
#     plt.title('X Coordinates')
#     plt.xlabel('Time [s]')
#     plt.ylabel('X Position [m]')
#     plt.legend()

#     # Plot Y Coordinates
#     plt.subplot(2, 2, 2)
#     plt.plot(time_stamps[:len(xt_y)], xt_y, label='Target Y', color='r')
#     plt.plot(time_stamps[:len(xd_y)], xd_y, label='Desired Y', color='g')
#     plt.plot(time_stamps[:len(xee_y)], xee_y, label='Actual Y', color='b')
#     plt.grid()
#     plt.title('Y Coordinates')
#     plt.xlabel('Time [s]')
#     plt.ylabel('Y Position [m]')
#     plt.legend()

#     # Plot Theta Coordinates
#     plt.subplot(2, 2, 3)
#     plt.plot(time_stamps[:len(xt_theta)], xt_theta, label='Target Theta', color='r')
#     plt.plot(time_stamps[:len(xd_theta)], xd_theta, label='Desired Theta', color='g')
#     plt.plot(time_stamps[:len(xee_theta)], xee_theta, label='Actual Theta', color='b')
#     plt.grid()
#     plt.title('Theta Coordinates')
#     plt.xlabel('Time [s]')
#     plt.ylabel('Theta [rad]')
#     plt.legend()

#     # Plot External Force (X-axis)
#     plt.subplot(2, 2, 4)
#     plt.plot(time_stamps[:len(fext_x)], fext_x, label='Fext X', color='k')
#     plt.grid()
#     plt.title('External Force (X-axis)')
#     plt.xlabel('Time [s]')
#     plt.ylabel('Force [N]')
#     plt.legend()

#     plt.pause(0.01)

# # Main function
# def real_time_plotter():
#     rospy.init_node('real_time_plotter', anonymous=True)

#     # Subscribers for the relevant topics
#     rospy.Subscriber('/cepheus/xt_x', Float64, xt_x_callback)
#     rospy.Subscriber('/cepheus/xd_x', Float64, xd_x_callback)
#     rospy.Subscriber('/cepheus/xee_x', Float64, xee_x_callback)

#     rospy.Subscriber('/cepheus/xt_y', Float64, xt_y_callback)
#     rospy.Subscriber('/cepheus/xd_y', Float64, xd_y_callback)
#     rospy.Subscriber('/cepheus/xee_y', Float64, xee_y_callback)

#     rospy.Subscriber('/cepheus/xt_theta', Float64, xt_theta_callback)
#     rospy.Subscriber('/cepheus/xd_theta', Float64, xd_theta_callback)
#     rospy.Subscriber('/cepheus/xee_theta', Float64, xee_theta_callback)

#     rospy.Subscriber('/cepheus/ft_sensor_topic', Float64, fext_x_callback) #mallon thelei WrenchStamped

#     # Set up the plot
#     plt.figure(figsize=(10, 8))

#     # Use FuncAnimation to update the plot every 10ms
#     ani = FuncAnimation(plt.gcf(), update_plot, interval=10)

#     # Start the ROS event loop
#     plt.show()
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         real_time_plotter()
#     except rospy.ROSInterruptException:
#         pass

def plot_data():
    plt.ion()  # Enable interactive mode
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))  # Create 2x2 grid of subplots

    while not rospy.is_shutdown():
        if time_stamps:
            # Clear previous plots
            axs[0, 0].cla()
            axs[0, 1].cla()
            axs[1, 0].cla()
            axs[1, 1].cla()

            # X Coordinates (Target, Desired, Actual)
            axs[0, 0].plot(time_stamps[:len(xt_x)], xt_x, label='Target X', color='r')
            axs[0, 0].plot(time_stamps[:len(xt_x)], xd_x, label='Desired X', color='g')
            axs[0, 0].plot(time_stamps[:len(xt_x)], xee_x, label='Actual X', color='b')
            axs[0, 0].set_title('X Coordinates')
            axs[0, 0].set_xlabel('Time [s]')
            axs[0, 0].set_ylabel('X Position [m]')
            axs[0, 0].legend()
            axs[0, 0].grid()

            # Y Coordinates (Target, Desired, Actual)
            axs[0, 1].plot(time_stamps[:len(xt_x)], xt_y, label='Target Y', color='r')
            axs[0, 1].plot(time_stamps[:len(xt_x)], xd_y, label='Desired Y', color='g')
            axs[0, 1].plot(time_stamps[:len(xt_x)], xee_y, label='Actual Y', color='b')
            axs[0, 1].set_title('Y Coordinates')
            axs[0, 1].set_xlabel('Time [s]')
            axs[0, 1].set_ylabel('Y Position [m]')
            axs[0, 1].legend()
            axs[0, 1].grid()

            # Theta Coordinates (Target, Desired, Actual)
            axs[1, 0].plot(time_stamps[:len(xt_x)], xt_theta, label='Target Theta', color='r')
            axs[1, 0].plot(time_stamps[:len(xt_x)], xd_theta, label='Desired Theta', color='g')
            axs[1, 0].plot(time_stamps[:len(xt_x)], xee_theta, label='Actual Theta', color='b')
            axs[1, 0].set_title('Theta Coordinates')
            axs[1, 0].set_xlabel('Time [s]')
            axs[1, 0].set_ylabel('Theta [rad]')
            axs[1, 0].legend()
            axs[1, 0].grid()

            # External Force (X-axis)
            axs[1, 1].plot(time_stamps[:len(xt_x)], fext_x, label='Fext X', color='k')
            axs[1, 1].set_title('External Force (X-axis)')
            axs[1, 1].set_xlabel('Time [s]')
            axs[1, 1].set_ylabel('Force [N]')
            axs[1, 1].legend()
            axs[1, 1].grid()

            # Redraw the plot
            plt.pause(0.01)
    
    # plt.ioff()
    plt.show()

def real_time_plotter():
    rospy.init_node('real_time_plotter', anonymous=True)
    # Subscribers for the relevant topics
    rospy.Subscriber('/cepheus/xt_x', Float64, xt_x_callback)
    rospy.Subscriber('/cepheus/xd_x', Float64, xd_x_callback)
    rospy.Subscriber('/cepheus/xee_x', Float64, xee_x_callback)

    rospy.Subscriber('/cepheus/xt_y', Float64, xt_y_callback)
    rospy.Subscriber('/cepheus/xd_y', Float64, xd_y_callback)
    rospy.Subscriber('/cepheus/xee_y', Float64, xee_y_callback)

    rospy.Subscriber('/cepheus/xt_theta', Float64, xt_theta_callback)
    rospy.Subscriber('/cepheus/xd_theta', Float64, xd_theta_callback)
    rospy.Subscriber('/cepheus/xee_theta', Float64, xee_theta_callback)

    rospy.Subscriber('/cepheus/ft_sensor_topic', WrenchStamped, fext_x_callback) #mallon thelei WrenchStamped


    plt.ion()  # Interactive mode to allow real-time updating
    plt.show()
    rate = rospy.Rate(10)
    plot_data()  # Call the plotting function
    rospy.spin()
    rate.sleep()



if __name__ == '__main__':
    try:
        real_time_plotter()
    except rospy.ROSInterruptException:
        pass