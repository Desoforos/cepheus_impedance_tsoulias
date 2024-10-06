#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

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
def update_plot(i):
    plt.clf()

    # Plot X Coordinates
    plt.subplot(2, 2, 1)
    plt.plot(time_stamps[:len(xt_x)], xt_x, label='Target X', color='r')
    plt.plot(time_stamps[:len(xd_x)], xd_x, label='Desired X', color='g')
    plt.plot(time_stamps[:len(xee_x)], xee_x, label='Actual X', color='b')
    plt.grid()
    plt.title('X Coordinates')
    plt.xlabel('Time [s]')
    plt.ylabel('X Position [m]')
    plt.legend()

    # Plot Y Coordinates
    plt.subplot(2, 2, 2)
    plt.plot(time_stamps[:len(xt_y)], xt_y, label='Target Y', color='r')
    plt.plot(time_stamps[:len(xd_y)], xd_y, label='Desired Y', color='g')
    plt.plot(time_stamps[:len(xee_y)], xee_y, label='Actual Y', color='b')
    plt.grid()
    plt.title('Y Coordinates')
    plt.xlabel('Time [s]')
    plt.ylabel('Y Position [m]')
    plt.legend()

    # Plot Theta Coordinates
    plt.subplot(2, 2, 3)
    plt.plot(time_stamps[:len(xt_theta)], xt_theta, label='Target Theta', color='r')
    plt.plot(time_stamps[:len(xd_theta)], xd_theta, label='Desired Theta', color='g')
    plt.plot(time_stamps[:len(xee_theta)], xee_theta, label='Actual Theta', color='b')
    plt.grid()
    plt.title('Theta Coordinates')
    plt.xlabel('Time [s]')
    plt.ylabel('Theta [rad]')
    plt.legend()

    # Plot External Force (X-axis)
    plt.subplot(2, 2, 4)
    plt.plot(time_stamps[:len(fext_x)], fext_x, label='Fext X', color='k')
    plt.grid()
    plt.title('External Force (X-axis)')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.legend()

# Main function
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

    rospy.Subscriber('/cepheus/ft_sensor_topic', Float64, fext_x_callback)

    # Set up the plot
    plt.figure(figsize=(10, 8))

    # Use FuncAnimation to update the plot every 10ms
    ani = FuncAnimation(plt.gcf(), update_plot, interval=10)

    # Start the ROS event loop
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    try:
        real_time_plotter()
    except rospy.ROSInterruptException:
        pass
