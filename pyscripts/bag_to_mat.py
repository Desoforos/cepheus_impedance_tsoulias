import argparse
import rosbag
import numpy as np
from scipy.io import savemat
# afto to arxeio ousiastika pairnei to rosbag kai to metatrepei se .mat
def main():
    path2 = "/home/desoforos/Desktop/IOANT_proti_fash/"
    parser = argparse.ArgumentParser(description='Convert ROS bag to .mat with specific signals')
    parser.add_argument('bagfile', type=str, help='Path to the ROS bag file')
    args = parser.parse_args()

    bag = rosbag.Bag(args.bagfile)

    # Initialize all desired variables
    acc_x, acc_y, acc_z = [], [], []
    vel_x, vel_y, vel_z = [], [], []
    force_x, force_y, force_z = [], [], []
    torque_x, torque_y, torque_z = [], [], []
    marker_x, marker_y, marker_z = [], [], []

    q1, q1dot = [], []
    q2, q2dot = [], []
    q3, q3dot = [], []
    sampling_frequency = 100  # Hz
    time_stamps = []

    # timeMarker =[]
    # timeWrench =[]
    # timeAcc = []
    # timeVel = []
    # timeq1 =[]
    # timeq1dot =[]
    # timeq2 =[]
    # timeq2dot = []
    # timeq3 = []
    # timeq3dot =[]

    exptime = 252 #252 seconds


    # cutoff_index = 12 * 100
    # fextx = fextx[cutoff_index:]


    for topic, msg, t in bag.read_messages(topics=['/imu/acceleration','/imu/angular_velocity','/bus0/ft_sensor0/ft_sensor_readings/wrench',
                                                    '/vicon/marker_x','/vicon/marker_y','/vicon/marker_z', '/read_left_elbow_position', '/read_left_elbow_velocity',
                                                    '/read_left_shoulder_position', '/read_left_shoulder_velocity', '/read_right_elbow_position', 
                                                    '/read_right_elbow_velocity', '/vicon/markers' ]):
        time_stamps.append(t.to_sec())

        if topic == '/imu/acceleration':
            acc_x.append(msg.vector.x)
            acc_y.append(msg.vector.y)
            acc_z.append(msg.vector.z)
        elif topic == '/imu/angular_velocity':
            vel_x.append(msg.vector.x)
            vel_y.append(msg.vector.y)
            vel_z.append(msg.vector.z)
        elif topic == '/bus0/ft_sensor0/ft_sensor_readings/wrench':
            force_x.append(msg.wrench.force.x)
            force_y.append(msg.wrench.force.y)
            force_z.append(msg.wrench.force.z)
            torque_x.append(msg.wrench.torque.x)
            torque_y.append(msg.wrench.torque.y)
            torque_z.append(msg.wrench.torque.z)
        # elif topic == '/vicon/marker_x':
        #     marker_x.append(msg.data)
        # elif topic == '/vicon/marker_y':
        #     marker_y.append(msg.data)
        # elif topic == '/vicon/marker_z':
        #     marker_z.append(msg.data)
        elif topic == '/vicon/markers' and msg.markers:
            # Save first marker's position
            marker_x.append(msg.markers[0].translation.x)
            marker_y.append(msg.markers[0].translation.y)
            marker_z.append(msg.markers[0].translation.z)
        elif topic == '/read_left_elbow_position':
            q1.append(msg.data)
        elif topic == '/read_left_elbow_velocity':
            q1dot.append(msg.data)
        elif topic == '/read_left_shoulder_position':
            q2.append(msg.data)
        elif topic == '/read_left_shoulder_velocity':
            q2dot.append(msg.data)
        elif topic == '/read_right_elbow_position':
            q3.append(msg.data)
        elif topic == '/read_right_elbow_velocity':
            q3dot.append(msg.data)

    bag.close()

    time_stamps = np.arange(0, len(q1)) / sampling_frequency
    time_stamps -= time_stamps[0]  # Start from 0

    #ta parakato einai listes xronou gia kathe megethos dioti otan ekana metriseis
    # tis ekana me to command line rosbag record, opote ekane record se diaforetikes sixnotites
    # kalytera na exeis ena node pou na ta kanei ola record stin idia sixnotita 
    timeMarker = np.arange(0,len(marker_x)) / (len(marker_x)/exptime)
    timeWrench = np.arange(0,len(force_x)) / (len(force_x)/exptime)
    timeAcc = np.arange(0,len(acc_x)) / (len(acc_x)/exptime)
    timeVel = np.arange(0,len(vel_x)) / (len(vel_x)/exptime)
    timeq1 = np.arange(0,len(q1)) / (len(q1)/exptime)
    timeq1dot = np.arange(0,len(q1dot)) / (len(force_x)/exptime)
    timeq2 = np.arange(0,len(q2)) / (len(q2)/exptime)
    timeq2dot = np.arange(0,len(q2dot)) / (len(q2dot)/exptime)
    timeq3 = np.arange(0,len(q3)) / (len(q3)/exptime)
    timeq3dot = np.arange(0,len(q3dot)) / (len(q3dot)/exptime)

    print("wrench frequency is", len(force_x)/exptime, " Hz.")
    print("Vicon frequency is", len(marker_x)/exptime, " Hz.")
    print("Acc frequency is", len(acc_x)/exptime, " Hz.")
    print("Vel frequency is", len(vel_x)/exptime, " Hz.")
    print("q1 frequency is", len(q1)/exptime, " Hz.")
    print("q1dot frequency is", len(q1dot)/exptime, " Hz.")
    print("q2 frequency is", len(q2)/exptime, " Hz.")
    print("q2dot frequency is", len(q2dot)/exptime, " Hz.")
    print("q3 frequency is", len(q3)/exptime, " Hz.")
    print("q3dot frequency is", len(q3dot)/exptime, " Hz.")
    # Ask for output filename
    mat_filename = path2 + input("Enter name of the output .mat file (without extension): ").strip() + ".mat"

    # Prepare dictionary with exact variable names
    mat_data = {
        'acc_x': np.array(acc_x),
        'acc_y': np.array(acc_y),
        'acc_z': np.array(acc_z),
        'vel_x': np.array(vel_x),
        'vel_y': np.array(vel_y),
        'vel_z': np.array(vel_z),
        'force_x': np.array(force_x),
        'force_y': np.array(force_y),
        'force_z': np.array(force_z),
        'torque_x': np.array(torque_x),
        'torque_y': np.array(torque_y),
        'torque_z': np.array(torque_z),
        'marker_x': np.array(marker_x),
        'marker_y': np.array(marker_y),
        'marker_z': np.array(marker_z),
        'q1': np.array(q1),
        'q1dot': np.array(q1dot),
        'q2': np.array(q2),
        'q2dot': np.array(q2dot),
        'q3': np.array(q3),
        'q3dot': np.array(q3dot),
        'timeVicon': np.array(timeMarker),
        'timeAcc': np.array(timeAcc),
        'timeVel': np.array(timeVel),
        'timeWrench': np.array(timeWrench),
        'timeq1': np.array(timeq1),
        'timeq1dot': np.array(timeq1dot),
        'timeq2': np.array(timeq2),
        'timeq2dot': np.array(timeq2dot),
        'timeq3': np.array(timeq3),
        'timeq3dot': np.array(timeq3dot),
        
    }

    savemat(mat_filename, mat_data)
    print(f"Saved to {mat_filename}")

if __name__ == '__main__':
    main()


        # elif topic == '/vicon/markers' and msg.markers:
        #     # Save first marker's position
        #     marker_x.append(msg.markers[0].translation.x)
        #     marker_y.append(msg.markers[0].translation.y)
        #     marker_z.append(msg.markers[0].translation.z)