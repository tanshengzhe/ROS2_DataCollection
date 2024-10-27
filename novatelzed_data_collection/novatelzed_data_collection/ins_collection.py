
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv
from geometry_msgs.msg import Pose

from novatel_gps_msgs.msg import Inspvax
from gps_msgs.msg  import GPSFix
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv
import os

import numpy as np

lat = []
long = []
vel = []
north_vel = []
east_vel = []
roll = []
pitch = []
azimuth = []
timestamp = []
accx = []
accy=[]
accz = []
x = []
y = []
z = []
track = []
altitude = []
timestamp_Y = []
timestamp_m = []
timestamp_d = []
timestamp_H = []
timestamp_M = []
timestamp_S = []


class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Inspvax,
            'inspvax',  # Replace with your topic name
            self.path_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Create a CSV file and write the header

    def path_callback(self, ins_msg):

        sec = ins_msg.header.stamp.sec
        nanosec = ins_msg.header.stamp.nanosec

        time = sec + nanosec * 1e-9
        now = datetime.fromtimestamp(time)
        time_Y = now.strftime("%Y")
        time_m = now.strftime("%m")
        time_d = now.strftime("%d")
        time_H = now.strftime("%H")
        time_M = now.strftime("%M")
        time_S = now.strftime("%S.%f")

        timestamp_Y.append(float(time_Y))
        timestamp_m.append(float(time_m))
        timestamp_d.append(float(time_d))
        timestamp_H.append(float(time_H))
        timestamp_M.append(float(time_M))
        timestamp_S.append(float(time_S))
        

        lat.append(float(ins_msg.latitude))
        long.append(float(ins_msg.longitude))
        altitude.append(float(ins_msg.altitude))
        # track.append(float(gps_msg.track))
        # alt.append(float(imu_msg.altitude))

        # accx.append(float(imuraw_msg.linear_acceleration.x))
        # accy.append(float(imuraw_msg.linear_acceleration.y))
        # accz.append(float(imuraw_msg.linear_acceleration.z))

        north_vel.append(float(ins_msg.north_velocity))
        east_vel.append(float(ins_msg.east_velocity))
        roll.append(float(ins_msg.roll))
        pitch.append(float(ins_msg.pitch))
        azimuth.append(float(ins_msg.azimuth))

        
        gps_location = np.array((lat, long, north_vel, east_vel, roll, pitch, azimuth,timestamp_Y,timestamp_m, timestamp_d, timestamp_H, timestamp_M, timestamp_S))
        self.get_logger().info(f'Received ins data latitude: {ins_msg.latitude}')
        # gps_location = np.array((lat, long, altitude, track))


        np.savetxt("/home/autodrive/Music/GPS_IMU_Data_Collection_ws/data_output_csv/ins_OCT18_Route1.csv", gps_location.T, delimiter=",")
        

def main(args=None):
    rclpy.init(args=args)
    path_subscriber = PathSubscriber()

    rclpy.spin(path_subscriber)

    # Cleanup
    path_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()