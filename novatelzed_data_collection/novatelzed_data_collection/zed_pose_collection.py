import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv
from geometry_msgs.msg import PoseStamped


from gps_msgs.msg  import GPSFix

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv
import os
import datetime

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
            PoseStamped,
            '/zed/zed_node/pose',  # Replace with your topic name
            self.path_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Create a CSV file and write the header

    def path_callback(self, pose_msg):
        print(pose_msg.pose.position.x)


        sec = pose_msg.header.stamp.sec
        nanosec = pose_msg.header.stamp.nanosec

        time = sec + nanosec * 1e-9
        now = datetime.datetime.fromtimestamp(time)
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



        x.append(float(pose_msg.pose.position.x))
        y.append(float(pose_msg.pose.position.y))
        z.append(float(pose_msg.pose.position.z))
        # track.append(float(gps_msg.track))
        # alt.append(float(imu_msg.altitude))

        # accx.append(float(imuraw_msg.linear_acceleration.x))
        # accy.append(float(imuraw_msg.linear_acceleration.y))
        # accz.append(float(imuraw_msg.linear_acceleration.z))

        # north_vel.append(float(imu_msg.north_velocity))
        # east_vel.append(float(imu_msg.east_velocity))
        # roll.append(float(imu_msg.roll))
        # pitch.append(float(imu_msg.pitch))
        # azimuth.append(float(imu_msg.azimuth))

        
        # gps_location = np.array((lat, long, vel, north_vel, east_vel, roll, pitch, azimuth,timestamp))

        gps_location = np.array((x,y,z,timestamp_Y,timestamp_m,timestamp_d,timestamp_H,timestamp_M,timestamp_S))


        np.savetxt("/home/autodrive/Music/single_topic_sub/zed_slam_pose.csv", gps_location.T, delimiter=",")
        

def main(args=None):
    rclpy.init(args=args)
    path_subscriber = PathSubscriber()

    rclpy.spin(path_subscriber)

    # Cleanup
    path_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
