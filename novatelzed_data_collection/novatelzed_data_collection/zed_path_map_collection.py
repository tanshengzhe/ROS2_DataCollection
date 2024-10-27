

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv

class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Path,
            'zed/zed_node/path_map',  # Replace with your topic name
            self.path_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Create a CSV file and write the header
        self.csv_file_path = 'unique_path_data_oct8_720_Route3.csv'
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Index', 'Position X', 'Position Y', 'Position Z'])

        # Set to keep track of unique positions
        self.seen_positions = set()

    def path_callback(self, msg):
        self.get_logger().info('Received Path message:')
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            for i, pose in enumerate(msg.poses):
                position = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                if position not in self.seen_positions:
                    self.seen_positions.add(position)
                    self.get_logger().info(f'Pose {i}: Position (x={position[0]}, y={position[1]}, z={position[2]})')
                    # Write data to the CSV file
                    writer.writerow([len(self.seen_positions) - 1, position[0], position[1], position[2]])

def main(args=None):
    rclpy.init(args=args)
    path_subscriber = PathSubscriber()

    rclpy.spin(path_subscriber)

    # Cleanup
    path_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
