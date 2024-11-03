#!/usr/bin/env python3

import rospy
import csv
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion
import rospkg
import os
import math

# Defining trajectory parameters
A  = 5          # Radius [m]
Omega = 0.5      # Angular velcoity [rad/s]
K1 = 1.0        # Proportional gain for velocity
K2 = 1.0        # Proportional gain for steering angle

########################
class DataLogger:
    def __init__(self):
        # Get the package path
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('sc649_assignment_3')

        # Initialize the ROS node
        rospy.init_node('datalogger', anonymous=True)

        # Open a CSV file for logging
        self.file_name = os.path.join(package_path, 'logs/data_log.csv')
        self.csv_file = open(self.file_name, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write the header row to the CSV file
        self.csv_writer.writerow(['Time', 'PositionX', 'PositionY', 'Orientation', 'Speed', 'SteeringAngle', 
                                  'XError','YError','HeadingError'])

        # Subscribe to the relevant topics
        rospy.Subscriber('/pf/pose/odom', Odometry, self.pose_callback)
        rospy.Subscriber('/racecar/ackermann_cmd_mux/output', AckermannDriveStamped, self.command_callback)

        # Initialize variables to store the latest odometry and control data
        self.latest_pose = None
        self.latest_command = None

        # Set the rate of logging
        self.rate = rospy.Rate(10)  # 10 Hz

    def pose_callback(self,data):
        self.latest_pose = [data.pose.pose.position.x,
                data.pose.pose.position.y,
                euler_from_quaternion([data.pose.pose.orientation.x,
                                    data.pose.pose.orientation.y,
                                    data.pose.pose.orientation.z,
                                    data.pose.pose.orientation.w])[2]]
    def command_callback(self, data):
        # Update the latest control command data
        self.latest_command = data

    def log_data(self):
        while not rospy.is_shutdown():
            # Check if we have the latest odometry and command data
            if self.latest_pose and self.latest_command:
                # Get current time
                current_time = rospy.get_time()

                # Extract odometry information
                position_x = self.latest_pose[0]
                position_y = self.latest_pose[1]
                orientation = self.latest_pose[2]

                # Extract control command information
                speed = self.latest_command.drive.speed
                steering_angle = self.latest_command.drive.steering_angle

                #### CODE FROM CONTROLLER PORTED HERE ####
                t = current_time
                pose = self.latest_pose
                x_r = A*math.cos(Omega*t)
                y_r = A*math.sin(Omega*t)

                # Position Error
                x_error = x_r - pose[0]
                y_error = y_r - pose[1]

                # Calculate the angle to the target point
                angle_to_target = math.atan2(y_error, x_error)
                heading_error = angle_to_target - pose[2]

                # Normalize the heading error to be within [-pi, pi]
                if heading_error > math.pi:
                    heading_error -= 2 * math.pi
                elif heading_error < -math.pi:
                    heading_error += 2 * math.pi

                #### CODE FROM CONTROLLER PORTED HERE ENDS####

                # Write the data to the CSV file
                self.csv_writer.writerow([current_time, position_x, position_y, orientation, speed, steering_angle,
                                          x_error,y_error,heading_error])
                self.csv_file.flush()  # Ensure data is written to the file

            self.rate.sleep()

    def cleanup(self):
        # Close the CSV file when shutting down
        self.csv_file.close()

if __name__ == '__main__':
    try:
        logger = DataLogger()
        logger.log_data()
    except rospy.ROSInterruptException:
        logger.cleanup()
