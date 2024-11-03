#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped

global pose,scan
pose = [0,0,0]

## quaternion to euler
def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)
########################


def RobotPose(data):
    global pose

    pose = [data.pose.pose.position.x,
            data.pose.pose.position.y,
            euler_from_quaternion([data.pose.pose.orientation.x,
                                   data.pose.pose.orientation.y,
                                   data.pose.pose.orientation.z,
                                   data.pose.pose.orientation.w])[2]]
    
## Main Node
def controller():

    global pose
    
    # Initialise ROS node
    rospy.init_node('main_controller', anonymous=True)
    rospy.Subscriber('/pf/pose/odom',Odometry,RobotPose)
    pub = rospy.Publisher('/racecar/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)
    velocity_msg = AckermannDriveStamped()
    rate = rospy.Rate(10)

    while(len(pose)==0):
        print('First pose not obtained yet')
        continue

    # Defining trajectory parameters
    A  = 5          # Radius [m]
    Omega = 0.5      # Angular velcoity [rad/s]
    K1 = 1.0        # Proportional gain for velocity
    K2 = 1.0        # Proportional gain for steering angle

    while not rospy.is_shutdown():
        
        #Add your control logic here

        #Based on the given question, choose the reference trajctory to be on the circle : (x_r = cos(omega*t), y_r = sin(omega*t))
        #Here 't' is the time parameter
        #The control design returns the linear velocity - 'v' and angular velocity - 'omega'

        #From (v,omega) calculate the linear velocity and the steering angle to be applied to MIT Racecar.

        t = rospy.get_time()
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

        # Control law to compute linear velocity and steering angle
        velocity_msg.drive.speed = K1 * math.sqrt(x_error**2 + y_error**2)  # Linear velocity proportional to distance error
        velocity_msg.drive.steering_angle = K2 * heading_error  # Steering angle proportional to heading error

        pub.publish(velocity_msg)

        rate.sleep()
########################

if __name__ == '__main__':
    try:
        time.sleep(5)
        controller()
    except rospy.ROSInterruptException:
        pass

