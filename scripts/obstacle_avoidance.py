#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('obstacle_avoidance')

        # Subscriber for the point cloud of obstacles captured by lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Subscriber for Ackermann control commands
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # TODO Publisher for modified Ackermann commands
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)

        # Store the last Ackermann message received
        self.last_ackermann_cmd = AckermannDrive()

        self.obstacles_near = False

    def obstacle_callback(self, msg):
        # TODO Process the point cloud with the obstacles taking into account the last received movement message to avoid collisions.
        point_list = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.obstacles_near = self.check_obstacles(point_list)
        #Send ackermann's message 
        # TODO Modify ackermann's message if necessary
        # self.modify_ackermann_command()
        rospy.loginfo(f"Obstacles near: {self.obstacles_near}")

        cmd = self.last_ackermann_cmd

        if self.obstacles_near:
            cmd.speed = 0.0

        self.cmd_pub.publish(cmd)
        
        return

    def check_obstacles(self, points):
        for point in points:
            x, y, z = point
            if x < 1.5 and abs(y) < 1.0:  
                return True
            
        return False

    def ackermann_callback(self, msg):
        # Stores the last command received
        self.last_ackermann_cmd = msg
        rospy.loginfo(f"Received Ackermann command: Speed={msg.speed}, Steering Angle={msg.steering_angle}")
        # self.modify_ackermann_command()


    def modify_ackermann_command(self):
        # Modify the Ackermann command to avoid obstacles (can be modified if necessary).
        cmd = AckermannDrive()
        cmd.drive.speed = 0.0  # Reduce speed
        cmd.drive.steering_angle = 0.0 # Change the steering angle
        
        # if self.obstacles_near:
        #     cmd = AckermannDrive()
        #     cmd.drive.speed = 0.0  # Reduce speed
        #     cmd.drive.steering_angle = 0.0 # Change the steering angle
        #     rospy.loginfo("Stopping due to an obstacle.")
        #     self.cmd_pub.publish(cmd)
        # else:
        #     rospy.loginfo(f"Publishing original Ackermann command: Speed={self.last_ackermann_cmd.drive.speed}, Steering Angle={self.last_ackermann_cmd.drive.steering_angle}")
        #     self.cmd_pub.publish(self.last_ackermann_cmd)
        

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
