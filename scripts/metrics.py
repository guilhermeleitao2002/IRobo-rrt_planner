#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from rosgraph_msgs.msg import Log
from time import time

def calculate_avg_path_error(path_errors):
        sum = 0
        for i in path_errors:
            sum += i
        return sum/len(path_errors)


class PathMetrics:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('path_metrics', anonymous=True)

        # Subscribers to get the global and actual paths
        self.global_path_sub = rospy.Subscriber('/move_base/RRTPlannerROS/global_plan', Path, self.global_path_callback)
        self.real_path_sub = rospy.Subscriber('/trajectory', Path, self.real_path_callback)

        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.robot_pose_callback)

        # Subscribe to the goal position
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        rospy.Subscriber('/rosout', Log, self.logcallback)

        # Variables to store paths
        self.global_path = []
        self.real_path = []
        self.start_time = None
        self.end_time = None
        self.num_retries = 0
        self.path_errors = []

        self.robot_position = None
        self.robot_orientation = None
        self.goal_position = None
        self.goal_orientation = None
        self.distance = 0
        self.yaw_diff = 0

        # Define tolerance for comparing paths (meters)
        self.path_tolerance = 0.1

    def global_path_callback(self, msg):
        # Start time when the global path is first planned
        if self.start_time is None:
            self.start_time = time()
        
        # Convert global path from Path message to a list of (x, y) tuples
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def real_path_callback(self, msg):
        # Convert real path from Path message to a list of (x, y) tuples
        self.real_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def robot_pose_callback(self, msg):
        # Extract the robot's current position and orientation
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_orientation = msg.pose.pose.orientation
        rospy.loginfo(f"Current robot position: {self.robot_position}")
        
        # Compare if goal is available
        if self.goal_position:
            self.compare_robot_to_goal()

    def goal_callback(self, msg):
        # Extract the goal's position and orientation
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        self.goal_orientation = msg.pose.orientation
        rospy.loginfo(f"Goal position: {self.goal_position}")
        
        # Compare if robot's position is available
        if self.robot_position:
            self.compare_robot_to_goal()


    def compare_robot_to_goal(self):
        # 1. Calculate Euclidean distance between robot and goal
        self.distance = self.euclidean_distance(self.robot_position, self.goal_position)

        # 2. Calculate orientation difference (yaw angle)
        robot_yaw = self.quaternion_to_yaw(self.robot_orientation)
        goal_yaw = self.quaternion_to_yaw(self.goal_orientation)
        self.yaw_diff = abs(goal_yaw - robot_yaw)

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw (rotation around Z axis)."""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_path_error(self, global_path, real_path):
        """Calculate the average error between the global and real paths."""
        total_error = 0
        count = 0

        for real_point in real_path:
            # Find the nearest point in the global path
            if(global_path):
                nearest_global_point = min(global_path, key=lambda p: self.euclidean_distance(real_point, p))
                total_error += self.euclidean_distance(real_point, nearest_global_point)
                count += 1
        
        return total_error / count if count > 0 else 0
    
    def logcallback(self, msg):
        # Check if the log message contains "Goal reached"
        if "Goal reached" in msg.msg:
            self.end_time = time()
            self.calculate_metrics()

    def calculate_metrics(self):
        # 1. Time to reach the goal
        
        # 3. Error between global and real paths
        self.path_errors.append(self.calculate_path_error(self.global_path, self.real_path))
        

        if self.start_time and self.end_time:
            time_to_goal = self.end_time - self.start_time
            rospy.loginfo(f"Time to reach goal: {time_to_goal:.2f} seconds")

        # 2. Distance traveled (real path)
        real_distance = self.calculate_path_distance(self.real_path)
        rospy.loginfo(f"Total distance traveled: {real_distance:.2f} meters")

        avg_error = calculate_avg_path_error(self.path_errors)
        rospy.loginfo(f"Avg path error: {avg_error}")

        rospy.loginfo(f"Distance to Goal: {self.distance}")
        rospy.loginfo(f"Yaw difference: {self.yaw_diff}")

        # 4. Number of tries (assuming each replan triggers a new global path)
        rospy.loginfo(f"Number of retries: {self.num_retries}")

        self.start_time = None
        self.end_time = None
        self.compare_robot_to_goal()

        

    def calculate_path_distance(self, path):
        """Calculate the total distance of the given path."""
        distance = 0
        for i in range(1, len(path)):
            distance += self.euclidean_distance(path[i-1], path[i])
        return distance

    def euclidean_distance(self, p1, p2):
        """Calculate Euclidean distance between two points (x1, y1) and (x2, y2)."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def increment_retries(self):
        """Method to be called when a replan happens (triggered manually or by planner)."""
        self.num_retries += 1

if __name__ == "__main__":
    try:
        # Initialize the PathMetrics object
        path_metrics = PathMetrics()

        # Keep the node alive
        rospy.spin()
    except rospy.ROSInterruptException:
        pass