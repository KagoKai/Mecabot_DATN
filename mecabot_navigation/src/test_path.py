#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import UInt8
import time
import tf
import tf.transformations

class GoalSender:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Predefined goals
        self.goals = [
            self.create_goal(3.6, 0.0, 0.0),
            self.create_goal(3.6, 1.2, 0.0),
            self.create_goal(3.6, 2.4, 0.0)
        ]

        # Subscribe to /start_path topic
        self.start_subscriber = rospy.Subscriber('/start_path', UInt8, self.start_callback)

        # Variable to keep track of start signal
        self.start_signal_received = False

    def create_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose = tf.transformations.quaternion_from_euler(0, 0, yaw)
        return goal

    def start_callback(self, msg):
        if msg.data == 1:
            self.start_signal_received = True
            rospy.loginfo("Start signal received. Sending goals.")
            self.send_goals()

    def send_goals(self):
        for goal in self.goals:
            if not self.start_signal_received:
                break  # Exit if start signal is no longer valid

            rospy.loginfo("Sending goal: %s", goal)
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            rospy.loginfo("Result: %s", result)
            rospy.sleep(5)  # Wait for 5 seconds before sending the next goal

if __name__ == '__main__':
    rospy.init_node('goal_sender_node')
    goal_sender = GoalSender()
    rospy.loginfo("Goal sender node is ready. Waiting for start signal.")
    rospy.spin()
