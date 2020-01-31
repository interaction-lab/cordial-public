#!/usr/bin/env python

from std_msgs.msg import String, Bool
from cordial_manager.msg import *
import time
import json
import os
import sys
import actionlib
import rospy
import roslib


class DecisionManager():

    def __init__(self):
        # Setup clients for all of the different interaction blocks
        topic_name = "do_interaction"
        self.action_client = actionlib.SimpleActionClient(topic_name, ManagerAction)
        print("Waiting for the server")
        self.action_client.wait_for_server()
        print("Server is ready")
        self.action_result = {
                "action_block_continue": False,
                "message": ""}
        self.action_feedback = {
                "action_block_state": ""}
        action_name = "greeting1"
        self.send_request_to_interaction_manager(action_name)

    def send_request_to_interaction_manager(self, action_name):
        goal = ManagerGoal()
        goal.action_block = action_name
        goal.optional_data = ""
        print("The goal to be sent is:", goal)
        self.action_client.send_goal(goal,
                                    done_cb=self.action_done_callback,
                                    feedback_cb=self.action_feedback_callback)

    def action_done_callback(self, terminal_state, result):
        print("Heard back from: "+ result.action_block, terminal_state, result)
        self.action_result[result.action_block]["action_block_continue"] = result.action_block_continue
        self.action_result[result.action_block]["message"] = result.message
        return

    def action_feedback_callback(self, feedback):
        self.action_feedback[feedback.action_block]["action_block_state"] = feedback.action_block_state
        return


if __name__ == '__main__':
    rospy.init_node("decision_controller_node", anonymous=True)
    DecisionManager()
    rospy.spin()