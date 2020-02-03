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

class DecistionState():
    SUCCESS = 0
    FAILURE = 1
    EMERGENCY = 2

class DecisionManager():

    def __init__(self):
        #Initialize the state variable
        self.state = DecistionState.SUCCESS
        self.index = 0
        self.failure_counter = 0
        self.action_feedback = {}
        self.action_result = {}
        # Setup clients for all of the different interaction blocks
        topic_name = "do_interaction"
        self.action_client = actionlib.SimpleActionClient(topic_name, ManagerAction)
        print("Waiting for the server")
        self.action_client.wait_for_server()
        print("Server is ready")
        self.success_action_name = ['greeting1', 'greeting2']
        self.failure_action_name = ['greeting1b', 'greeting2b']
        topics = self.success_action_name + self.failure_action_name
        for topic_name in topics:
            self.action_result[topic_name] = {
                    "action_block":"",
                    "action_block_continue": False,
                    "message": ""}
            self.action_feedback[topic_name] = {
                    "action_block":"",
                    "action_block_state": ""}
        self.send_request_to_interaction_manager(self.success_action_name[self.index])
       

    def send_request_to_interaction_manager(self, action_name):
        goal = ManagerGoal()
        goal.action_block = action_name
        goal.optional_data = ""
        print("The goal to be sent is:", goal)
        self.action_client.send_goal(goal,
                                    done_cb=self.action_done_callback,
                                    feedback_cb=self.action_feedback_callback)
        self.action_client.wait_for_result(rospy.Duration(10))

    def action_done_callback(self, terminal_state, result):
        print("Heard back from: "+ result.action_block, terminal_state, result)
        self.action_result[result.action_block]["action_block_continue"] = result.action_block_continue
        self.action_result[result.action_block]["message"] = result.message

        if self.action_result[result.action_block]["action_block_continue"]:
            self.failure_counter = 0
            self.index += 1
            self.state = DecistionState.SUCCESS
            self.send_request_to_interaction_manager(self.success_action_name[self.index])
        else:
            if message == 'failure':
                self.failure_counter += 1
                self.index = self.index
                self.state = DecisionState.FAILURE
                while self.failure_counter < 4: # 3 re-entry allowed
                    self.send_request_to_interaction_manager(self.failure_action_name[self.index])
            elif message == 'emergency':
                self.state = DecistionState.EMERGENCY
        return
        
    

    def action_feedback_callback(self, feedback):
        self.action_feedback[feedback.action_block]["action_block_state"] = feedback.action_block_state
        return


if __name__ == '__main__':
    rospy.init_node("decision_controller_node", anonymous=True)
    DecisionManager()
    rospy.spin()