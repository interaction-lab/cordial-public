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

INTERACTION_TIMEOUT = rospy.Duration.from_sec(60.0)


class InteractionManager():
    """Receive interactions and execute interaction steps"""
    _feedback = CordialFeedback()
    _result = CordialResult()
    def __init__(self):
        # Read interactions from json file
        self.read_interactions()
        # Setup server for decision manager to call
        self.interaction_server = actionlib.SimpleActionServer('do_interaction', CordialAction,
                                                               self.interaction_command_callback, False)
        
        # Setup clients for all of the different nodes
        self.action_clients = {}
        self.action_result = {}
        self.action_feedback = {}
        action_types = ["behaving", "long_behaving", "synthesizing", "dialoging", "sensing","long_sensing", "detecting"]
        for action_type in action_types:
            self.action_clients[action_type] = actionlib.SimpleActionClient(action_type, CordialAction)
            rospy.loginfo("Waiting for {} module server".format(action_type))
            self.action_clients[action_type].wait_for_server()
            self.action_result[action_type] = {
                "do_continue": False,
                "message": ""}
            self.action_feedback[action_type] = {
                "state": ""}
        rospy.loginfo("Module servers initialization completed")
        self.interaction_server.start()
        rospy.loginfo("The interaction_manager server starts")
        return
   

    def read_interactions(self, local_path="/data/test.json"):
        base_dir = os.path.dirname(__file__)
        with open(base_dir + local_path, "r") as json_file:
            self.interaction_data = json.load(json_file)
        return


    def action_done_callback(self, terminal_state, result):
        """Save action results"""
        rospy.logdebug("Heard back from: "+ result.action, terminal_state, result)
        self.action_result[result.action]["do_continue"] = result.do_continue
        self.action_result[result.action]["message"] = result.message
        return


    def action_feedback_callback(self, feedback):
        """Save action feedback"""
        self.action_feedback[feedback.action]["state"] = feedback.state
        return


    def interaction_command_callback(self, goal_data):
        """Handle interaction command"""

        # Recieve goal and load steps
        interaction_block_label = goal_data.action
        rospy.loginfo("The interaction manager received the following command block:"+ str(interaction_block_label))
        interaction_steps = self.interaction_data[interaction_block_label]['steps']
        rospy.logdebug("The interaction steps are:", interaction_steps)

        # Iterate through block of steps
        for action in interaction_steps:
            rospy.logdebug("The action is:", action)
            rospy.loginfo("Beginning action step:" + action["description"])
            self._send_goal_(action["action"], 
                            optional=action["goal"], 
                            wait=action["running_option"] == "block")

            # Enter loop if dialoging
            if action["running_option"] == "loop":
                rospy.loginfo("Entering action loop")
                loop_action = self._process_looping_action_()

                # After exiting the loop check if loop action failed
                if not self.action_result[loop_action]["do_continue"]:
                    action_success, error_message = self._check_result_status_()
                    if action_success:
                        continue
                    else:
                        self._send_interaction_result_(interaction_block_label, False, error_message)
                        return()

            # At the end of the a step check if the step failed
            if not self.action_result[action["action"]]["do_continue"]:
                action_success, error_message = self._check_result_status_()
                if action_success:
                    continue
                else:
                    self._send_interaction_result_(interaction_block_label, False, error_message)
                    return()

        # When the block has been successfully completed
        self._send_interaction_result_(interaction_block_label,True,"")
        return


    def _check_result_status_(self):
        message = self.action_result[loop_action]["message"]
        status = message.split('_')[0]
        if status == "success":
            return(True, "")
        else:
            error = message.split('_')[1]
            return(False, error)


    def _send_interaction_result_(self, label, action_continue, message):
        self._result.action = label
        self._result.do_continue = action_continue
        self._result.message = message
        if action_continue:
            self.interaction_server.set_succeeded(self._result)     
        else:
            rospy.loginfo("Do not continue, error said:", error)
            self.interaction_server.set_aborted(self._result)
        return 

    def _process_looping_action_(self):
        while not rospy.is_shutdown():  # dialoguing loops continue until do_continue is false

            # expected behavior is to start looping after starting a dialogue
            for loop_action in ["synthesizing", "behaving", "sensing", "dialoging"]:
                self._send_goal_(loop_action)

                if not self.action_result[loop_action]["do_continue"]:
                    rospy.loginfo("received instructions to stop - do not continue")
                    # Check if error, if not error--finish synth and behave
                    message = self.action_result[loop_action]["message"]
                    status = message.split('_')[0]

                    # Still need to finish behaving after dialogue indicates success
                    if status == "success":
                        for loop_action in ["synthesizing", "behaving"]:
                            _send_goal_(loop_action)
                    break  # the for loop
            else:
                continue
            break  # the while loop if you broke the for loop
            rospy.sleep()
        return(loop_action)


    def _send_goal_(self, action, optional = "", wait=True):
        rospy.loginfo("sending out instruction to start - " + action)
        goal = CordialGoal(action=action, optional_data=optional)
        rospy.loginfo("goal message:"+ str(goal))
        self.action_clients[action].send_goal(goal,
                                                    done_cb=self.action_done_callback,
                                                    feedback_cb=self.action_feedback_callback)
        if wait:
            self.action_clients[action].wait_for_result(INTERACTION_TIMEOUT)
        return


if __name__ == '__main__':
    rospy.init_node("interaction_controller_node", anonymous=True)
    InteractionManager()
    rospy.spin()