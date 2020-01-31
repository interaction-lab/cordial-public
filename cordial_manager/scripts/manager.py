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
roslib.load_manifest('cordial_manager')


class InteractionManager():

    def __init__(self):
        # Node startup
        rospy.init_node("interaction_controller_node", anonymous=True)

        # Setup server for decision manager to call
        self.interaction_server = actionlib.SimpleActionServer('do_interaction', ManagerAction,
                                                               self.handle_interacting, False)
        self.interaction_server.start()
        self._feedback = ManagerFeedback()
        self._result = ManagerResult()

        # Setup clients for all of the different nodes
        self.action_clients = {}
        self.action_feedback = {}
        self.action_result = {}
        topics = ["behaving", "long_behaving", "synthesizing", "dialoging", "sensing", "detecting"]
        for topic_name in topics:
            self.action_clients[topic_name] = actionlib.SimpleActionClient(topic_name, InteractionAction)
            #self.action_clients[topic_name].wait_for_server()
            self.action_result[topic_name] = {
                "interaction_continue": False,
                "message": ""}
            self.action_feedback[topic_name] = {
                "interaction_state": ""}

        self.read_interactions()
        self.handle_interacting("greeting1")  # FOR TESTING
        rospy.spin()

    def read_interactions(self):
        base_dir = os.path.dirname(__file__)
        with open(base_dir + '/data/test.json', "r") as json_file:
            self.interaction_data = json.load(json_file)

    def action_done_callback(self, terminal_state, result):
        print("Heard back from: "+ result.interacting_action, terminal_state, result)
        self.action_result[result.interacting_action]["interaction_continue"] = result.interaction_continue
        self.action_result[result.interacting_action]["message"] = result.message
        return

    def action_feedback_callback(self, feedback):
        self.action_feedback[feedback.interacting_action]["interaction_state"] = feedback.interaction_state
        return

    def handle_interacting(self, data):
        # Load interaction block specified by the manager
        print(data)
        interaction = self.interaction_data[data]  # TODO update to be data.something?
        interaction_steps = self.interaction_data[data]['steps']
        max_wait_time = rospy.Duration.from_sec(60.0)  # TODO parameterize

        # Iterate through block of steps
        for action in interaction_steps:

            # Do Action
            print("Begining action step:" + action["description"])
            goal = InteractionGoal()
            goal.interacting_action = action["action"]
            goal.optional_data = action["goal"]
            self.action_clients[action["action"]].send_goal(goal,
                                                  done_cb=self.action_done_callback,
                                                  feedback_cb=self.action_feedback_callback)

            # Wait if blocking
            if action["running_option"] == "block":
                print("Waiting for action step to complete")
                self.action_clients[action["action"]].wait_for_result(max_wait_time)

            # Enter loop if dialoging
            if action["running_option"] == "loop":
                print("Entering action loop")
                while not rospy.is_shutdown():  # dialoguing loops continue until interaction_continue is false
                    # do loop (synthesize->behave->sense->dialoging)
                    # expected behavior is to start looping after starting a dialogue
                    for loop_action in ["synthesizing", "behaving" , "sensing", "dialoging"]:
                        print("sending out instruction to start - " + loop_action)
                        goal = InteractionGoal(interacting_action=loop_action, optional_data="")
                        print("goal message:", goal)
                        self.action_clients[loop_action].send_goal(goal,
                                                                   done_cb=self.action_done_callback,
                                                                   feedback_cb=self.action_feedback_callback)
                        self.action_clients[loop_action].wait_for_result(max_wait_time)
                        if not self.action_result[loop_action]["interaction_continue"]:
                            print("received instructions to stop - do not continue")
                            # Check if error, if not error--finish synth and behave
                            message = self.action_result[loop_action]["message"]
                            status = message.split('_')[0]
                            if status == "success":
                                for loop_action in ["synthesizing", "behaving"]:
                                    print("sending out instruction to start - " + loop_action)
                                    goal = InteractionGoal(interacting_action=loop_action, optional_data="")
                                    print("goal message:", goal)
                                    self.action_clients[loop_action].send_goal(goal,
                                                                            done_cb=self.action_done_callback,
                                                                            feedback_cb=self.action_feedback_callback)
                                    self.action_clients[loop_action].wait_for_result(max_wait_time)
                            break  # the for loop
                    else:
                        continue
                    break  # the while loop if you broke the for loop
                    rospy.sleep()
                # check error type and make a decision about breaking out of the block
                if not self.action_result[loop_action]["interaction_continue"]:
                    message = self.action_result[loop_action]["message"]
                    status = message.split('_')[0]
                    if status == "success":
                        continue
                    else:
                        error = message.split('_')[1]
                        print("Do not continue error said:", error)
                        self._result.action_block_success = False
                        self._result.error_message = error
                        self.interaction_server.set_aborted(self._result)
                        return()

            if not self.action_result[action["action"]]["interaction_continue"]:
                message = self.action_result[action["action"]]["message"]
                status = message.split('_')[0]
                if status == "success":
                    continue
                else:
                    error = message.split('_')[1]
                    print("Do not continue error(main loop) said:", error)
                    self._result.action_block_success = False
                    self._result.error_message = error
                    self.interaction_server.set_aborted(self._result)

        # When the block has been successfully completed
        self._result.action_block_success = True
        self._result.error_message = ""
        self.interaction_server.set_succeeded(self._result)


if __name__ == '__main__':
    InteractionManager()


# EXAMPLE:
# FEEDBACK_MESSAGE = ''
# INTERACTION_MESSAGE = ''
# INTERACTION_CONTINUE = True

# class SynthesizeServer():
#     _feedback = InteractionFeedback()
#     _result = InteractionResult()
#     def __init__(self, name):
#         self.action_name = name
#         self.action = actionlib.SimpleActionServer(self.action_name, InteractionAction, self.execute_goal, False)
#         self.action.start()

#     def execute_goal(self, goal):
#         goal_name = goal.interacting_action
#         success = True
#         if goal.optional_data != '':
#             DIALOGUE_MESSAGE = optional_data
#         TTSManager.handle_tts_realtime(DIALOGUE_MESSAGE)
#         self._feedback.interacting_action = goal_name
#         self._feedback.interacting_state = FEEDBACK_MESSAGE
#         ## Decide when to send the feedback
#         # self.action.publish_feedback(self._feedback)
#         while not SYNTHESIZE_DONE:
#             if self.action.is_preempt_requested():
#                     self.action.set_preempted()
#                     success = False
#         if success:
#             self._result.interaction_continue = INTERACTION_CONTINUE
#             self._result.interacting_action = goal_name
#             self._result.message = INTERACTION_MESSAGE
#             self.action.set_succeeded(self._result)
