#!/usr/bin/env python

from cordial_manager.msg import *
from std_msgs.msg import String, Bool
import time
import json
import os
import sys
import actionlib
import rospy
import roslib
roslib.load_manifest('cordial_behavior')


class InteractionManager():

    def __init__(self):
        # Node startup
        rospy.init_node("interaction_controller_node", anonymous=True)

        # Setup server for decision manager to call
        self.interaction_server = actionlib.SimpleActionServer('do_interaction', BehaviorAction,
                                                               self.handle_interacting, False)
        self.interaction_server.start()

        self.action_clients = {}
        # Setup clients for all of the different nodes
        topics = ["behaving", "long_behaving", "synthesizing", "dialoging", "sensing", "detecting"]
        for topic_name in topics:
            self.action_clients[topic_name] = actionlib.SimpleActionClient(topic_name, InteractionAction)
            self.action_clients[topic_name].wait_for_server()
        # # Behavior
        # self.action_clients['behaving']
        # self.behavior_client = actionlib.SimpleActionClient("behaving", BehaviorAction)
        # self.behavior_client.wait_for_server()

        # # Behavior Long Running
        # self.long_running_behavior_client = actionlib.SimpleActionClient("long_behaving", BehaviorAction)
        # self.long_running_behavior_client.wait_for_server()

        # # Text-to-Speech (Polly)
        # self.tts_client = actionlib.SimpleActionClient("synthesizing", BehaviorAction)
        # self.tts_client.wait_for_server()

        # # Dialogue Handler (Lex)
        # self.dialogue_client = actionlib.SimpleActionClient("dialoging", BehaviorAction)
        # self.dialogue_client.wait_for_server()

        # # Sensors Handler
        # self.sensor_client = actionlib.SimpleActionClient("sensing", BehaviorAction)
        # self.sensor_client.wait_for_server()

        # # Detectors Handler
        # self.detector_client = actionlib.SimpleActionClient("detecting", BehaviorAction)
        # self.detector_client.wait_for_server()

        self.read_interactions()
        self.handle_interacting("greeting1")  # FOR TESTING
        rospy.spin()

    def read_interactions(self):
        base_dir = os.path.dirname(__file__)
        with open(base_dir + '/data/interactions.json', "r") as json_file:
            self.interaction_data = json.load(json_file)

    def handle_interacting(self, data):
        # Load interaction block specified by the manager
        interaction = self.interaction_data[data]  # TODO update to be data.something
        interaction_steps = self.interaction_data[interaction]['steps']

        # Iterate through specified steps
        for action in interaction_steps:
            # Do Action
            print(action["description"])
            goal = BehaviorActionGoal(action["name"])
            self.action_clients[action["action"]].send_goal(goal)
            # Wait if blocking
            if action["running_option"] == "block":
                self.action_clients[action["action"]].wait_for_result(rospy.Duration.from_sec(60.0))
            # Enter loop if dialoging
            if action["running_option"] == "block":


if __name__ == '__main__':
    InteractionManager()
