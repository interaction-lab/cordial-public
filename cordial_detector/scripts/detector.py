#!/usr/bin/env python

import rospy
import os
import sys
import json
import time
import actionlib
from std_msgs.msg import String, Bool
from cordial_manager.msg import *


FACE_DETECTING_MESSAGE = ''
DETECTING_DONE = False

FEEDBACK_MESSAGE = ''
INTERACTION_MESSAGE = ''
INTERACTION_CONTINUE = True


class DetectorServer():
	_feedback = InteractionFeedback()
	_result = InteractionResult()
	def __init__(self, name):
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, InteractionAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.interacting_action
		success = True
		FACE_DETECTING_MESSAGE = True
		if goal.optional_data != '':
			FACE_DETECTING_MESSAGE = optional_data
		DetectorManager.handle_face_detecting_start(FACE_DETECTING_MESSAGE)
		self._feedback.interacting_action = goal_name
		self._feedback.interacting_state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not DETECTING_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
		if success:
			self._result.interaction_continue = INTERACTION_CONTINUE
			self._result.interacting_action = goal_name
			self._result.message = INTERACTION_MESSAGE
			self.action.set_succeeded(self._result)


class DetectorManager():
	def __init__(self):
		self.face_detector_publisher = rospy.Publisher("cordial/detector/faces/detecting", Bool, queue_size=1)
		
	def handle_face_detecting_start(self, data):
		self.face_detector_publisher.publish(data)

if __name__ == '__main__':
	rospy.init_node("detector_node", anonymous=True)
    DetectorManager()
	DetectorServer("detecting")
	rospy.spin()

