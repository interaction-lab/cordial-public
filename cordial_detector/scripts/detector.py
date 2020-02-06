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
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.dm = manager
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		global FACE_DETECTING_MESSAGE, DETECTING_DONE, FEEDBACK_MESSAGE, INTERACTION_MESSAGE, INTERACTION_CONTINUE
		goal_name = goal.action
		success = True
		FACE_DETECTING_MESSAGE = True
		if goal.optional_data != '':
			FACE_DETECTING_MESSAGE = goal.optional_data
		self.dm.handle_face_detecting_start(FACE_DETECTING_MESSAGE)
		self._feedback.action = goal_name
		self._feedback.state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not DETECTING_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = INTERACTION_CONTINUE
			self._result.action = goal_name
			self._result.message = INTERACTION_MESSAGE
			FACE_DETECTING_MESSAGE = ''
			DETECTING_DONE = False
			FEEDBACK_MESSAGE = ''
			INTERACTION_MESSAGE = ''
			INTERACTION_CONTINUE = True
			self.action.set_succeeded(self._result)


class DetectorManager():
	def __init__(self):
		self.face_detector_publisher = rospy.Publisher("cordial/detector/faces/detecting", Bool, queue_size=1)
		
	def handle_face_detecting_start(self, data):
		self.face_detector_publisher.publish(data)
		

if __name__ == '__main__':
		rospy.init_node("detector_node", anonymous=True)
		dm = DetectorManager()
		DetectorServer("detecting", dm)
		rospy.spin()

