#!/usr/bin/env python

import rospy
import os
import sys
import json
import time
import actionlib
from std_msgs.msg import String, Bool
from cordial_manager.msg import *


LISTENING_MESSAGE = ''
RECORDING_MESSAGE = ''
LISTENING_DONE = False
RECORDING_DONE = False
FEEDBACK_MESSAGE = ''
INTERACTION_MESSAGE = ''
INTERACTION_CONTINUE = True

class LongSensorsServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.sm = manager
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		global RECORDING_DONE, RECORDING_MESSAGE, INTERACTION_MESSAGE, INTERACTION_CONTINUE, FEEDBACK_MESSAGE
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			LISTENING_MESSAGE = goal.optional_data
		self.sm.handle_listening_start(LISTENING_MESSAGE)
		self._feedback.action = goal_name
		self._feedback.state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not LISTENING_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = INTERACTION_CONTINUE
			self._result.action = goal_name
			self._result.message = INTERACTION_MESSAGE
			RECORDING_MESSAGE = ''
			RECORDING_DONE = False
			FEEDBACK_MESSAGE = ''
			INTERACTION_MESSAGE = ''
			INTERACTION_CONTINUE = True
			self.action.set_succeeded(self._result)

class SensorsServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.sm = manager
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		global LISTENING_MESSAGE, LISTENING_DONE, INTERACTION_CONTINUE, INTERACTION_MESSAGE, FEEDBACK_MESSAGE
		goal_name = goal.action
		success = True
		LISTENING_MESSAGE = True
		if goal.optional_data != '':
			LISTENING_MESSAGE = goal.optional_data
		self.sm.handle_listening_start(LISTENING_MESSAGE)
		self._feedback.action = goal_name
		self._feedback.state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not LISTENING_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = True
			self._result.action = goal_name
			self._result.message = INTERACTION_MESSAGE
			LISTENING_MESSAGE = ''
			LISTENING_DONE = False
			FEEDBACK_MESSAGE = ''
			INTERACTION_MESSAGE = ''
			INTERACTION_CONTINUE = True
			self.action.set_succeeded(self._result)


class SensorsManager():
	def __init__(self):
		rospy.Subscriber('cordial/listening/done', Bool, self.handle_listening_done, queue_size=1)
		self.microphone_publisher = rospy.Publisher("cordial/listening", Bool, queue_size=1)
		self.camera_record_publisher = rospy.Publisher("cordial/recording/video", Bool, queue_size=1)
		self.microphone_record_publisher = rospy.Publisher("cordial/recording/audio", Bool, queue_size=1)
		

	def handle_listening_start(self, data):
		self.microphone_publisher.publish(data)

	def handle_listening_done(self,data):
		global LISTENING_DONE
		LISTENING_DONE = data.data

if __name__ == '__main__':
		rospy.init_node("sensor_node", anonymous=True)
		sm = SensorsManager()
		SensorsServer("sensing", sm)
		LongSensorsServer("long_sensing", sm)
		rospy.spin()
