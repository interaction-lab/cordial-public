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

class LongSensorsServer():
	_feedback = InteractionFeedback()
	_result = InteractionResult()
	def __init__(self, name):
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, InteractionAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		success = True
		if goal.optional_data != '':
			LISTENING_MESSAGE = optional_data
		SensorsManager.handle_listening_start(LISTENING_MESSAGE)
		while not LISTENING_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
		if success:
			self._result.interacting_success = True

class SensorsServer():
	_feedback = InteractionFeedback()
	_result = InteractionResult()
	def __init__(self, name):
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, InteractionAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		success = True
		LISTENING_MESSAGE = True
		if goal.optional_data != '':
			LISTENING_MESSAGE = optional_data
		SensorsManager.handle_listening_start(LISTENING_MESSAGE)
		while not LISTENING_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
		if success:
			self._result.interacting_success = True


class SensorsManager():
	def __init__(self):
		rospy.init_node("sensor_node", anonymous=True)
		rospy.Subscriber('cordial/listening/done', Bool, self.handle_listening_done)
		self.microphone_publisher = rospy.Publisher("cordial/listening", Bool, queue_size=1)
		self.camera_record_publisher = rospy.Publisher("cordial/recording/video", Bool, queue_size=1)
		self.microphone_record_publisher = rospy.Publisher("cordial/recording/audio", Bool, queue_size=1)
		rospy.spin()

	def handle_listening_start(self, data):
		self.microphone_publisher.publish(data)

	def handle_listening_done(self,data):
		LISTENING_DONE = data.data

if __name__ == '__main__':
    SensorsManager()
	SensorsServer("sensing")
	LongSensorsServer("long_sensing")
