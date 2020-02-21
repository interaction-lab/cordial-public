#!/usr/bin/env python

import rospy
import os
import sys
import json
import time
import actionlib
from std_msgs.msg import String, Bool
from cordial_manager.msg import *

class LongSensorsServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.recording_message = True
		self.controller_manager.recording_done = False
		self.controller_manager.long_interaction_message = ''
		self.controller_manager.long_interaction_continue = False
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			self.controller_manager.recording_message = False
		if goal.optional_data == "stop_recording": ##CHANGE IT IN V2
			self.controller_manager.recording_message = False
		elif goal.optional_data == "start_recording": ##CHANGE IT IN V2
			self.controller_manager.recording_done = True
		self.controller_manager.handle_recording_start(self.controller_manager.recording_message)
		self._feedback.action = goal_name
		#self._feedback.state = # Controller state
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not self.controller_manager.recording_done:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = self.controller_manager.long_interaction_continue
			self._result.action = goal_name
			self._result.message = self.controller_manager.long_interaction_message
			self.controller_manager.recording_message = ''
			self.controller_manager.recording_done = False
			self.controller_manager.long_interaction_message = ''
			self.controller_manager.long_interaction_continue = False
			self.action.set_succeeded(self._result)

class SensorsServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.listening_start = True
		self.controller_manager.listening_done = False
		self.controller_manager.interaction_message = ''
		self.controller_manager.interaction_continue = True
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			if goal.optional_data == "userlost":
				self.controller_manager.listening_start = False
				self.controller_manager.listening_done = True
				print("If user lost not listen but directly dialoging")
		self.controller_manager.handle_listening_start(self.controller_manager.listening_start)
		self._feedback.action = goal_name
		#self._feedback.state = #Controller state
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not self.controller_manager.listening_done:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = True
			self._result.action = goal_name
			self._result.message = self.controller_manager.interaction_message
			self.controller_manager.listening_start = True
			self.controller_manager.listening_done = False
			self.controller_manager.interaction_message = ''
			self.controller_manager.interaction_continue = True
			self.action.set_succeeded(self._result)


class SensorsManager():
	def __init__(self):
		# Initialize variables useful for the Server
		self.listening_start = True ##Bool which trigger the microphone
		self.recording_message = ""
		self.listening_done = False
		self.recording_done = False
		self.interaction_message = ""
		self.interaction_continue = True
		self.long_interaction_message = ""
		self.long_interaction_continue = False
		# Declare subscribers and publishers
		rospy.Subscriber('cordial/listening/done', Bool, self.handle_listening_done, queue_size=1)
		#self.microphone_data_publisher = rospy.Publisher('/cordial/microphone/audio', AudioData, self.handle_audio_data, queue_size=1)
		self.microphone_publisher = rospy.Publisher("cordial/listening", Bool, queue_size=1)
		self.camera_record_publisher = rospy.Publisher("cordial/recording/video", Bool, queue_size=1)
		self.microphone_record_publisher = rospy.Publisher("cordial/recording/audio", Bool, queue_size=1)
		self.data_record_publisher = rospy.Publisher("cordial/recording/data", Bool, queue_size=1)
		

	def handle_listening_start(self, data):
		self.microphone_publisher.publish(data)

	def handle_recording_start(self, data):
		self.camera_record_publisher.publish(data)
		#self.microphone_record_publisher.publish(data)
		self.data_record_publisher.publish(data)
		return

	def handle_listening_done(self,data):
		self.listening_done = data.data

if __name__ == '__main__':
		rospy.init_node("sensor_node", anonymous=True)
		controller_manager = SensorsManager()
		SensorsServer("sensing", controller_manager)
		LongSensorsServer("long_sensing", controller_manager)
		rospy.spin()
