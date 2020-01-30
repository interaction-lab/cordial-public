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


class DetectorServer():
	_feedback = InteractionFeedback()
	_result = InteractionResult()
	def __init__(self, name):
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, InteractionAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		success = True
		FACE_DETECTING_MESSAGE = True
		if goal.optional_data != '':
			FACE_DETECTING_MESSAGE = optional_data
		DetectorManager.handle_face_detecting_start(FACE_DETECTING_MESSAGE)
		while not DETECTING_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
		if success:
			self._result.interacting_success = True


class DetectorManager():
	def __init__(self):
		rospy.init_node("detector_node", anonymous=True)
		rospy.Subscriber('cordial/detector/faces/done', Bool, self.handle_face_detecting_done)
		self.face_detector_publisher = rospy.Publisher("cordial/detector/faces/detecting", Bool, queue_size=1)
		rospy.spin()

	def handle_face_detecting_start(self, data):
		self.face_detector_publisher.publish(data)

	def handle_face_detecting_done(self,data):
		DETECTING_DONE = data.data #it will be true just when all the detections finish!

if __name__ == '__main__':
    DetectorManager()
	DetectorServer("detecting")
