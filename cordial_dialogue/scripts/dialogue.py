#!/usr/bin/env python
from sys import byteorder
from array import array
from struct import pack
import sys
import pyaudio
import wave
import struct
import rospy
import actionlib
from lex_common_msgs.srv import *
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from cordial_manager.msg import *


class DialogueServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.prompt_message = ''
		self.controller_manager.dialogue_process_done = False
		self.controller_manager.interaction_message = ''
		self.controller_manager.interaction_continue = True
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		print(goal)
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			self.controller_manager.prompt_message = goal.optional_data
			print("Dialogue has heard: " + self.controller_manager.prompt_message)
			self.controller_manager.send_textToAWS(self.controller_manager.prompt_message)
		else:
			while self.controller_manager.prompt_message == '':
				print("Waiting from the microphone input data")
				rospy.Rate(10)
			self.controller_manager.send_audioToAWS_client(self.controller_manager.prompt_message)
		self._feedback.action = goal_name
		#self._feedback.state = #Controller state
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not self.controller_manager.dialogue_process_done:
			if self.action.is_preempt_requested():
					print("action preempted")
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			print("enter if success")
			self._result.do_continue = self.controller_manager.interaction_continue
			self._result.action = goal_name
			self._result.message = self.controller_manager.interaction_message
			self.controller_manager.prompt_message = ''
			self.controller_manager.dialogue_process_done = False
			self.controller_manager.interaction_message = ''
			self.controller_manager.interaction_continue = True
			self.action.set_succeeded(self._result)


class DialogueManager():
	def __init__(self):
		# Initialize variables useful for the Server
		self.prompt_message = ""
		self.dialogue_process_done = False
		self.interaction_message = ""
		self.interaction_continue = True
		# Declare subscribers and publishers
	   	rospy.Subscriber('/cordial/microphone/audio', AudioData, self.handle_audio_data, queue_size=1)
		self.text_publisher = rospy.Publisher('cordial/dialogue/script', String, queue_size=1)
		

	def handle_audio_data(self, data):
		self.prompt_message = data.data

	def handle_lex_response(self,lex_response):
		if len(lex_response.text_response) > 0:
			print("The lex response is: ", lex_response)
			#When lex failed in understanding the user
			if lex_response.dialog_state == 'Failed':
				self.interaction_message = 'failed_understanding'
				self.interaction_continue = False
				print("In Failed dialogue state")
			elif lex_response.dialog_state == 'Fulfilled':
				self.interaction_message = 'success'
				self.interaction_continue = False
				print("In Fulfilled dialogue state, the response is:" + lex_response.text_response)
				self.text_publisher.publish(lex_response.text_response)
			else:
				print("In general dialogue state, the response is:" + lex_response.text_response)
				self.text_publisher.publish(lex_response.text_response)
			self.dialogue_process_done = True

	def send_audioToAWS_client(self,audiodatarequest):
		print("Starting client request..")
		#print("The audio data request is: ", audiodatarequest)
		audiodata = audiodatarequest
		rospy.wait_for_service("/lex_node/lex_conversation")
		try:
			send_data_service = rospy.ServiceProxy("/lex_node/lex_conversation", AudioTextConversation)
			lex_response = send_data_service(content_type='audio/x-l16; sample-rate=16000; channel-count=1',
		                           accept_type='text/plain; charset=utf-8',
		                           text_request=None,
		                           audio_request=AudioData(data=audiodata))
			self.handle_lex_response(lex_response)
		except rospy.ServiceException, e:
			print "Service call failed: %s" %e

	def send_textToAWS(self,textdatarequest):
		print("Starting client request..")
		#textdata = textdatarequest.data
		textdata = textdatarequest # FOR TESTING
		rospy.wait_for_service("/lex_node/lex_conversation")
		try:
			send_data_service = rospy.ServiceProxy("/lex_node/lex_conversation", AudioTextConversation)
			lex_response = send_data_service(content_type='text/plain; charset=utf-8',
		                           accept_type='text/plain; charset=utf-8',
		                           text_request= textdata,
		                           audio_request= AudioData(data=''))
			self.handle_lex_response(lex_response)
		except rospy.ServiceException, e:
			print "Service call failed: %s" %e

	
	

if __name__ == '__main__':
		rospy.init_node("dialogue_node", anonymous=True)
		controller_manager = DialogueManager()
		DialogueServer("dialoging", controller_manager)
		rospy.spin()
    
    




