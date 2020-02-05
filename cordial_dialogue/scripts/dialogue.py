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

PROMPT_MESSAGE = ''
DIALOGUE_PROCESSING_DONE = False
FEEDBACK_MESSAGE = ''
INTERACTION_MESSAGE = ''
INTERACTION_CONTINUE = True

class DialogueServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.dm = manager
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		global PROMPT_MESSAGE, DIALOGUE_PROCESSING_DONE, FEEDBACK_MESSAGE, INTERACTION_MESSAGE, INTERACTION_CONTINUE
		print(goal)
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			PROMPT_MESSAGE = goal.optional_data
			print("Dialogue has heard: " + PROMPT_MESSAGE)
			self.dm.send_textToAWS(PROMPT_MESSAGE)
		else:
			while PROMPT_MESSAGE == '':
				print("Waiting from the microphone input data")
				rospy.Rate(10)
			self.dm.send_audioToAWS_client(PROMPT_MESSAGE)
		self._feedback.action = goal_name
		self._feedback.state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not DIALOGUE_PROCESSING_DONE:
			if self.action.is_preempt_requested():
					print("action preempted")
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			print("enter if success")
			self._result.do_continue = INTERACTION_CONTINUE
			self._result.action = goal_name
			self._result.message = INTERACTION_MESSAGE
			PROMPT_MESSAGE = ''
			DIALOGUE_PROCESSING_DONE = False
			FEEDBACK_MESSAGE = ''
			INTERACTION_MESSAGE = ''
			INTERACTION_CONTINUE = True
			self.action.set_succeeded(self._result)


class DialogueManager():
	def __init__(self):
	   	rospy.Subscriber('/cordial/microphone/audio', AudioData, self.handle_audio_data, queue_size=1)
		self.text_publisher = rospy.Publisher('cordial/dialogue/script', String, queue_size=1)
		#self.send_textToAWS("Hey") # FOR TESTING
		

	def handle_audio_data(self, data):
		global PROMPT_MESSAGE
		PROMPT_MESSAGE = data.data
		#print("The audio prompt_messages are: ", PROMPT_MESSAGE)

	def handle_lex_response(self,lex_response):
		global INTERACTION_CONTINUE
		global INTERACTION_MESSAGE
		global DIALOGUE_PROCESSING_DONE
		if len(lex_response.text_response) > 0:
			print("The lex response is: ", lex_response)
			#When lex failed in understanding the user
			if lex_response.dialog_state == 'Failed':
				INTERACTION_MESSAGE = 'failed_understanding'
				INTERACTION_CONTINUE = False
				print("In Failed dialogue state")
			elif lex_response.dialog_state == 'Fulfilled':
				INTERACTION_MESSAGE = 'success'
				INTERACTION_CONTINUE = False
				print("In Fulfilled dialogue state, the response is:" + lex_response.text_response)
				self.text_publisher.publish(lex_response.text_response)
			else:
				print("In general dialogue state, the response is:" + lex_response.text_response)
				self.text_publisher.publish(lex_response.text_response)
			DIALOGUE_PROCESSING_DONE = True

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
		dm = DialogueManager()
		DialogueServer("dialoging", dm)
		rospy.spin()
    
    




