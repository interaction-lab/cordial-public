#!/usr/bin/env python
from sys import byteorder
from array import array
from struct import pack

import sys
import pyaudio
import wave
import struct
import rospy
from lex_common_msgs.srv import *
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData


class DialogueManager():
	def __init__(self):
		rospy.init_node("dialogue_node", anonymous=True)
	   	rospy.Subscriber('/cordial/microphone/audio', AudioData, self.send_audioToAWS_client)
		rospy.Subscriber('/cordial/dialogue/dialoging', String, self.send_textToAWS)
		self.text_publisher = rospy.Publisher('cordial/dialogue/script', String, queue_size=10)
		self.send_textToAWS("Hey") # FOR TESTING
		rospy.spin()

	def handle_lex_response(self,lex_response):
		if len(lex_response.text_response) > 0:
			self.text_publisher.publish(lex_response.text_response)

	def send_audioToAWS_client(self,audiodatarequest):
		print("Starting client request..")
		audiodata = audiodatarequest.data
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
    DialogueManager()
    
    




