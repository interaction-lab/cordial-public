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


class AudioInput():
	def __init__(self,pi, nuc):
		#inizialize the node
		self.pi_topic = pi
		self.nuc_topic = nuc
		rospy.init_node("audio_node", anonymous=True)
		print("Subscribe the AudioData")
		#call the AudioConversationText service to send audio to Lex
	   	rospy.Subscriber(self.nuc_topic+"/microphone_input", AudioData, self.send_audioToAWS_client)
		rospy.Subscriber(self.nuc_topic+"/state", String, self.handle_state)
		rospy.spin()

	def handle_lex_response(self,lex_response):
		if len(lex_response.text_response) > 0:
			self.audio_output_publisher.publish(lex_response.text_response)

	def handle_state(self,request):
		state = request.data
		print("The state is " + state)
		## MANAGE THE STATES : GESTURES AND FACE EXPRESSIONS!! The states are: Listening, Finished listening, Idle


	def send_audioToAWS_client(self,audiodatarequest):
		#initialize the publisher to send response to be played from tts
		print("Publish the ResponseAudio")
		self.audio_output_publisher = rospy.Publisher(self.nuc_topic+"/lex/audio_output", String, queue_size = 10)
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

	
	

if __name__ == '__main__':
	pi =  "qt_robot"
	nuc = "qtpc"
    	AudioInput(pi, nuc)
    
    




