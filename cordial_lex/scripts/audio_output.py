#!/usr/bin/env python

import sys
import pyaudio
import wave
import struct
import rospy
import actionlib
import json
import time
import random
from std_msgs.msg import String
from cordial_tts import CoRDialTTS



class AudioOutput():
	
	def __init__(self, pi, nuc):
		self.pi_topic = pi
		self.nuc_topic = nuc
		rospy.init_node("audio_node", anonymous=True)
		print("Subscribe the AudioData from the response")
		# Subscriber received the response text from LEX
		rospy.Subscriber(self.nuc_topic+"/lex/audio_output", String, self.handle_tts)
		self.text_publisher = rospy.Publisher(self.nuc_topic+"/text_output", String, queue_size=10)
		rospy.spin()


	def handle_tts(self,request):
		## Send Text to TTS service --> Cordial TTS
		text_content = request.data
		self.text_publisher.publish(text_content)
		return


	

if __name__ == '__main__':
	pi =  "qt_robot"
	nuc = "qtpc"
    	AudioOutput(pi, nuc)
