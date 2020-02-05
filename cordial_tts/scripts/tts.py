#!/usr/bin/env python

import rospy
import os
import sys
import pyaudio
import wave
import struct
import json
import time
import random
import numpy as np
import actionlib
from std_msgs.msg import String
from cordial_tts import CoRDialTTS
from cordial_behavior.msg import Behavior
from cordial_manager.msg import *
import soundfile as sf


WAV_HEADER_LENGTH = 24

DIALOGUE_MESSAGE = ''
SYNTHESIZE_DONE = False
FEEDBACK_MESSAGE = ''
INTERACTION_MESSAGE = ''
INTERACTION_CONTINUE = True


class SynthesizeServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name):
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		global DIALOGUE_MESSAGE, SYNTHESIZE_DONE, FEEDBACK_MESSAGE, INTERACTION_CONTINUE, INTERACTION_MESSAGE
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			DIALOGUE_MESSAGE = goal.optional_data
		while DIALOGUE_MESSAGE == '':
			print("Wait until dialogue message is updated")
			rospy.Rate(10)
		ttsm = TTSManager()
		ttsm.handle_tts_realtime(DIALOGUE_MESSAGE)
		print("The dialogue message is:" + DIALOGUE_MESSAGE)
		self._feedback.action = goal_name
		self._feedback.state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not SYNTHESIZE_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = INTERACTION_CONTINUE
			self._result.action = goal_name
			self._result.message = INTERACTION_MESSAGE
			DIALOGUE_MESSAGE = ''
			SYNTHESIZE_DONE = False
			FEEDBACK_MESSAGE = ''
			INTERACTION_MESSAGE = ''
			INTERACTION_CONTINUE = True
			self.action.set_succeeded(self._result)


class TTSManager():
	def __init__(self):
		
		rospy.Subscriber('cordial/dialogue/script', String, self.handle_dialogue_message)
		self.behavior_publisher = rospy.Publisher("cordial/behavior", Behavior, queue_size=10)
		#self.handle_tts_realtime("Hello *QT/bye* I am QT *happy_face*") #FOR TESTING
		

	def handle_dialogue_message(self, data):
		global DIALOGUE_MESSAGE
		DIALOGUE_MESSAGE = data.data
		print("TTS Heard from Lex: " + DIALOGUE_MESSAGE)

	def handle_tts_realtime(self, data):
		global SYNTHESIZE_DONE
		print("The TTS message received")
		#outdir = os.path.dirname(os.path.abspath("home		
		outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_dialogue/scripts/data"
		voice = "Ivy"	
		tts = CoRDialTTS(voice)
		phraseID = "1"
		text_content = data
		print("The text content is: " + text_content)
		#text_content = req #FOR TESTING
		file_saved = tts.phrase_to_file(phraseID, text_content, outdir)
		print("The output from the TTS is: ", file_saved)
		behaviours = sorted(file_saved["behaviors"], key = lambda i: i['start']) # sorting the behaviours
		path_audio_file = file_saved["file"] # path of audiofile.ogg
		data, samplerate = sf.read(outdir + '/'+phraseID+'.ogg')
		sf.write(outdir + '/'+phraseID+'.wav', data, samplerate)
		file_handle =outdir + '/'+phraseID+'.wav'
		data = np.fromfile(file_handle, np.uint8)[WAV_HEADER_LENGTH:] #Loading wav file
		data = data.astype(np.uint8).tostring()
		data_array = data
		audio_frame = samplerate
		behavior_msg = Behavior()
		behavior_msg.audio_frame = audio_frame
		behavior_msg.audio_data =  data_array
		behavior_msg.behavior_json = str(behaviours)
		print("The behavior message is sent")
		self.behavior_publisher.publish(behavior_msg)
		SYNTHESIZE_DONE = True

if __name__ == '__main__':
		rospy.init_node("tts_node", anonymous=True)
		TTSManager()
		SynthesizeServer("synthesizing")
		rospy.spin()
