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
	_feedback = InteractionFeedback()
	_result = InteractionResult()
	def __init__(self, name):
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, InteractionAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.interacting_action
		success = True
		if goal.optional_data != '':
			DIALOGUE_MESSAGE = optional_data
		TTSManager.handle_tts_realtime(DIALOGUE_MESSAGE)
		self._feedback.interacting_action = goal_name
		self._feedback.interacting_state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not SYNTHESIZE_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
		if success:
			self._result.interaction_continue = INTERACTION_CONTINUE
			self._result.interacting_action = goal_name
			self._result.message = INTERACTION_MESSAGE
			self.action.set_succeeded(self._result)


class TTSManager():
	def __init__(self):
		
		rospy.Subscriber('cordial/dialogue/script', String, self.handle_dialogue_message)
		self.behavior_publisher = rospy.Publisher("cordial/behavior", Behavior, queue_size=10)
		#self.handle_tts_realtime("Hello *QT/bye* I am QT *happy_face*") FOR TESTING
		

	def handle_dialogue_message(self, data):
		DIALOGUE_MESSAGE = data

	def handle_tts_realtime(self, req):
		#outdir = os.path.dirname(os.path.abspath("home																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							/qtrobot/catkin_ws/cordial-public/cordial_lex/script/data/"))
		outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_dialogue/scripts/data"
		voice = "Ivy"	
		tts = CoRDialTTS(voice)
		phraseID = "1"
		text_content = req.data
		#text_content = req #FOR TESTING
		file_saved = tts.phrase_to_file(phraseID, text_content, outdir)
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
		self.behavior_publisher.publish(behavior_msg)
		SYNTHESIZE_DONE = True

if __name__ == '__main__':
	rospy.init_node("tts_node", anonymous=True)
    TTSManager()
	SynthesizeServer("synthesizing")
	rospy.spin()
