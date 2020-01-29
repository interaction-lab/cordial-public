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
from std_msgs.msg import String
from cordial_tts import CoRDialTTS
from cordial_behavior.msg import Behavior
import soundfile as sf


WAV_HEADER_LENGTH = 24

class TTSManager():
	def __init__(self):
		rospy.init_node("tts_node", anonymous=True)
		rospy.Subscriber('cordial/dialogue/script', String, self.handle_tts_realtime)
		self.behavior_publisher = rospy.Publisher("cordial/behavior", Behavior, queue_size=10)
		#self.handle_tts_realtime("Hello *QT/bye* I am QT *happy_face*") FOR TESTING
		rospy.spin()

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
		#self.play_publisher.publish(behavior_msg) -----OLD
		self.behavior_publisher.publish(behavior_msg)

if __name__ == '__main__':
    TTSManager()
