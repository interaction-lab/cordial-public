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
from qt_robot_speaker.msg import PlayRequest
from cordial_tts import CoRDialTTS
import soundfile as sf


WAV_HEADER_LENGTH = 24

class TTSManager():
	def __init__(self, pi, nuc):
		"""
		Subscriber: topic names are chosen according to where the message comes from (nuc or pi).
		Publisher: topic names are chosen according to where the message is addressed (nuc or pi).
		"""
		self.pi_topic = pi
		self.nuc_topic = nuc
		rospy.init_node("tts_node", anonymous=True)
		rospy.Subscriber(self.nuc_topic+"/tts/text_output", String, self.handle_tts_realtime)
		self.behavior_publisher = rospy.Publisher(self.nuc_topic+"/behavior", String, queue_size=10)
		self.play_publisher = rospy.Publisher(self.nuc_topic+"/speaker_output/play", PlayRequest, queue_size=10)
		rospy.spin()

	def handle_tts_realtime(self, req):
		#outdir = os.path.dirname(os.path.abspath("home																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							/qtrobot/catkin_ws/cordial-public/cordial_lex/script/data/"))
		outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_lex/scripts/data"
		voice = "Ivy"	
		tts = CoRDialTTS(voice)
		phraseID = "1"
		text_content = req.data
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
		speaker_msg = PlayRequest()
		speaker_msg.audio_frame = audio_frame
		speaker_msg.data =  data_array
		self.play_publisher.publish(speaker_msg)
		self.behavior_publisher.publish(str(behaviours))

if __name__ == '__main__':
    pi =  "qt_robot"
    nuc = "qtpc"
    TTSManager(pi, nuc)
