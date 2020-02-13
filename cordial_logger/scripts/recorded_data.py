#!/usr/bin/env python

import rospy
import os
import sys
import json
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from audio_common_msgs.msg import AudioData
import wave
import pyaudio

CHANNEL = 1
SAMPLERATE = 16000
FORMAT_SIZE = pyaudio.paInt16

class RecordingManager():

	def __init__(self):
		self.recorded_audio_frames = []
		self.recorded_video_frames = []
		self.first_video_frame = True
		self.is_video_recording = True
		self.is_audio_recording = True
		self.cv_bridge = CvBridge()
		rospy.Subscriber("/cordial/chest_camera/video", Image, self.handle_recorded_video, queue_size=1)
		rospy.Subscriber("/cordial/recording/audio/data", AudioData, self.handle_recorded_audio, queue_size=1)
		rospy.Subscriber("/cordial/recording/audio", Bool, self.handle_trigger_recorded_audio, queue_size=1)
		rospy.Subscriber("/cordial/recording/video", Bool, self.handle_trigger_recorded_video, queue_size=1)
		

	def handle_trigger_recorded_video(self, data):
		if not data.data:
			print("Stop video recording")
			for frame in self.recorded_video_frames:
				self.video_data.write(frame)
			self.is_video_recording = False
			self.video_data.release()
			cv2.destroyAllWindows()



	def handle_recorded_video(self, data):
		if self.is_video_recording:
			if self.first_video_frame:
				print("First frame")
				fps = data.header.stamp.secs
				#fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
				fourcc = cv2.VideoWriter_fourcc(*'MJPG')
				path = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data/OutPut.avi"
				self.video_data = cv2.VideoWriter(path, fourcc , fps, (data.width, data.height), True)
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
				self.first_video_frame = False
			else:
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
			try:
				print("Writing")
				self.recorded_video_frames.append(frame)
			except:
				print("Not writing!!")


	def handle_recorded_audio(self,data):
		self.recorded_audio_frames.append(data.data)

	def handle_trigger_recorded_audio(self, data):
		if not data.data:
			self.is_audio_recording = False
			p = pyaudio.PyAudio()
			recorded_ID = "test"
			outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data"
			wf = wave.open(outdir + "/"+ recorded_ID + ".wav", 'wb')
			wf.setnchannels(CHANNEL)
			wf.setsampwidth(p.get_sample_size(FORMAT_SIZE))
			wf.setframerate(SAMPLERATE)
			wf.writeframes(b''.join(self.recorded_frames))
			wf.close()




if __name__ == '__main__':
	rospy.init_node("recorded_node", anonymous=True)
	controller_manager = RecordingManager()
	rospy.spin()