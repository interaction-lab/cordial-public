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
from qt_robot_speaker.msg import PlayRequest
from datetime import datetime
import wave
import pyaudio
import base64

CHANNEL = 1
SAMPLERATE = 16000
FORMAT_SIZE = pyaudio.paInt16
CHUNK_SIZE = 1024

class RecordingManager():

	def __init__(self):
		self.recorded_audio_frames = []
		self.recorded_video_frames = []
		self.first_video_frame = True
		self.is_video_recording = True
		self.is_audio_recording = True
		self.cv_bridge = CvBridge()
		rospy.Subscriber("/cordial/chest_camera/video", Image, self.handle_recorded_video, queue_size=1)
		#rospy.Subscriber("/cordial/recording/audio/data", PlayRequest, self.handle_recorded_audio, queue_size=1)
		rospy.Subscriber("/audio", AudioData, self.handle_recorded_audio_common, queue_size=1)
		#rospy.Subscriber("/cordial/dialogue/data", String, self.handle_user_prompt, queue_size=1)
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
			#self.text_file.close()

	def handle_recorded_video(self, data):
		if self.is_video_recording:
			if self.first_video_frame:
				print("First frame")
				# Create the writer for the video
				fps = data.header.stamp.secs
				fourcc = cv2.VideoWriter_fourcc(*'MJPG')
				file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') 
				path = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data/video/"
				self.video_data = cv2.VideoWriter(path + file_name + ".avi", fourcc , fps, (data.width, data.height), True)
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
				self.first_video_frame = False
				# Create the txt file to store the data
				text_file = open(path+file_name+'.txt','wb')
				text_file.write("Start")
				text_file.close()
				self.text_file = open(path+file_name+'.txt','a')
			else:
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
			try:
				self.recorded_video_frames.append(frame)
				retval, buffer = cv2.imencode('.jpg', frame)
				jpg_as_text = base64.b64encode(buffer)
				#self.text_file.write('\n Frame:' + str(jpg_as_text))
			except:
				print("Not writing!!")


	def handle_recorded_audio(self,data):
		self.recorded_audio_frames.append(data.data)
		self.sample_width = data.audio_frame


	def handle_recorded_audio_common(self,data):
		#print(data.data)
		#P = 2
		#S = data.data
		#result = [x * P for x in S]
		#print(result)
		self.recorded_audio_frames.append(data.data)
		#self.sample_width = data.audio_frame

	def handle_trigger_recorded_audio(self, data):
		if not data.data:
			self.is_audio_recording = False
			p = pyaudio.PyAudio()
			file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
			outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data/audio/"
			wf = wave.open(outdir + "/"+ file_name + ".wav", 'wb')
			wf.setnchannels(CHANNEL)
			wf.setsampwidth(p.get_sample_size(FORMAT_SIZE))
			wf.setframerate(SAMPLERATE)
			wf.setnframes(CHUNK_SIZE)
			wf.writeframes(b''.join(self.recorded_audio_frames))
			wf.close()




if __name__ == '__main__':
	rospy.init_node("recorded_node", anonymous=True)
	controller_manager = RecordingManager()
	rospy.spin()