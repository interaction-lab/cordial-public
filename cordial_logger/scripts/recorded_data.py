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
from qt_nuitrack_app.msg import Skeletons
from datetime import datetime
import wave
import pyaudio
import ast

CHANNEL = 1
SAMPLERATE = 16000
FORMAT_SIZE = pyaudio.paInt16
CHUNK_SIZE = 1024
FPS = 23

class RecordingManager():

	def __init__(self):
		self.recorded_audio_common_frames = []
		self.recorded_video_frames = []
		self.recorded_video_head_frames = []
		self.script_data = []
		self.first_video_frame = True
		self.first_video_head_frame = True
		self.is_video_recording = False
		self.is_audio_recording = False
		self.is_data_recording = False
		self.cv_bridge = CvBridge()
		self.fps = FPS
		rospy.Subscriber("/cordial/chest_camera/video", Image, self.handle_recorded_video, queue_size=1)
		rospy.Subscriber("/audio/channel0", AudioData, self.handle_recorded_audio_common, queue_size=1)
		rospy.Subscriber("/cordial/dialogue/data", String, self.handle_user_prompt, queue_size=1)
		#audio direction
		rospy.Subscriber("/camera/color/image_raw", Image, self.handle_recorded_video_head, queue_size=1)
		rospy.Subscriber("/qt_nuitrack_app/skeletons", Skeletons, self.handle_recorded_skeleton_position, queue_size=1)

		rospy.Subscriber("/cordial/recording/audio", Bool, self.handle_trigger_recorded_audio, queue_size=1)
		rospy.Subscriber("/cordial/recording/video", Bool, self.handle_trigger_recorded_video, queue_size=1)
		rospy.Subscriber("/cordial/recording/data", Bool, self.handle_trigger_recorded_data, queue_size=1)
		

	def handle_recorded_skeleton_position(self, data):
		return
	
	def handle_user_prompt(self,data):
		if self.is_data_recording:
			transcript = ast.literal_eval(data.data)
			self.script_data.append(transcript)
		return

	def handle_trigger_recorded_data(self, data):
		if not data.data:
			self.is_data_recording = False
			file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
			outdir = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data/logfile/script/"
			with open(outdir + file_name+'.json', 'w') as outfile:
				json.dump(self.script_data, outfile)
		else:
			self.is_data_recording = True


	def handle_recorded_video_head(self,data):
		if self.is_video_recording:
			if self.first_video_head_frame:
				print("First frame")
				# Create the writer for the video
				fourcc = cv2.VideoWriter_fourcc(*'MJPG')
				file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') 
				path = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data/video_head/"
				fps = FPS
				self.video_head_data = cv2.VideoWriter(path + file_name + ".avi", fourcc , fps, (data.width, data.height), True)
				frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
				self.first_video_head_frame = False
			else:
				frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
				cv2.imshow("QTHeadCamVideo", frame)
			try:
				self.recorded_video_head_frames.append(frame)
			except:
				print("Not writing!!")

	def handle_trigger_recorded_video(self, data):
		if not data.data:
			print("Stop video recording")
			for frame in self.recorded_video_frames:
				self.video_data.write(frame)
			for frame in self.recorded_video_head_frames:
				self.video_head_data.write(frame)
			self.is_video_recording = False
			self.video_data.release()
			self.video_head_data.release()
			cv2.destroyAllWindows()
		elif data.data:
			self.is_video_recording = True


	def handle_recorded_video(self, data):
		if self.is_video_recording:
			if self.first_video_frame:
				print("First frame")
				# Create the writer for the video
				#self.fps = data.header.stamp.secs
				print("Is recording")
				fps = FPS
				fourcc = cv2.VideoWriter_fourcc(*'MJPG')
				file_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') 
				path = "/home/qtrobot/catkin_ws/src/cordial-public/cordial_logger/scripts/data/video/"
				self.video_data = cv2.VideoWriter(path + file_name + ".avi", fourcc , fps, (data.width, data.height), True)
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
				self.first_video_frame = False
			
			else:
				frame = self.cv_bridge.imgmsg_to_cv2(data, "rgb8")
				print("Is recording")
			try:
				self.recorded_video_frames.append(frame)
			except:
				print("Not writing!!")

	def handle_recorded_audio_common(self,data):
		self.recorded_audio_common_frames.append(data.data)

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
			wf.writeframes(b''.join(self.recorded_audio_common_frames))
			wf.close()
		elif data.data:
			self.is_audio_recording = True




if __name__ == '__main__':
	rospy.init_node("recorded_node", anonymous=True)
	controller_manager = RecordingManager()
	rospy.spin()