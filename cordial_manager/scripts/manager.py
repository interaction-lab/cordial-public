#!/usr/bin/env python

import rospy
import sys
from lex_common_msgs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData
from qt_robot_speaker.msg import PlayRequest

class InteractionManager():
	
	def __init__(self, pi, nuc):
		"""
		Subscriber: topic names are chosen according to where the message comes from (nuc or pi).
		Publisher: topic names are chosen according to where the message is addressed (nuc or pi).
		"""
		self.pi_topic = pi
		self.nuc_topic = nuc
		rospy.init_node("manager_node", anonymous=True)
		rospy.Subscriber(self.pi_topic+"/microphone_input", AudioData, self.handle_microphone_input)
		rospy.Subscriber(self.pi_topic+"/state", String, self.handle_state)
		rospy.Subscriber(self.nuc_topic+"/text_output", String, self.handle_text_output)
		rospy.Subscriber(self.nuc_topic+"/gestures", String, self.handle_gestures)
		rospy.Subscriber(self.nuc_topic+"/behavior", String, self.handle_behavior)
		rospy.Subscriber(self.nuc_topic+"/face", String, self.handle_face)
		rospy.Subscriber(self.nuc_topic+"/speaker_output/play", PlayRequest, self.handle_speaker_play)
		rospy.Subscriber(self.pi_topic+"/speaker_state", Bool, self.handle_speaker_state)
		self.gestures_pub = rospy.Publisher(self.pi_topic+"/gestures", String, queue_size=1)
		self.microphone_input_pub = rospy.Publisher(self.nuc_topic+"/microphone_input", AudioData, queue_size = 5)
		self.speaker_output = rospy.Publisher(self.pi_topic+"/speaker_output/play", PlayRequest, queue_size = 5)
		self.state_pub = rospy.Publisher(self.nuc_topic+"/state", String, queue_size = 10)
		self.speaker_state_pub = rospy.Publisher(self.nuc_topic+"/speaker_state", Bool, queue_size = 10)
		self.text_output_pub = rospy.Publisher(self.nuc_topic+"/tts/text_output", String, queue_size = 10)
		self.behavior_pub = rospy.Publisher(self.pi_topic+"/behavior", String, queue_size = 10)
		self.face_pub = rospy.Publisher(self.pi_topic+"/face", String, queue_size = 10)
		rospy.spin()


	def handle_gestures(self, req):
		self.gestures_pub.publish(req.data)
		return


	def handle_microphone_input(self, req):
		self.microphone_input_pub.publish(req.data)
		return

	def handle_speaker_play(self, req):
	    print(req.audio_frame)
	    self.speaker_output.publish(req.audio_frame, req.data)
	    return
	
	def handle_speaker_state(self, req):
	    self.speaker_state_pub.publish(req.data)
	    return

	
	def handle_state(self, req):
		self.state_pub.publish(req.data)
		return

	def handle_text_output(self, req):
		self.text_output_pub.publish(req.data)
		return

	def handle_behavior(self, req):
		self.behavior_pub.publish(req.data)
		return

	def handle_face(self, req):
		self.face_pub.publish(req.data)
		return
	
if __name__ == '__main__':
	pi =  "qt_robot"
	nuc = "qtpc"
    	InteractionManager(pi, nuc)

