#!/usr/bin/env python

import rospy
import sys
import actionlib
import json
from ast import literal_eval
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from cordial_behavior.msg import Behavior
from qt_robot_gestures.msg import Gesture
from cordial_face.msg import FaceRequest
from cordial_manager.msg import *
from qt_nuc_speaker.msg import PlayRequest # Rename it! PlayAudio
from threading import Timer
import os
import numpy

RATE_FEEDBACK = 10
#data = "[{'start': 0.075, 'time': 2,'type': 'action', 'id': 'QT/bye'},{'start': 0.075, 'time': 2,'type': 'action', 'id': 'QT/point_front'}, {'start': 0.075,'time': 2, 'type': 'viseme', 'id': 'POSTALVEOLAR'},{'start': 0.006, 'time': 2,  'type': 'action', 'id': 'happy_face'}]"

class LongBehaviorServer():
	_feedback = CordialFeedback()
	_result = CordialResult()

	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.tracker_info = ''
		self.controller_manager.detector_state = ''
		self.controller_manager.tracker_done = False
		self.controller_manager.long_interaction_message = ''
		self.controller_manager.long_interaction_continue = False
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.action
		success = True
		# Starting the tracking
		if goal.optional_data != '':
			if goal.optional_data == "tracking_face":
				self.controller_manager.tracker_info = goal.optional_data
		self.controller_manager.handle_long_behavior(self.controller_manager.tracker_info)
		while self.controller_manager.detector_state == '':
			rospy.logdebug("Waiting state from tracker")
			rospy.Rate(RATE_FEEDBACK)
		self._feedback.action = goal_name
		self._feedback.state = self.controller_manager.detector_state #Long Behavior state
		
		while not self.controller_manager.tracker_done:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			self.action.publish_feedback(self._feedback)
			rospy.Rate(20)

		if success:
			self._result.do_continue = self.controller_manager.long_interaction_continue
			self._result.action = goal_name
			self._result.message = self.controller_manager.long_interaction_message
			print(self._result)
			self.controller_manager.tracker_info = ''
			self.controller_manager.detector_state = ''
			self.controller_manager.tracker_done = False
			self.controller_manager.long_interaction_message = ''
			self.controller_manager.long_interaction_continue = False
			self.action.set_succeeded(self._result)


class BehaviorServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.controller_manager = manager
		self.controller_manager.all_behaviors_done = False
		self.controller_manager.tts_message = ''
		self.controller_manager.interaction_message = ''
		self.controller_manager.interaction_continue = True
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			if goal.optional_data == "userlost":
				self.controller_manager.user_lost_override = True
			elif goal.optional_data == "endblock": ##TODO: do that directly from the block. Stop all the nodes and children! Not from here!
				self.controller_manager.end_block = True
		self.controller_manager.handle_behavior(self.controller_manager.tts_message)
		self._feedback.action = goal_name
		#self._feedback.state = behavior state
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not self.controller_manager.all_behaviors_done:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(RATE_FEEDBACK)
		print("Server behavior sending a response")
		if success:
			self._result.do_continue = self.controller_manager.interaction_continue
			self._result.action = goal_name
			self._result.message = self.controller_manager.interaction_message
			self.controller_manager.all_behaviors_done = False
			self.controller_manager.tts_message = ''
			self.controller_manager.interaction_message = ''
			self.controller_manager.interaction_continue = True
			self.controller_manager.user_lost_override = False
			self.action.set_succeeded(self._result)
		

class BehaviorManager():

	def __init__(self):
		# These variables are not yer declared in the Controller Manager, but they could be useful 
		self.tracker_done = False
		self.tracker_info = ""
		self.tracker_state = False
		self.detector_state = ""
		self.long_interaction_message = ""
		self.long_interaction_continue = False
		self.tts_message = ""
		self.all_behaviors_done = False
		self.interaction_message = ""
		self.interaction_continue = True

		# Declare subscribers and publishers
		rospy.Subscriber('cordial/behavior', Behavior, self.handle_tts_message, queue_size=1)
		rospy.Subscriber('cordial/speaker/done', Bool, self.handle_speaker_message, queue_size=1)
		rospy.Subscriber('cordial/behavior/gesture/done', Bool, self.handle_gestures_message, queue_size=1)
		rospy.Subscriber('/cordial/detector/faces', Point, self.handle_detector_message, queue_size=1) 
		rospy.Subscriber('cordial/behavior/tracking/state', Bool, self.handle_tracking_state, queue_size=1) # States: NO_TRACK, ACQUIRING, FOUND, LOST 
		self.face_publisher = rospy.Publisher('cordial/behavior/face/expressing', FaceRequest, queue_size=1)
		self.gesture_publisher = rospy.Publisher('cordial/behavior/gesture/moving', Gesture, queue_size=1)
		self.speaker_publisher = rospy.Publisher('cordial/speaker/playing', PlayRequest, queue_size=1)
		self.tracker_publisher = rospy.Publisher('cordial/behavior/tracking', Bool, queue_size=1)
		self.get_facial_expressions_list()
		self.is_behavior_done = False
		self.speaker_done = False
		self.gesture_done = False
		self.facial_expression_done = False
		self.user_lost = True
		self.user_lost_override = False
		self.end_block = False
		

	def handle_tracking_state(self, data):
		# is TRUE if the human is presente, is FALSE if the user is lost
		self.tracking_state = data.data
		rospy.logdebug("The tracker has updated the state" + str(self.tracking_state))
		if self.tracking_state:
			self.user_lost = False
		elif not self.tracking_state:
			self.user_lost = True
		rospy.logdebug("The user is lost: " + str(self.user_lost))

	def handle_tts_message(self, data):
		self.tts_message = data
		print("The message is received from the TTS")
		#self.handle_behavior(data) # FOR TESTING

	def handle_gestures_message(self, data):
		self.gesture_done = data.data
		if self.gesture_done:
			rospy.loginfo("The gesture has finished")


	def handle_speaker_message(self, data):
		self.speaker_done = data.data
		if self.speaker_done:
			rospy.loginfo("The speaker has finished")

	def handle_detector_message(self, data):
		#rospy.loginfo("I received the message from the detector!!")
		self.detector_state = "FOUND"

	def handle_long_behavior(self, data):
		if data == "tracking_face":
			self.tracker_publisher.publish(True) 

	def check_if_behavior_done(self):
		if self.gesture_done and self.speaker_done:
			self.is_behavior_done = True
			rospy.loginfo("Single behavior done")
			self.gesture_done = False
			self.speaker_done = False
		return

	def get_facial_expressions_list(self):
		self.facial_expression_list = []
		self.face_expression_au = []
		base_dir = os.path.dirname(__file__)
		with open(base_dir + '/default_name/cordial_face_expression.json', "r") as json_file:
			data = json.load(json_file)
			for facial_expression in data:
				self.facial_expression_list.append(facial_expression)
				au_name = str(facial_expression)
				aus = []
				for dofs in data[facial_expression]['dofs']:
					aus.append(str(dofs))
				for keyframe in data[facial_expression]['keyframes']:
					au_degrees = keyframe['pose']
					au_ms = keyframe['time']
					self.face_expression_au.append({ "au_name": au_name,
								"aus": aus, 
								"au_degrees": au_degrees,
								"au_ms": au_ms})

	def handle_behavior(self, data):
		if data != "":
			audio_frame_array = data.audio_frame
			audio_data_array = data.audio_data
			data_array = data.behavior_json
			for index in range(len(audio_frame_array)):
				audio_frame = audio_frame_array[index]
				audio_data = audio_data_array[index] 
				data = literal_eval(data_array[index])
				word_timing = filter(lambda b: b["type"] == "word", data)
				behav = filter(lambda b: b["type"] != "word", data)
				facial_expression_list = self.facial_expression_list
				visemes = ["BILABIAL","LABIODENTAL","INTERDENTAL","DENTAL_ALVEOLAR","POSTALVEOLAR","VELAR_GLOTTAL","CLOSE_FRONT_VOWEL","OPEN_FRONT_VOWEL","MID_CENTRAL_VOWEL","OPEN_BACK_VOWEL","CLOSE_BACK_VOWEL", 'IDLE']
				all_gesture_behaviors = filter(lambda b: b["id"] not in visemes, behav)
				gesture_behaviors = filter(lambda b: b["id"] not in facial_expression_list, all_gesture_behaviors)
				viseme_behaviors = filter(lambda b: b["id"] in visemes, behav)
				facial_expression_behaviors = filter(lambda b: b["id"] in facial_expression_list, behav)
				if not index == 0: 
					while not self.is_behavior_done:
						self.check_if_behavior_done()
				r = rospy.Rate(10)
				counter = 0
				while self.user_lost and not self.user_lost_override:
					counter += 1
					rospy.loginfo("User is lost!! Do something " + str(counter))
					r.sleep()
					if counter > 40:
						self.interaction_message = "failure_userlost"
						self.interaction_continue = False
						break
				if not self.user_lost or (self.user_lost and (self.user_lost_override or self.end_block)):
					self.is_behavior_done = False
					self.gesture_done = False
					self.speaker_done = False
					self.handle_audio(audio_data, audio_frame)
					self.handle_visemes(viseme_behaviors)
					self.handle_gestures(gesture_behaviors, word_timing)
					self.handle_facial_expression(facial_expression_behaviors)
				else:
					break
			while not self.is_behavior_done:
				self.check_if_behavior_done()
			self.all_behaviors_done = True
			rospy.loginfo("All behaviors done")
	

	def handle_audio(self, audio_data, audio_frame):
		speaker_msg = PlayRequest()
		speaker_msg.audio_frame = audio_frame
		speaker_msg.data = audio_data
		print("The speaker message is published")
		self.speaker_publisher.publish(speaker_msg)


	def handle_facial_expression(self, facial_expression_behaviors):
		ordered_facial_behaviors = sorted(facial_expression_behaviors, key=lambda behavior: behavior["start"])
		validated_face_expr = []
		for au in self.face_expression_au:
			for fexp in  ordered_facial_behaviors:
				if au['au_name'] == fexp['id']:
					validated_face_expr.append(au)
		if validated_face_expr == []:
			self.facial_expression_done = True
			rospy.loginfo("No facial expression")
		for f in validated_face_expr:
			aus = map(lambda s: s[2:], f['aus'])
			au_ms = f['au_ms']*1000
			#print(str(aus), str(f['au_degrees']), str(au_ms))
			f_expr_req = FaceRequest(aus=aus, au_degrees=f['au_degrees'], au_ms=au_ms)
			def send_facerequest():
				print("Send face request")
				self.face_publisher.publish(f_expr_req)
			rospy.sleep(0.5) #TODO: change that!
			t = Timer(0.1, send_facerequest)
			t.start()
			start_time = rospy.Time.now()

		
	def handle_gestures(self, gesture_behaviors, word_timing):
		#Handle Gestures
		ordered_behaviors = sorted(gesture_behaviors, key=lambda behavior: behavior["start"])
		if ordered_behaviors == []:
			rospy.loginfo("No gestures")
			self.gesture_done = True
		timing_word_behaviors = word_timing + gesture_behaviors
		ordered_timing_word_behaviors = sorted(timing_word_behaviors, key=lambda behavior: behavior["start"]) # I dont know why is not sorted!!!!! CHECK IT
		start_time = rospy.Time.now()
		for index, behav in enumerate(ordered_timing_word_behaviors[:-1]):
			if behav["type"] != "word":
				while rospy.Time.now()-start_time < rospy.Duration.from_sec(behav["start"]):
					pass
				gesture_timing = float(ordered_timing_word_behaviors[index + 1]["time"])
				rospy.loginfo("Play " + str(behav["id"]) + " at time:" + str(behav["start"]) + " with a duration of: " + str(gesture_timing))
				self.gesture_publisher.publish(gesture_timing, behav["id"])

	def handle_visemes(self, viseme_behaviors):
		min_duration=0.05
		for i in range(0,len(viseme_behaviors)-1):
		        viseme_behaviors[i]["duration"]=viseme_behaviors[i+1]["start"]-viseme_behaviors[i]["start"]
		viseme_behaviors[-1]["duration"]=min_duration
		viseme_behaviors=filter(lambda b: b["duration"]>= min_duration, viseme_behaviors)

		ordered_visemes = sorted(viseme_behaviors, key=lambda b: b["start"])
		viseme_ids = map(lambda b: b["id"], ordered_visemes)
		viseme_times = map(lambda b: b["start"], ordered_visemes)
		viseme_speed = 10
		#timing_adjust = rospy.Duration.from_sec(0)
		viseme_req = FaceRequest(visemes=viseme_ids, viseme_ms=viseme_speed, times=viseme_times)
		#viseme_delay_time = timing_adjust.to_sec()
		def send_visemes():
			print("Send viseme")
			self.face_publisher.publish(viseme_req)
		t = Timer(0.01, send_visemes)
		t.start()
		start_time = rospy.Time.now()



if __name__ == '__main__':
	rospy.init_node("behavior_node", anonymous=True)
	controller_manager = BehaviorManager()
	BehaviorServer("behaving", controller_manager)
	LongBehaviorServer("long_behaving",controller_manager)
	rospy.spin()




