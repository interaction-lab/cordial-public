#!/usr/bin/env python

import rospy
import sys
import actionlib
import json
from ast import literal_eval
from std_msgs.msg import String, Bool
from cordial_behavior.msg import Behavior
from qt_robot_gestures.msg import Gesture
from cordial_face.msg import FaceRequest
from cordial_manager.msg import *
from qt_nuc_speaker.msg import PlayRequest # Rename it! PlayAudio
from threading import Timer
import os

RATE_FEEDBACK =""
#data = "[{'start': 0.075, 'time': 2,'type': 'action', 'id': 'QT/bye'},{'start': 0.075, 'time': 2,'type': 'action', 'id': 'QT/point_front'}, {'start': 0.075,'time': 2, 'type': 'viseme', 'id': 'POSTALVEOLAR'},{'start': 0.006, 'time': 2,  'type': 'action', 'id': 'happy_face'}]"
TTS_MESSAGE = ''
SPEAKER_DONE = False

DETECTOR_STATE = ''
DETECTOR_DONE = False
TRACKER_MESSAGE = ''


FEEDBACK_MESSAGE = ''
INTERACTION_MESSAGE = ''
INTERACTION_CONTINUE = True

class LongBehaviorServer():
	_feedback = CordialFeedback()
	_result = CordialResult()

	def __init__(self, name, manager):
		self.bm = manager
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		global TRACKER_MESSAGE , DETECTOR_STATE, FEEDBACK_MESSAGE, DETECTOR_DONE
		goal_name = goal.action
		success = True
		# Starting the tracking
		if goal.optional_data != '':
			TRACKER_MESSAGE = goal.optional_data
		self.bm.handle_long_behavior(TRACKER_MESSAGE)
		while DETECTOR_STATE == '':
			print("Waiting state from tracker")
			rospy.Rate(10)
		FEEDBACK_MESSAGE = DETECTOR_STATE
		self._feedback.action = goal_name
		self._feedback.state = FEEDBACK_MESSAGE
		while not DETECTOR_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			self.action.publish_feedback(self._feedback)
			rospy.Rate(10)

		if success:
			self._result.do_continue = INTERACTION_CONTINUE
			self._result.action = goal_name
			self._result.message = INTERACTION_MESSAGE
			TRACKER_MESSAGE = ''
			DETECTOR_STATE = ''
			DETECTOR_DONE = False
			FEEDBACK_MESSAGE = ''
			INTERACTION_MESSAGE = ''
			INTERACTION_CONTINUE = True
			self.action.set_succeeded(self._result)


class BehaviorServer():
	_feedback = CordialFeedback()
	_result = CordialResult()
	def __init__(self, name, manager):
		self.bm = manager
		self.action_name = name
		self.action = actionlib.SimpleActionServer(self.action_name, CordialAction, self.execute_goal, False)
		self.action.start()

	def execute_goal(self, goal):
		global TTS_MESSAGE, SPEAKER_DONE
		goal_name = goal.action
		success = True
		if goal.optional_data != '':
			TTS_MESSAGE = goal.optional_data
		self.bm.handle_behavior(TTS_MESSAGE)
		self._feedback.action = goal_name
		self._feedback.state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not SPEAKER_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
			rospy.Rate(10)
		if success:
			self._result.do_continue = INTERACTION_CONTINUE
			self._result.action = goal_name
			self._result.message = INTERACTION_MESSAGE
			SPEAKER_DONE = False
			TTS_MESSAGE = ''
			FEEDBACK_MESSAGE = ''
			INTERACTION_MESSAGE = ''
			INTERACTION_CONTINUE = True
			self.action.set_succeeded(self._result)
		

class BehaviorManager():

	def __init__(self):
		rospy.Subscriber('cordial/behavior', Behavior, self.handle_tts_message, queue_size=1)
		rospy.Subscriber('cordial/speaker/done', Bool, self.handle_speaker_message, queue_size=1)
		rospy.Subscriber('cordial/detector/faces', String, self.handle_detector_message, queue_size=1) # I dont know if it is a string!
		rospy.Subscriber('cordial/behavior/tracking/state', Bool, self.handle_tracking_state, queue_size=1) # States: NO_TRACK, ACQUIRING, FOUND, LOST 
		self.face_publisher = rospy.Publisher('cordial/behavior/face/expressing', FaceRequest, queue_size=1)
		self.gesture_publisher = rospy.Publisher('cordial/behavior/gesture/moving', Gesture, queue_size=1)
		self.speaker_publisher = rospy.Publisher('cordial/speaker/playing', PlayRequest, queue_size=1)
		self.tracker_publisher = rospy.Publisher('cordial/behavior/tracking', Bool, queue_size=1)
		self.get_facial_expressions_list()
		#self.handle_behavior(data) #FOR TESTING
		

	def handle_tracking_state(self, data):
		global DETECTOR_STATE
		DETECTOR_STATE = data.data

	def handle_tts_message(self, data):
		global TTS_MESSAGE
		TTS_MESSAGE = data
		print("The message is received from the TTS")

			
	def handle_speaker_message(self, data):
		global SPEAKER_DONE
		SPEAKER_DONE = data.data
		if SPEAKER_DONE:
			rospy.loginfo("The speaker has finished")

	def handle_detector_message(self, data):
		global DETECTOR_MESSAGE
		DETECTOR_MESSAGE = data

	def handle_long_behavior(self, data):
		if data == "tracking":
			self.tracker_publisher.publish(True)

		

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
		#print("The behaviors data are:", data)
		audio_frame = data.audio_frame
		audio_data = data.audio_data
		data = literal_eval(data.behavior_json)
		#data = literal_eval(data) #FOR TESTING
		word_timing = filter(lambda b: b["type"] == "word", data)
		behav = filter(lambda b: b["type"] != "word", data)
		facial_expression_list = self.facial_expression_list
		visemes = ["BILABIAL","LABIODENTAL","INTERDENTAL","DENTAL_ALVEOLAR","POSTALVEOLAR","VELAR_GLOTTAL","CLOSE_FRONT_VOWEL","OPEN_FRONT_VOWEL","MID_CENTRAL_VOWEL","OPEN_BACK_VOWEL","CLOSE_BACK_VOWEL", 'IDLE']
		all_gesture_behaviors = filter(lambda b: b["id"] not in visemes, behav)
		gesture_behaviors = filter(lambda b: b["id"] not in facial_expression_list, all_gesture_behaviors)
		viseme_behaviors = filter(lambda b: b["id"] in visemes, behav)
		facial_expression_behaviors = filter(lambda b: b["id"] in facial_expression_list, behav)
		self.handle_audio(audio_data, audio_frame)
		self.handle_visemes(viseme_behaviors)
		self.handle_gestures(gesture_behaviors, word_timing)
		self.handle_facial_expression(facial_expression_behaviors)

	def handle_audio(self, audio_data, audio_frame):
		speaker_msg = PlayRequest()
		speaker_msg.audio_frame = audio_frame
		speaker_msg.data = audio_data
		print("The speaker message is published")
		self.speaker_publisher.publish(speaker_msg)


	def handle_facial_expression(self, facial_expression_behaviors):
		rospy.loginfo(facial_expression_behaviors)
		ordered_facial_behaviors = sorted(facial_expression_behaviors, key=lambda behavior: behavior["start"])
		validated_face_expr = []
		for au in self.face_expression_au:
			for fexp in  ordered_facial_behaviors:
				if au['au_name'] == fexp['id']:
					validated_face_expr.append(au)

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
		timing_word_behaviors = word_timing + gesture_behaviors
		ordered_timing_word_behaviors = sorted(timing_word_behaviors, key=lambda behavior: behavior["start"]) # I dont know why is not sorted!!!!! CHECK IT
		start_time = rospy.Time.now()
		for index, behav in enumerate(ordered_timing_word_behaviors[:-1]):
			if behav["type"] != "word":
				while rospy.Time.now()-start_time < rospy.Duration.from_sec(behav["start"]):
					pass
				if index == len(ordered_timing_word_behaviors) - 1:
					gesture_timing = ordered_timing_word_behaviors[index +1 ]["time"]
				gesture_timing = float(ordered_timing_word_behaviors[index +1]["time"]) #you cannot have a behavior sets at the end of the sentence
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
	bm = BehaviorManager()
	BehaviorServer("behaving", bm)
	LongBehaviorServer("long_behaving",bm)
	rospy.spin()




