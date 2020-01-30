#!/usr/bin/env python

import rospy
import sys
import actionlib
import json
from ast import literal_eval
from std_msgs.msg import String, Bool
from qt_robot_gestures.msg import Gesture
from cordial_face.msg import FaceRequest
from cordial_manager.msg import *
from qt_robot_speaker.msg import PlayRequest # Rename it! PlayAudio
from threading import Timer
import os


#data = "[{'start': 0.006, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 0.075, 'type': 'viseme', 'id': 'POSTALVEOLAR'}, {'start': 0.136, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 0.264, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 0.332, 'type': 'viseme', 'id': 'CLOSE_BACK_VOWEL'}, {'start': 0.383, 'type': 'viseme', 'id': 'VELAR_GLOTTAL'}, {'start': 0.463, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 0.535, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 0.571, 'type': 'viseme', 'id': 'MID_CENTRAL_VOWEL'}, {'start': 0.581, 'args': [], 'type': 'action', 'id': 'breath_face'}, {'start': 0.611, 'type': 'viseme', 'id': 'BILABIAL'}, {'start': 0.689, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 0.73, 'type': 'viseme', 'id': 'VELAR_GLOTTAL'}, {'start': 0.829, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 0.951, 'type': 'viseme', 'id': 'LABIODENTAL'}, {'start': 1.009, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 1.076, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 1.174, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 1.254, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.307, 'type': 'viseme', 'id': 'INTERDENTAL'}, {'start': 1.337, 'type': 'viseme', 'id': 'MID_CENTRAL_VOWEL'}, {'start': 1.368, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.3780000000000001, 'args': [], 'type': 'action', 'id': 'happy_face'}, {'start': 1.51, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 1.617, 'type': 'viseme', 'id': 'BILABIAL'}, {'start': 1.699, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.804, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 1.857, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.903, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.977, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 2.107, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 2.281, 'type': 'viseme', 'id': 'IDLE'}]"
TTS_MESSAGE = ''
DETECTOR_MESSAGE = ''
SPEAKER_DONE = False
TRACKER_DONE = False
FEEDBACK_MESSAGE = ''
ERROR_MESSAGE = ''

class LongBehaviorServer():
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
			DETECTOR_MESSAGE = optional_data
		BehaviorManager.handle_tracker(DETECTOR_MESSAGE)
		self._feedback.interacting_action = goal_name
		self._feedback.interacting_state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not TRACKER_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
		if success:
			self._result.interacting_success = True
			self._result.interacting_action = goal_name
			self._result.error_message = ERROR_MESSAGE
			self.action.set_succeeded(self._result)


class BehaviorServer():
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
			TTS_MESSAGE = optional_data
		BehaviorManager.handle_behavior(TTS_MESSAGE)
		self._feedback.interacting_action = goal_name
		self._feedback.interacting_state = FEEDBACK_MESSAGE
		## Decide when to send the feedback
		# self.action.publish_feedback(self._feedback)
		while not SPEAKER_DONE:
			if self.action.is_preempt_requested():
					self.action.set_preempted()
					success = False
		if success:
			self._result.interacting_success = True
			self._result.interacting_action = goal_name
			self._result.error_message = ERROR_MESSAGE
			self.action.set_succeeded(self._result)
		

class BehaviorManager():

	def __init__(self):
		rospy.init_node("behavior_node", anonymous=True)
		rospy.Subscriber('cordial/behavior', Behavior, self.handle_tts_message)
		rospy.Subscriber('cordial/speaker/done', Bool, self.handle_speaker_message)
		rospy.Subscriber('cordial/detector/faces', String, self.handle_detector_message) # I dont know if it is a string!
		rospy.Subscriber('cordial/behavior/tracking/done', Bool, self.handle_tracking_done) 
		self.face_publisher = rospy.Publisher('cordial/behavior/face/expressing', FaceRequest, queue_size=10)
		self.gesture_publisher = rospy.Publisher('cordial/behavior/gesture/moving', Gesture, queue_size=10)
		self.speaker_publisher = rospy.Publisher('cordial/speaker/playing', PlayRequest, queue_size=10)
		self.tracker_publisher = rospy.Publisher('cordial/behavior/tracking', Bool, queue_size=1)
		self.get_facial_expressions_list()
		#self.handle_behavior(data) #FOR TESTING
		rospy.spin()

	def handle_tracking_done(self, data):
		TRACKER_DONE = data.data

	def handle_tts_message(self, data):
		TTS_MESSAGE = data

			
	def handle_speaker_message(self, data):
		SPEAKER_DONE = data.data
		rospy.loginfo("The speaker has finished: " +str(data.data))

	def handle_detector_message(self, data):
		DETECTOR_MESSAGE = data

	def handle_tracker(self, data):
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
			print(str(aus), str(f['au_degrees']), str(au_ms))
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
	BehaviorManager()
	BehaviorServer("behaving")
	LongBehaviorServer("long_behaving")




