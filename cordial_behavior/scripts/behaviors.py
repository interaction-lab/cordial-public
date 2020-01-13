#!/usr/bin/env python

import rospy
import sys
import actionlib
import json
from ast import literal_eval
from std_msgs.msg import String
from cordial_behavior.msg import *
from cordial_face.msg import FaceRequest
from threading import Timer
"""
data = "[{'start': 0.006, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 0.075, 'type': 'viseme', 'id': 'POSTALVEOLAR'}, {'start': 0.136, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 0.264, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 0.332, 'type': 'viseme', 'id': 'CLOSE_BACK_VOWEL'}, {'start': 0.383, 'type': 'viseme', 'id': 'VELAR_GLOTTAL'}, {'start': 0.463, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 0.535, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 0.571, 'type': 'viseme', 'id': 'MID_CENTRAL_VOWEL'}, {'start': 0.581, 'args': [], 'type': 'action', 'id': 'behav1'}, {'start': 0.611, 'type': 'viseme', 'id': 'BILABIAL'}, {'start': 0.689, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 0.73, 'type': 'viseme', 'id': 'VELAR_GLOTTAL'}, {'start': 0.829, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 0.951, 'type': 'viseme', 'id': 'LABIODENTAL'}, {'start': 1.009, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 1.076, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 1.174, 'type': 'viseme', 'id': 'CLOSE_FRONT_VOWEL'}, {'start': 1.254, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.307, 'type': 'viseme', 'id': 'INTERDENTAL'}, {'start': 1.337, 'type': 'viseme', 'id': 'MID_CENTRAL_VOWEL'}, {'start': 1.368, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.3780000000000001, 'args': [], 'type': 'action', 'id': 'behav2'}, {'start': 1.51, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 1.617, 'type': 'viseme', 'id': 'BILABIAL'}, {'start': 1.699, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.804, 'type': 'viseme', 'id': 'OPEN_FRONT_VOWEL'}, {'start': 1.857, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.903, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 1.977, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 2.107, 'type': 'viseme', 'id': 'DENTAL_ALVEOLAR'}, {'start': 2.281, 'type': 'viseme', 'id': 'IDLE'}]"
"""

class BehaviorManager():

	def __init__(self, pi, nuc):
		self.pi_topic = pi
		self.nuc_topic = nuc
		rospy.init_node("behavior_node", anonymous=True)
		rospy.Subscriber(self.pi_topic+'/behavior', String, self.handle_behavior)
		self._behavior_client = actionlib.SimpleActionClient('/DB1/Behavior',BehaviorAction)
		self.face_publisher = rospy.Publisher('/DB1/face', FaceRequest, queue_size=10)
		rospy.spin()
	

	def handle_behavior(self, data):
		print("Handle behaviors")
		behav = literal_eval(data.data)
		visemes = ["BILABIAL","LABIODENTAL","INTERDENTAL","DENTAL_ALVEOLAR","POSTALVEOLAR","VELAR_GLOTTAL","CLOSE_FRONT_VOWEL","OPEN_FRONT_VOWEL","MID_CENTRAL_VOWEL","OPEN_BACK_VOWEL","CLOSE_BACK_VOWEL", 'IDLE']
		gesture_behaviors = filter(lambda b: b["id"] not in visemes, behav)
		viseme_behaviors = filter(lambda b: b["id"] in visemes, behav)
		self.handle_visemes(viseme_behaviors)


	def handle_gestures(self, gesture_behaviors):
		#Handle Gesures
		"""
		ordered_behaviors = sorted(gesture_behaviors, key=lambda behavior: behavior["start"])
		start_time = rospy.Time.now()
        for a in ordered_behaviors:
            rospy.loginfo("Playing behavior: " + str(a))
            while rospy.Time.now()-start_time < rospy.Duration.from_sec(a["start"])+timing_adjust and not self._server.is_preempt_requested():
                pass
            if self._server.is_preempt_requested():
                preempted = True
                break


            if_conflict = BehaviorGoal.DROP
            if "interrupt" in a.keys():
                if_conflict = BehaviorGoal.OVERRIDE

            if "args" in a.keys():
                args = a["args"]
            else:
                args = []

            if "hold" in a.keys():
                hold = bool(a["hold"])
            else:
                hold = True # default is yes, hold pose until told otherwise

            if "move_time" in a.keys():
                end_move = rospy.Time.now()+rospy.Duration.from_sec(a["move_time"])
            else:
                end_move = rospy.Time()

            if "end" in a.keys():
                end_hold = rospy.Duration.from_sec(a["end"])+start_time
            else:
                end_hold = rospy.Time()

            goal = BehaviorGoal(behavior=a["id"], end_hold=end_hold, end_move=end_move, return_to_prior=not hold, if_conflict = if_conflict, args = args, wait_and_block = False) #old version

            self._behavior_client.send_goal(goal) #old version
            return
            """
		
		

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
		#print(viseme_req)
		#viseme_delay_time = timing_adjust.to_sec()
		#print(viseme_delay_time)
		def send_visemes():
			print("Send viseme")
			self.face_publisher.publish(viseme_req)
		t = Timer(0.1, send_visemes)
		t.start()
		start_time = rospy.Time.now()



if __name__ == '__main__':
	pi =  "qt_robot"
	nuc = "qtpc"
	BehaviorManager(pi, nuc)




