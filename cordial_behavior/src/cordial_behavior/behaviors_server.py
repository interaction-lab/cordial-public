import rospy
import sys
import actionlib
import json
from ast import literal_eval
from std_msgs.msg import String
from cordial_behavior.msg import *
from cordial_face.msg import FaceRequest

"""
class BehaviorServer():

	def __init__(self, pi, nuc):
		self.pi_topic = pi
		self.nuc_topic = nuc
		rospy.init_node("behavior_server", anonymous=True)
		self._behavior_client = actionlib.SimpleActionClient(base_topic+'Behavior',BehaviorAction)
        self._phone_face = True
        if self._phone_face:
            self._face_pub = rospy.Publisher(base_topic+'face', FaceRequest, queue_size=10)
            rospy.sleep(0.5)
        rospy.loginfo("CoRdial Player waiting for behavior server..")
        self._behavior_client.wait_for_server()
        rospy.loginfo("CoRDial Player connected to behavior server")
		rospy.spin()
	

	def handle_behavior(self, request):
		return


if __name__ == '__main__':
	pi =  "qt_robot"
	nuc = "qtpc"
    BehaviorServer(pi, nuc)


"""

