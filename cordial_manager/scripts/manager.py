#!/usr/bin/env python

import rospy
import sys
import os
import json
import time
from std_msgs.msg import String, Bool


class InteractionManager():
	
	def __init__(self):
		self.pub_dic = {}
		self.interaction_data = {}
		self.action_data = {}
		self.tracked = False
		rospy.init_node("interaction_controller_node", anonymous=True)
		#rospy.Subscriber('/cordial/interacting', String, self.handle_interacting)
		self.read_interactions()
		self.handle_interacting("greeting1") #FOR TESTING
		rospy.spin()

	def read_interactions(self):
		base_dir = os.path.dirname(__file__)
		with open(base_dir + '/data/interactions.json', "r") as json_file:
			self.interaction_data = json.load(json_file)
		with open(base_dir + '/data/actions.json', "r") as json_file:
			self.action_data = json.load(json_file)
			for action in self.action_data:
				if  self.action_data[action]['type'] == "Bool":
					self.pub_dic[self.action_data[action]['topic']] = rospy.Publisher(self.action_data[action]['topic'], Bool, queue_size=1)
				elif self.action_data[action]['type'] == "String":
					self.pub_dic[self.action_data[action]['topic']] = rospy.Publisher(self.action_data[action]['topic'], String, queue_size=1)


	def handle_user_lost(self, data):
		print(data.data)
		#what to do when the user is lost. Likely send a message to the DM to update the interaction


	def handle_interacting(self, data):
		for interaction in self.interaction_data:
			#if data.data == interaction:
			if data == interaction: #FOR TESTING 
				steps = self.interaction_data[interaction]['steps']
				parameters = self.interaction_data[interaction]['parameters']
				for step in steps:
					wait_from_type_sub = str(step['wait']['from'])
					wait_message_type = str(step['wait']['message_type'])
					wait_message_content = str(step['wait']['for'])
					wait_min_time = int(step['wait']['min_wait'])
					wait_max_time = int(step['wait']['max_wait'])
					check_from_type_sub = str(step['check']['from'])
					check_message_type = str(step['check']['message_type'])
					check_message_content = str(step['check']['for'])
					check_min_time = int(step['check']['min_wait'])
					check_max_time = int(step['check']['max_wait'])
					name = str(step['name'])
					log_text = str(step['description'])
					type_sub = str(step['type'])

					action_message_content = str(self.action_data[name]['message_content'])
					action_topic = str(self.action_data[name]['topic'])
					action_topic_type = str(self.action_data[name]['type'])
					rospy.sleep(wait_min_time)
					if type_sub == "Topic":
						publisher = self.pub_dic[action_topic]
						if wait_from_type_sub == '':
							if action_topic_type == "Bool":
								while not rospy.is_shutdown():
									rospy.loginfo("First step: the publisher is " + log_text)
									publisher.publish(bool(action_message_content))
									if check_message_type == "Bool":
										rospy.loginfo("Message type is Bool and the topic is:" + check_from_type_sub)
										rospy.sleep(2)
										msg = wait_for_message(check_from_type_sub, Bool, 1)
										if msg == check_message_content:
											print("I received something!")
											break
									elif check_message_type == "String":
										msg = wait_for_message(check_from_type_sub, String, 1)
										if msg == check_message_content:
											print("I received something!")
											break
							elif action_topic_type == "String":
								while not rospy.is_shutdown():
									rospy.loginfo("The publisher is " + log_text)
									publisher.publish(str(action_message_content))
									if check_message_type == "Bool":
										msg = wait_for_message(check_from_type_sub, Bool, 1)
										if msg == check_message_content:
											print("I received something!")
											break
									elif check_message_type == "String":
										msg = wait_for_message(check_from_type_sub, String, 1)
										if msg == check_message_content:
											print("I received something!")
											break
						else:
							if action_topic_type == "Bool":
								msg = wait_for_message(wait_from_type_sub, Bool, wait_max_time)
								if msg == wait_message_content:
									while not rospy.is_shutdown():
										rospy.loginfo("The publisher is " + log_text)
										publisher.publish(bool(action_message_content))
										if check_message_type == "Bool":
											msg = wait_for_message(check_from_type_sub, Bool, 1)
											if msg == check_message_content:
												print("I received something!")
												break
										elif check_message_type == "String":
											msg = wait_for_message(check_from_type_sub, String, 1)
											if msg == check_message_content:
												print("I received something!")
												break
							elif action_topic_type == "String":
								msg = wait_for_message(wait_from_type_sub, String, wait_max_time)
								if msg == wait_message_content:
									while not rospy.is_shutdown():
										rospy.loginfo("The publisher is " + log_text)
										publisher.publish(str(action_message_content))
										if check_message_type == "Bool":
											msg = wait_for_message(check_from_type_sub, Bool, 1)
											if msg == check_message_content:
												print("I received something!")
												break
										elif check_message_type == "String":
											msg = wait_for_message(check_from_type_sub, String, 1)
											if msg == check_message_content:
												print("I received something!")
												break


						

					#print(wait_from_type_sub, wait_min_time, name, log_text, message_content,topic)
					
						
		#check which interaction starts! read the json in the init!
		#publish the text to lex to start the interaction (lex_text_input_pub) or start the microphone (listening_pub) or stop everything!!

class _WFM(object):
    def __init__(self):
        self.msg = None
    def cb(self, msg):
        if self.msg is None:
            self.msg = msg

def wait_for_message(topic, topic_type, timeout=None):
    """
    Receive one message from topic.
    
    This will create a new subscription to the topic, receive one message, then unsubscribe.

    @param topic: name of topic
    @type  topic: str
    @param topic_type: topic type
    @type  topic_type: L{rospy.Message} class
    @param timeout: timeout time in seconds
    @type  timeout: double
    @return: Message
    @rtype: L{rospy.Message}
    @raise ROSException: if specified timeout is exceeded
    @raise ROSInterruptException: if shutdown interrupts wait
    """
    wfm = _WFM()
    s = None
    try:
        s = rospy.topics.Subscriber(topic, topic_type, wfm.cb)
        if timeout is not None:
            timeout_t = time.time() + timeout
            while not rospy.core.is_shutdown() and wfm.msg is None:
                rospy.rostime.wallsleep(0.01)
                if time.time() >= timeout_t:
                    #raise rospy.exceptions.ROSException("timeout exceeded while waiting for message on topic %s"%topic)
                    return False

        else:
            while not rospy.core.is_shutdown() and wfm.msg is None:
                rospy.rostime.wallsleep(0.01)            
    finally:
        if s is not None:
            s.unregister()
    if rospy.core.is_shutdown():
        raise rospy.exceptions.ROSInterruptException("rospy shutdown")
    return wfm.msg
	
if __name__ == '__main__':
	InteractionManager()

