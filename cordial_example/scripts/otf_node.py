#!/usr/bin/python


import roslib; roslib.load_manifest('cordial_example')
import rospy
from std_msgs.msg import String
import yaml
from cordial_core import RobotManager
from cordial_tts import CoRDialTTS
from subprocess import call
from time import sleep


#------------------------------------------------------------------------------
# Example robot use with CoRDial
# Copyright (C) 2019 Kangmin Tan
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#------------------------------------------------------------------------------


class tts_node():
    def __init__(self):
        rospy.init_node("tts_handler")

        self.temp_store = rospy.get_param('phrase_file_dir')
        self.yaml_store = rospy.get_param('phrase_file_yaml')
        self.rm = RobotManager("DB1")
        self.voice = "Justin" #change later to reflect the voice specified in launch file params
        self.tts = CoRDialTTS(self.voice)

        self.tts_subscriber = rospy.Subscriber('tts_topic', String, self.handle_tts_request)


    def handle_tts_request(self, msg):

		data = self.process_script(msg.data, "otf_statement", self.temp_store, self.tts, ".wav")
		data = {"otf_statement":data}



		with open(self.yaml_store, 'w') as f_out:
			yaml.dump(data,f_out,default_flow_style = False)
		
		command = "oggdec " + self.temp_store + "/" + "*.ogg"
		call(command , shell=True)
		self.rm.say("otf_statement",wait=True)


    def process_script(self, phrase, phraseID, outdir, tts, file_format=".ogg"):
        line = phrase.strip()
        data = tts.phrase_to_file(phraseID, line, outdir)
        print(data)
        data["file"]= data["file"].split(".")[0]+file_format
        return data


if __name__=="__main__":
    tts_node = tts_node()
    rospy.spin()
	
		
