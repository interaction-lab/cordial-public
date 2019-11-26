#!/usr/bin/python


import roslib; roslib.load_manifest('cordial_example')
import rospy
from std_msgs.msg import String
import yaml
from cordial_core import RobotManager
from cordial_tts import CoRDialTTS
from subprocess import call
from time import sleep
import argparse

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
    def __init__(self, phrase_file_dir, phrase_file_yaml):
        rospy.init_node("tts_handler")

        # rospy.sleep(3)
        
        self.temp_store = phrase_file_dir
        self.yaml_store = phrase_file_yaml

        print(self.yaml_store, self.temp_store)
        print(phrase_file_yaml, phrase_file_dir)

        self.rm = RobotManager("DB1")
        self.voice = "Justin" #change later to reflect the voice specified in launch file params
        self.tts = CoRDialTTS(self.voice)

        self.tts_subscriber = rospy.Subscriber('tts_topic', String, self.handle_tts_request)


    def handle_tts_request(self, msg):
        print("request received: {}".format(msg.data))

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

    parser = argparse.ArgumentParser(description='CoRDial TTS node for on-the-fly text-to-speech')
    parser.add_argument('-p', '--phrase-file-dir', help='Where the temporary audio file for the phrase is stored', nargs="?", default="$(find cordial_example)/speech/otf_tts/data")
    parser.add_argument('-y', '--yaml-file', help="Where the temporary yaml file is stored (contains the information on the visemes and behavior data)", nargs="?", default="$(find cordial_example)/speech/otf_tts/phrases.yaml")

    args = parser.parse_known_args()[0]

    tts_node = tts_node(args.phrase_file_dir, args.yaml_file)
    rospy.spin()
	
		
