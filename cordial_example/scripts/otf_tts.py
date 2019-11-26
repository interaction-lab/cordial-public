#!/usr/bin/python


import roslib; roslib.load_manifest('cordial_example')
import rospy
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







def process_script(phrase, phraseID, outdir, tts, file_format=".ogg"):
	line = phrase.strip()
	data = tts.phrase_to_file(phraseID, line, outdir)
	print(data)
	data["file"]= data["file"].split(".")[0]+file_format
	return data

def select_voice():
	voice_option = raw_input("Select the voice: enter J for Justin and K for Kimberly: ")
	while True:
		if voice_option == "J" or voice_option == "j":
			return "Justin"
		if voice_option == "K" or voice_option == "k":
			return "Kimberly"
		print("Invalid Selection! Please try again.")
		voice_option = raw_input("Select the voice: enter J for Justin and K for Kimberly")



if __name__=="__main__":
	rospy.init_node("CoRDial_example")
	
	# Import audio file and yaml file destinations from launch file parameters
	temp_store = rospy.get_param('phrase_file_dir')
	yaml_store = rospy.get_param('phrase_file_yaml')
	rm = RobotManager("DB1")
	voice = select_voice()
	tts = CoRDialTTS(voice)
	while True:
		#Read user input
		phrase = raw_input("Say: ")
		if phrase == "EXIT":
			break
		data = process_script(phrase, "statement1", temp_store, tts,".wav")
		data = {"statement1":data}
		with open(yaml_store, 'w') as f_out:
			yaml.dump(data,f_out,default_flow_style = False)
		
		command = "oggdec " + temp_store + "/" + "*.ogg"
		call(command , shell=True)
		rm.say("statement1",wait=True)
