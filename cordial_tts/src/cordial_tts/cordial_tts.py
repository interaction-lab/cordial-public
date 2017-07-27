#!/usr/bin/env python

#------------------------------------------------------------------------------
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

"""
Text To Speech Service integrated with Amazon Polly
"""
import sys
import re
import os
import json
from boto3 import client
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import pygame
import tempfile

class CoRDialTTS():
    def __init__(self, voice, **kwargs):
        self.voice = voice; 
        self.aws_polly = client("polly")

    def extract_behaviors(self,line):
        vis_transl = {"p": "M_B_P",
                  "t": "N_NG_D_Z",
                  "S": "CH_SH_ZH",
                  "T": "N_NG_D_Z",
                  "f": "M_B_P",
                  "k": "AA_AH",
                  "i": "EY",
                  "r": "R_ER",
                  "s": "N_NG_D_Z",
                  "u": "CH_SH_ZH",
                  "@": "AA_AH",
                  "a": "AA_AH",
                  "e": "EY",
                  "E": "EH_AE_AY",
                  "i": "EY",
                  "o": "AO_AW",
                  "O": "AA_AH",
                  "u": "AO_AW",
                  "sil": "IDLE"}
        

        tokens = re.split("(<[^<>]*>)", line)

        phrase = "".join(filter(lambda s: ">" not in s, tokens))

        def cond_split(s):
            if len(s)>=2 and s[-1]==">" and s[0]=="<":
                return [s]
            else:
                return re.split("\s+",s)


        tokens = map(lambda s: cond_split(s), tokens)
        words = []

        for t in tokens:
            words += filter(lambda s: len(s) > 0, t)

        actions = []
        i = 0
        for w in words:
            if re.match("<.*>", w):
                args = w.strip("<>").split()
                name = args.pop(0)
                actions.append([i,name,args])
            else:
                i += 1

	try:
	    response = self.aws_polly.synthesize_speech(Text=phrase, OutputFormat="json",
                                                        VoiceId=self.voice, SpeechMarkTypes =["viseme", "word"])
    	except (BotoCoreError, ClientError) as error:
	    print(error)
	    sys.exit(-1)

	s = []
	if "AudioStream" in response:
	    with closing(response["AudioStream"]) as stream:
                data = stream.read()
                s = data.split('\n')
                s = [json.loads(line) for line in s if line != '']
	else:
	    print("Could not stream audio")
	    sys.exit(-1)

	word_times = filter(lambda l: l["type"]=="word", s) # Start edits
        for a in actions:
            if a[0] > len(word_times)-1:
                a[0] = s[-1]["time"] / 1000.  # convert ms to seconds
            else:
	        a[0] = (word_times[a[0]]["time"]) / 1000.  # convert ms to seconds


	data=[]
        for a in actions:
            args = a[2]
            data.append({"start":float(a[0]),
                         "type":"action",
                         "args":args,
			 "id": a[1]}) # End edits

	visemes = map(lambda l: [l["time"],vis_transl[l["value"]]], filter(lambda l: l["type"]=="viseme",s))
	for v in visemes:
            data.append({"start":float(v[0]) / 1000.,  # convert ms to seconds
                         "type":"viseme",
                         "id": v[1]})	


        return phrase, data

    def phrase_to_file(self,name, line, outdir):
        outdir_path = os.path.abspath(os.path.expanduser(outdir))
        
        data={}
        phrase, data["behaviors"]=self.extract_behaviors(line)

        #self._tts.fetch_voice(phrase, outdir_path+"/"+name+"."+self._tts.codec)

        data["file"] = outdir_path+"/"+name+".ogg"
        data["text"] = '"'+phrase+'"'


	try:
	    response = self.aws_polly.synthesize_speech(Text=phrase, OutputFormat="ogg_vorbis",
                                                        VoiceId=self.voice)
    	except (BotoCoreError, ClientError) as error:
	    print(error)
	    sys.exit(-1)

	if "AudioStream" in response:
	    with closing(response["AudioStream"]) as stream:
		output = data["file"]
		try:
		    with open(output, "wb") as file:
			file.write(stream.read())
		except IOError as error:
		    	print(error)
			sys.exit(-1)
	    
	else:
	    print("Could not stream audio")
	    sys.exit(-1)

        return data

    def say(self,phrase, wait=False, interrupt=False):
        try:
	    response = self.aws_polly.synthesize_speech(Text=phrase, 
                                                        OutputFormat="ogg_vorbis",
                                                        VoiceId=self.voice)
    	except (BotoCoreError, ClientError) as error:
	    print(error)
	    sys.exit(-1)

        with tempfile.SpooledTemporaryFile() as f:
            if "AudioStream" in response:
                with closing(response["AudioStream"]) as stream:
                    try:
                        f.write(stream.read())
                    except IOError as error:
                        print(error)
                        sys.exit(-1)
	    
            else:
                print("Could not stream audio")
                sys.exit(-1)
            
            f.seek(0)

            if not pygame.mixer.get_init():
                pygame.mixer.init()
            else:
                if interrupt:
                    pygame.mixer.stop()
            channel = pygame.mixer.Channel(5)
            sound = pygame.mixer.Sound(f)
            channel.play(sound)
            if wait:
                while channel.get_busy():
                    pass
                return -1
            return sound.get_length()
	    

    def is_speaking(self):
        return not pygame.mixer.get_init() or pygame.mixer.Channel(5).get_busy()

    def shutup(self):
        if pygame.mixer.get_init():
            pygame.mixer.stop()
