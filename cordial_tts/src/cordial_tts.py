#!/usr/bin/env python

#------------------------------------------------------------------------------
# Interface between CoRDial and Pyvona
# Copyright (C) 2017 Elaine Schaertl Short
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



import roslib; roslib.load_manifest('cordial_tts')
import pyvona as iv
import sys
import yaml
import re
import os

class CoRDialTTS():
    def __init__(self, voice, access_key, secret_key):
        self._tts = tts = iv.create_voice(access_key, secret_key, voice)

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


        
        s = self._tts.fetch_speechmarks_str(phrase)
        s = s.split("\n")
        s = map(lambda l: l.split(" "), s)
        s = filter(lambda l: len(l) > 1, s)

        word_times = filter(lambda l: l[1]=="word", s)
        for a in actions:
            if a[0] > len(word_times)-1:
                a[0] = s[-1][0]
            else:
                a[0] = (word_times[a[0]][0])

        data=[]
        for a in actions:
            args = a[2]
            data.append({"start":float(a[0])+.01, #prevent visemes and actions from being at exactly the same time
                         "type":"action",
                         "args":args,
                         "id": a[1]})

        visemes = map(lambda l: [l[0],vis_transl[l[-1]]], filter(lambda l: l[1]=="viseme",s))

        for v in visemes:
            data.append({"start":float(v[0]),
                         "type":"viseme",
                         "id": v[1]})
        return phrase, data

    def phrase_to_file(self,name, line, outdir):
        outdir_path = os.path.abspath(os.path.expanduser(outdir))
        
        data={}
        phrase, data["behaviors"]=self.extract_behaviors(line)

        self._tts.fetch_voice(phrase, outdir_path+"/"+name+"."+self._tts.codec)

        data["file"] = outdir_path+"/"+name+"."+self._tts.codec
        data["text"] = '"'+phrase+'"'
        return data

    def say(self,phrase, wait=False, interrupt=False):
        return self._tts.speak(phrase, wait, interrupt)

    def is_speaking(self):
        self._tts.is_busy()

    def shutup(self):
        self._tts.shutup()
