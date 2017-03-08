#!/usr/bin/env python

#------------------------------------------------------------------------------
# Code to pull audio via Pyvona for offline use.
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
from cordial_tts import CoRDialTTS       

def process_script(filename, outdir, tts, file_format=".ogg"):
    out = {}
    with open(filename, 'r') as f:
        for line in f:
            l = line.strip()
            tokens = l.split(']')
            p = tokens[0].strip("[")
            l = tokens[1].strip()
            data=tts.phrase_to_file(p,l,outdir)            
            if p in out.keys():
                print "Warning: phrase id: " + p + " in script file " + filename + " is overwriting a previous instance; check your files!"
                
            data["file"]= data["file"].split(".")[0]+file_format
            out[p]=data
    return out

def main():
    if not len(sys.argv) >= 3:
        print "Usage: gen_phrases.py [voice] [access key] [secret key] [target data dir] [target file] [script file 1] [script file 2] ... [script file n]"
        print "Voices: Ivy, Justin, "
        exit()
    
    data = {}


    access = sys.argv[2]
    secret = sys.argv[3]

    
    tts = CoRDialTTS(sys.argv[1], access, secret)

    for f in sys.argv[6:len(sys.argv)]:
        phs = process_script(f,sys.argv[4],tts,".wav")

        for p in phs:
            if p in data.keys():
                print "Warning: phrase id: " + p + " in script file " + f + " is overwriting a previous instance; check your files!"

        
        data.update(phs)

    with open(sys.argv[5], 'w') as f_out:
        f_out.write("# Auto-generated phrase file for the dragonbot\n")
        f_out.write("# Generated from files: "+ " ".join(sys.argv[3:len(sys.argv)]) + "\n\n")
        yaml.dump(data,f_out,default_flow_style = False)



if __name__=="__main__":
    main()
