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
import sys
import yaml
import re
import os
import argparse
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
    parser=argparse.ArgumentParser(description="Use cordial_tts to download speech from the AWS servers.")
    parser.add_argument('-v', '--voice', help="Which voice to use with TTS. Child Voices: Ivy, Justin; Adult Voices: Salli, Joey, Kimberly, Kendra, Eric, Jennifer; Silly Voices: Chipmunk", default="Ivy")
    parser.add_argument('-o', '--output', help="Filename to write to", default="phrases.yaml")
    parser.add_argument('-d', '--outdir', help="Directory to store audio files", default="phrases.yaml")
    parser.add_argument('script_file', help="Files to read script from", nargs='+')


    args = parser.parse_args()

    tts = CoRDialTTS(args.voice)

    data = {}
    for f in args.script_file:
        phs = process_script(f,args.outdir,tts,".wav")

        for p in phs:
            if p in data.keys():
                print "Warning: phrase id: " + p + " in script file " + f + " is overwriting a previous instance; check your files!"

        
        data.update(phs)

    with open(args.output, 'w') as f_out:
        f_out.write("# Auto-generated phrase file for the dragonbot\n")
        f_out.write("# Generated from files: "+ " ".join(sys.argv[3:len(sys.argv)]) + "\n\n")
        yaml.dump(data,f_out,default_flow_style = False)



if __name__=="__main__":
    main()
