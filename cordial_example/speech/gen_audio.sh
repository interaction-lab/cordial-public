#!/bin/bash

mkdir data
rosrun cordial_tts gen_phrases.py Justin data phrases.yaml script.txt
oggdec data/*.ogg
