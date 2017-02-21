#!/bin/bash

mkdir data
rosrun cordial_tts gen_phrases.py Justin YOUR_ACCESS_KEY YOUR_SECRET_KEY data phrases.yaml script.txt
oggdec data/*.ogg
