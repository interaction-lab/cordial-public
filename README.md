CoRDial Setup Instructions

1. Install Ubuntu 16.04 on your computer

    http://releases.ubuntu.com/16.04/

2. Install ROS kinetic

    http://wiki.ros.org/kinetic/Installation/Ubuntu

3. Install Prerequisites

~~~~
$ sudo apt-get install nodejs
$ sudo apt-get install npm
$ sudo npm install http-server -g

$ ln -s /usr/bin/nodejs /usr/bin/node


$ sudo apt-get install ros-kinetic-rosbridge-server vorbis-tools python-pygame python-requests python-serial ros-kinetic-tf python-gst0.10 python-scipy
~~~~
4. Install the pololu maestro MCB software

    https://www.pololu.com/file/0J315/maestro-linux-150116.tar.gz
    installation instructions are in README.txt

5. Clone the repository

~~~~
$ roscd
git clone https://github.com/interaction-lab/cordial-public.git
~~~~

6. Make everything

~~~~
rosmake cordial_sprite cordial_tablet
~~~~

7. If you're on 16.10, add the rosbridge websocket port to your firewall's allowed list

~~~~
sudo ufw allow 9090
~~~~

8. Setup your ssh keys for git (optional)

    https://help.github.com/articles/generating-an-ssh-key/

9. Test your installation

    1. generate audio files (only needs to be done once or when changes to speech are made):
        
        ~~~~
        roscd cordial_example/speech
        ./gen_audio.sh
        ~~~~

    2. start "background" ROS nodes:
        ~~~~
        roslaunch cordial_example run.launch
        ~~~~

    3. view face:
      (In a new terminal window)
      
        ~~~~
        roscd cordial_face/web
        http-server
        #Navigate to second address listed in web browser to view the robot face screen
        ~~~~

    4. run example:
      (In a new terminal window)
      
      ~~~~
        rosrun cordial_example robot_only.py
      ~~~~
