CoRDial Setup Instructions

1. Install Ubuntu 16.04 on your computer

    http://releases.ubuntu.com/16.04/

2. Install ROS kinetic

    http://wiki.ros.org/kinetic/Installation/Ubuntu

3. Install Prerequisites

~~~~
sudo apt-get install nodejs
sudo apt-get install npm
sudo npm install http-server -g

ln -s /usr/bin/nodejs /usr/bin/node
#fixes namespace issue (https://github.com/nodejs/node-v0.x-archive/issues/3911)

sudo apt-get install ros-kinetic-rosbridge-server vorbis-tools python-pygame python-requests python-serial ros-kinetic-tf python-gst0.10 python-scipy
~~~~
5. Install the pololu maestro MCB software

    https://www.pololu.com/file/0J315/maestro-linux-150116.tar.gz
    installation instructions are in README.txt

8. Clone the repository

~~~~
roscd
git clone https://github.com/interaction-lab/cordial-public.git
~~~~

10. Make everything

~~~~
rosmake cordial_sprite cordial_tablet
~~~~

11. If you're on 16.10, add the rosbridge websocket port to your firewall's allowed list

~~~~
sudo ufw allow 9090
~~~~

7. Setup your ssh keys for git (optional)

    https://help.github.com/articles/generating-an-ssh-key/

12. Test your installation

    1. generate audio files (only needs to be done once or when changes to speech are made):
        roscd cordial_example/speech
        ./gen_audio.sh

    2. start "background" ROS nodes:
        plug in the robot (if testing with physical robot)
        roslaunch cordial_example run.launch

    3. run example:
        roscd cordial_example/scripts
        ./tablet_only.py
        (can also try ./robot_only.py)

    4. view tablet:
        roscd cordial_tablet/web
        http-server
        Navigate to second address listed in web browser to view the tablet screen

    5. view face:
        roscd cordial_face/web
        http-server
        Navigate to second address listed in web browser to view the robot face screen
