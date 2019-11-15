CoRDial Setup Instructions

1. Install Ubuntu 16.04 on your computer

    http://releases.ubuntu.com/16.04/

2. Install ROS kinetic (full version)

    http://wiki.ros.org/kinetic/Installation/Ubuntu

3. Install Prerequisites

~~~~
$ sudo apt-get install nodejs
$ sudo apt-get install npm
$ sudo npm install http-server -g
$ sudo pip install boto3

$ ln -s /usr/bin/nodejs /usr/bin/node


$ sudo apt-get install ros-kinetic-rosbridge-server vorbis-tools python-pygame python-requests python-serial ros-kinetic-tf python-gst0.10 python-scipy
~~~~
4. Install the pololu maestro MCB software (if using the spritebot)

    https://www.pololu.com/file/0J315/maestro-linux-150116.tar.gz
    installation instructions are in README.txt
    
5. Set up AWS account following these steps (or request access to the lab account from a PhD): 

    Create an Amazon Web Services account. AWS has a 1 year free trial that includes a limited number of Polly usages.
    Keep this in mind so you do not get charged money at the end of the year.

    Once you've created an account, create an IAM user to access Polly.

      * Give the IAM user access permissions to AWS Polly.
      * Give the IAM user access keys. Be sure to save the secret key as you only have one chance to look at it.
      
    https://docs.aws.amazon.com/cli/latest/userguide/install-linux.html - Use this link to install the AWS CLI on your PC.
    
    Then, in the terminal,
    ~~~~
    $ aws configure
    # Enter the IAM user access and secret keys here.
    ~~~~

6. Clone the repository in <your_catkin_workspace>/src

~~~~
$ git clone https://github.com/interaction-lab/cordial-public.git
~~~~

7. Make everything

~~~~
$ cd ..
$ catkin_make clean
$ catkin_make
$ catkin_make install
~~~~

8. If you're on 16.10, add the rosbridge websocket port to your firewall's allowed list

~~~~
$ sudo ufw allow 9090
~~~~

9. Setup your ssh keys for git (optional)

    https://help.github.com/articles/generating-an-ssh-key/
    

10. Test your installation

    1. generate audio files (only needs to be done once or when changes to speech are made):
        
        ~~~~
        $ roscd cordial_example/speech
        $ ./gen_audio.sh
        ~~~~

    2. start "background" ROS nodes:
        ~~~~
        $ roslaunch cordial_example test_setup.launch
        ~~~~

    3. view face (In a new terminal window):
      
        ~~~~
        $ roscd cordial_face/web
        $ http-server
        # Navigate to second address listed in web browser to view the robot face screen
        # Go to any of the first 4 options - Kiwi GL/Kiwi/Chili GL/Chili
        ~~~~

    4. run example (In a new terminal window):
      
        ~~~~
        $ rosrun cordial_example robot_only.py
        ~~~~
