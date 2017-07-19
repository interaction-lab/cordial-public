# Cordial Polly
This version of the text to speech package for cordial-public uses Amazon Polly. 

## Setup
Install Boto:

	$ pip install boto3

Setup a ROS-build workspace to contain this package: [Configure ROS workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

### Amazon Web Services
Ask if the lab already has an AWS account. If so, request the access key to authenticate into AWS programmatically.
Otherwise, go through the steps below for creating and configuring an AWS account.

#### Create AWS Account
[Create an Amazon Web Services account](http://docs.aws.amazon.com/lambda/latest/dg/setting-up.html). AWS has a 1 year
free trial that includes a limited number of Polly usages. Keep this in mind so you do not get charged money at the end
of the year.

Once you've created an account, [create an IAM user](http://docs.aws.amazon.com/IAM/latest/UserGuide/id_users_create.html#id_users_create_console) to access Polly.
* Give the IAM user access permissions to AWS Polly.
* Give the IAM user access keys. Be sure to **save the secret key** as you only have one chance to look at it.

#### Configure AWS CLI
Once you have an AWS account, [install the AWS CLI](http://docs.aws.amazon.com/cli/latest/userguide/installing.html).

[Configure your local machine to use AWS](http://docs.aws.amazon.com/cli/latest/userguide/cli-chap-getting-started.html)
* You'll need the AWS IAM user's access and secret key
* For region, use `us-west-2`

## Text To Speech
Create a `script.txt` file (filename not important). The format must be a single line per phrase with a unique key for each phrase:

```
[key1]phrase1
[key2]phrase2
```

This package also supports CoRDial behaviors embedded in the phrases inside angle brackets. For example:

```
[hello_world]Hello<behavior1>, world!
```

## Run
Source the ROS workspace setup files.

To run the TTS directly:

    $ rosrun cordial_polly gen_phrases.py -v <Polly-Voice> -o <output-dir> -p <phrases-file> path/to/script.txt

For help:

    $ rosrun cordial_polly gen_phrases.py -h

The TTS system will create OGG audio files in the output-dir and a CoRDial phrases YAML file for controlling SPRITE.

This repo also includes an example bash script called `gen_audio.sh` for running the TTS script from another package.
