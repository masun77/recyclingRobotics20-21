# recycling_mqp_desktop

In the src/scripts folder:
 - dynamixel_control
 - env - the virtual environment for running code. Activate it with .\env\Scripts\activate from within the scripts folder
    before running python files
 - rospy
 - serial
 - TrainYourOwnYOLO - cloned github repo for YOLO image recognition. check out the readmes within the folder for more info
 - dynamixel_control
 - exampleRobot.py - run with 'python exampleRobot.py' from this folder to run the robot
 - state.txt - the state of the robot: on if y, off otherwise
 - stopCall.py - run this with 'python stopCall.py' in a terminal to stop the robot

To run example robot:

# Create a virtual environment (to keep your libraries separate)
python -m venv env
.\env\Scripts\activate

# Install libraries
pip install -r TrainYourOwnYOLO\requirements.txt

# Run the robot file
python exampleRobot.py