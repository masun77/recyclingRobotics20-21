from PIL import Image
import numpy as np
from TrainYourOwnYOLO.three_Inference.camera_detector_helper import setup, processImage, close

# Check whether to continue running the while loop
def still_running():
    file = open('state.txt', 'r')
    contents = file.read()
    file.close()
    return contents

# Get the center of a bounding box
# box - the prediction from the yolo model [xmin, ymin, xmax, ymax, ...]
def getCenter(box):
    xAve = (box[2] + box[0]) / 2
    yAve = (box[3] + box[1]) / 2
    return xAve, yAve


# Run the robot
setup()

# do other robot setup stuff... homing etc...

# this is just a loop I wrote, you could make
# a prettier loop or thread or whatever
while still_running() == "y":
    # todo: get an image from the camera -> np array (use example code)
    img = np.array(Image.open("testImg.jpeg"))

    # call processImage on the nparray. The returned
    # prediction object contains the xmin, ymin, xmax, ymax, class, confidence
    # of the max-confidence bounding box detected
    box = processImage(img)

    # use the bounding box to find where you want to pickup
    center = getCenter(box)
    print("   Center: " + str(center))

    # todo: some other robot stuff like pick up the object

    # to end the loop, either manually change the contents
    # of the text file 'state.txt' to anything other than y
    # or open up another terminal and call 'python stopCall.py'
    # from this folder
close()
