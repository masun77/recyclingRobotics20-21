from PIL import Image
import numpy as np
from TrainYourOwnYOLO.three_Inference.camera_detector_helper import setup, processImage, close
import pyrealsense2 as rs
import numpy as np
import cv2

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


# Setup machine vision
setup()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# do other robot setup stuff... homing etc...

# this is just a loop I wrote, you could make
# a prettier loop or thread or whatever
while still_running() == "y":

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape

    # If depth and color resolutions are different, resize color image to match depth image for display
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
    else:
        images = np.hstack((color_image, depth_colormap))

    # Show images
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', images)
    # cv2.waitKey(1)

    # get an image from the camera (np array)
    img = color_image

    # call processImage on the nparray. The returned
    # prediction object contains the xmin, ymin, xmax, ymax, class, confidence
    # of the max-confidence bounding box detected
    box = processImage(img)

    # use the bounding box to find where you want to pickup
    if box != None:
        center = getCenter(box)
        print("   Center: " + str(center))
    else:
        print("Unable to find object")

    # todo: some other robot stuff like pick up the object

    # to end the loop, either manually change the contents
    # of the text file 'state.txt' to anything other than y
    # or open up another terminal and call 'python stopCall.py'
    # from this folder


pipeline.stop()
close()
