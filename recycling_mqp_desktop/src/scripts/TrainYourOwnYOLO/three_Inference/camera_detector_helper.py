import os
import sys

def get_parent_dir(n=1):
    """returns the n-th parent dicrectory of the current
    working directory"""
    current_path = os.path.dirname(os.path.abspath(__file__))
    for _ in range(n):
        current_path = os.path.dirname(current_path)
    return current_path

# Do this before the other imports so they work
src_path = os.path.join(get_parent_dir(1), "2_Training", "src")
utils_path = os.path.join(get_parent_dir(1), "Utils")

sys.path.append(src_path)
sys.path.append(utils_path)

import argparse
from keras_yolo3.yolo import YOLO, detect_video, detect_webcam
from PIL import Image
from timeit import default_timer as timer
from utils import load_extractor_model, load_features, parse_input, detect_object
import test
import utils
import pandas as pd
import numpy as np
from Get_File_Paths import GetFileList
import random
from Train_Utils import get_anchors

yolo = None
detection_results_folder = ""
counter = 1
start = None
out_df = None
detection_results_file = ""

# Setup the variables and yolo model - takes up to 10 seconds
def setup():
    global yolo, detection_results_folder, start, out_df, detection_results_file
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"

    # Set up folder names for default values
    data_folder = os.path.join(get_parent_dir(n=1), "Data")

    image_folder = os.path.join(data_folder, "Source_Images")

    image_test_folder = os.path.join(image_folder, "Test_Images")

    detection_results_folder = os.path.join(image_folder,
                                            "Test_Image_Detection_Results")
    detection_results_file = os.path.join(detection_results_folder,
                                          "Detection_Results.csv")

    model_folder = os.path.join(data_folder, "Model_Weights")

    model_weights = os.path.join(model_folder, "trained_weights_final.h5")
    model_classes = os.path.join(model_folder, "data_classes.txt")

    anchors_path = os.path.join(src_path, "keras_yolo3", "model_data",
                                "yolo_anchors.txt")

    confidence = .25

    # Make a dataframe for the prediction outputs
    out_df = pd.DataFrame(
        columns=[
            "image",
            "image_path",
            "xmin",
            "ymin",
            "xmax",
            "ymax",
            "label",
            "confidence",
            "x_size",
            "y_size",
        ]
    )

    # labels to draw on images
    class_file = open(model_classes, "r")
    input_labels = [line.rstrip("\n") for line in class_file.readlines()]
    print("Found {} input labels: {} ...".format(len(input_labels), input_labels))

    # define YOLO detector
    yolo = YOLO(
        **{
            "model_path": model_weights,
            "anchors_path": anchors_path,
            "classes_path": model_classes,
            "score": confidence,
            "gpu_num": 0,
            "model_image_size": (416, 416),
        }
    )

    start = timer()

# Use model to find predictions for a single image. Save the bounding boxes to
# a dataframe to write to a csv at end.
# img - numpy array of RGB image
# return the prediction with max confidence for this image
def processImage(img):
    global yolo, detection_results_folder, counter, out_df

    imgName = "img_" + str(counter)

    prediction, image = detect(
        yolo,
        img,
        imgName,
        save_img=True,
        save_img_path=detection_results_folder,
        postfix= "_cardboard",
    )
    counter += 1
    y_size, x_size, _ = np.array(image).shape
    maxConf = 0
    maxPred = None
    for single_prediction in prediction:
        out_df = out_df.append(
            pd.DataFrame(
                [
                    [
                        os.path.basename(imgName.rstrip("\n")),
                        imgName.rstrip("\n"),
                    ]
                    + single_prediction
                    + [x_size, y_size]
                ],
                columns=[
                    "image",
                    "image_path",
                    "xmin",
                    "ymin",
                    "xmax",
                    "ymax",
                    "label",
                    "confidence",
                    "x_size",
                    "y_size",
                ],
            )
        )
        if (single_prediction[5] > maxConf):
            maxPred  = single_prediction

    return maxPred

# Shut down the model session and print results
def close():
    global yolo, start, out_df, detection_results_file
    yolo.close_session()
    end = timer()
    print(
        "Processed {} images in {:.1f}sec - {:.1f}FPS".format(
            counter,
            end - start,
            counter / (end - start),
        )
    )
    out_df.to_csv(detection_results_file, index=False)

def detect(yolo, image_array, img_name, save_img, save_img_path="./", postfix=""):
    """
    Call YOLO logo detector on input image, optionally save resulting image.

    Args:
      yolo: keras-yolo3 initialized YOLO instance
      img_name: path to image file
      save_img: bool to save annotated image
      save_img_path: path to directory where to save image
      postfix: string to add to filenames
    Returns:
      prediction: list of bounding boxes in format (xmin,ymin,xmax,ymax,class_id,confidence)
      image: unaltered input image as (H,W,C) array
    """

    prediction, new_image = yolo.detect_image(Image.fromarray(image_array))

    if save_img:
        new_image.save(os.path.join(save_img_path, img_name + ".jpg"))

    return prediction, image_array