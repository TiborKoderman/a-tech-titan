#!/usr/bin/python3

import rospy
import cv2
import actionlib
import numpy as np
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import face_recognition
import argparse
import pickle
import getch
from std_msgs.msg import Int8

def detect_posters(rgb_image):

    data = pickle.loads(open("encodings_poster.pickle", "rb").read())

    boxes = face_recognition.face_locations(rgb_image, model="hog")
    encodings = face_recognition.face_encodings(rgb_image, boxes)
    object_type = ""

    for encoding in encodings:
        matches = face_recognition.compare_faces(data["encodings"],
            encoding)
        name = "Unknown"
        if True in matches:
            matchedIdxs = [i for (i, b) in enumerate(matches) if b]
            counts = {}
            for i in matchedIdxs:
                name = data["names"][i]
                counts[name] = counts.get(name, 0) + 1
            name = max(counts, key=counts.get)
        object_type = name

    return object_type

def detect_poster_faces(rgb_image):

    data = pickle.loads(open("encodings_poster_faces.pickle", "rb").read())

    boxes = face_recognition.face_locations(rgb_image, model="hog")
    encodings = face_recognition.face_encodings(rgb_image, boxes)
    final_name = ""

    for encoding in encodings:
        matches = face_recognition.compare_faces(data["encodings"],
            encoding)
        name = "Unknown"
        if True in matches:
            matchedIdxs = [i for (i, b) in enumerate(matches) if b]
            counts = {}
            for i in matchedIdxs:
                name = data["names"][i]
                counts[name] = counts.get(name, 0) + 1
            name = max(counts, key=counts.get)
        final_name = name

    return final_name