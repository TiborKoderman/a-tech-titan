#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import actionlib
import numpy as np
import face_recognition
import tf2_geometry_msgs
import tf2_ros
import tf
import copy
import math
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float32, Float64, Int32, Bool, ColorRGBA, Int8
from nav_msgs.msg import OccupancyGrid
from actionlib import SimpleActionClient
from os.path import dirname, join

import speech_recognition as sr

from exercise6.srv import Barrels

from geometry_msgs.msg import PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, Twist
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import ColorRGBA
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from exercise6.msg import Ring
from exercise6.msg import Cylinder
import pickle
import argparse
import pyttsx3

import face_recognition
import getch
import pytesseract

class Movement:

    def __init__(self):
        
        printStatusMsgs.info("Initializing Movement")

        rospy.init_node('movement', anonymous=True)
        
        printStatusMsgs.ok("Node initialized")
        

        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )
            
        self.cancel_goal_publisher = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )
            
        self.ring_sub = rospy.Subscriber(
            "/detected_ring", Ring, self.find_rings_callback
        )
         
        self.cylinder_sub = rospy.Subscriber(
            "/detected_cylinder_color", Cylinder, self.find_cylinders_callback
        )
        """  
        self.odom_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback
        )
        """
        
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.arm_user_command_pub = rospy.Publisher(
            "/arm_command", String, queue_size=10
        )
        
         #move arm into position
        self.pose_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.suspects = list()
        self.state = "get_next_waypoint"

        self.arm_camera_sub = Subscriber("/arm_camera/rgb/image_raw", Image)
        self.arm_depth_sub = Subscriber("/arm_camera/depth/image_raw", Image)

        self.camera_sub = Subscriber("/camera/rgb/image_raw", Image)
        self.depth_sub = Subscriber("/camera/depth/image_raw", Image)


        # ats_main = ApproximateTimeSynchronizer([self.camera_sub, self.depth_sub], queue_size=5, slop=0.01)
        # ats_main.registerCallback(self.main_camera_callback)

        ats_arm = ApproximateTimeSynchronizer([self.arm_camera_sub, self.arm_depth_sub], queue_size=5, slop=0.01)
        ats_arm.registerCallback(self.arm_camera_callback)
        
        self.arm_image_pub = rospy.Publisher(
            "/arm_camera/vizualisations", Image, queue_size=10
        );
        
        # ats = ApproximateTimeSynchronizer([self.arm_camera_sub, self.arm_depth_sub], queue_size=5, slop=0.01)
        # ats.registerCallback(self.arm_depth_callback)

        printStatusMsgs.ok("Topics sucessfully subscribed")
        
        
        self.tf_buf = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buf)
        
           
        self.bridge = CvBridge()
        
         
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")
        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)
        self.dims = (0, 0, 0)

        self.marker_array = MarkerArray()
        self.text_marker_array = MarkerArray()
        self.marker_array_est = MarkerArray()
        self.annotation_array = MarkerArray()


        self.marker_num = 0
        self.est_count = 0

        self.processed_image_pub = rospy.Publisher('processed_image', Image, queue_size=1000)
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.text_markers_pub = rospy.Publisher('text_face_markers', MarkerArray, queue_size=1000)
        self.face_markers_est_pub = rospy.Publisher('face_markers_est', MarkerArray, queue_size=1000)
        self.annotation_markers_pub = rospy.Publisher('annotation_markers', MarkerArray, queue_size=1000)


        self.markers_pub.publish(self.marker_array)
        self.text_markers_pub.publish(self.text_marker_array)
        self.face_markers_est_pub.publish(self.marker_array_est)
        self.annotation_markers_pub.publish(self.annotation_array)


        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.numb_of_faces = 7
        self.ap = argparse.ArgumentParser()
        self.data = pickle.loads(open(join(dirname(__file__), "encodings.pickle"), "rb").read())
        self.current_num_faces = 0
        self.current_num_annotations = 0
        self.faces = set()
        
        self.seq = 0

        self.park_location = "green"
        
        printStatusMsgs.ok("Face detection initialized")
        
        # printStatusMsgs.info("Initializing variables for rings and cylinders")
        
        self.ring_marker_array = MarkerArray()
        self.ring_marker_num = 1
        
        self.cylinder_marker_array = MarkerArray()
        self.cylinder_marker_num = 1
        
        self.parking_markers_array = MarkerArray()
        """
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        ""
        """
        self.ring_markers_pub = rospy.Publisher('ring_markers', MarkerArray, queue_size=1000)
        self.cylinder_markers_pub = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=1000)    
        self.parking_markers_pub = rospy.Publisher('parking_markers', MarkerArray, queue_size=1000)
        
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        
        self.rings = list()
        self.cylinders = list()

        self.susBarrels = ["yellow", "green"]
        
        self.number_of_rings = 3
        self.number_of_cylinders = 1
        
        self.state = "get_next_waypoint"
        self.botLocationX = 0.0
        self.botLocationY = 0.0
        self.quarternion = [None] * 4
        
        self.objectLocationX = 0.0
        self.objectLocationY = 0.0
        
        # printStatusMsgs.info("initializing speech engine")
        
        self.SpeechEngine = pyttsx3.init()
        self.SpeechEngine.setProperty("rate", 160)
        self.SpeechEngine.say("System online")
        self.SpeechEngine.runAndWait()

        self.st = SpeechTranscriber()


        
        
        printStatusMsgs.ok("speech engine initialized")
        
        self.wpGenerator = WaypointGenerator()
        #wait, than publish all initial values
        rospy.sleep(1)
        printStatusMsgs.info("Moving arm to erection position")
        self.arm_user_command_pub.publish(String("erection"))
        # self.arm_user_command_pub.publish(String("barrelsearch"))
        
        printStatusMsgs.ok("Initialization complete")


    def arm_camera_callback(self, image, depth):
        if self.state != "search_barrels":
            return
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            print(e)


        name = detect_poster_faces(rgb_image)

        if name != '':
            print("Found person!", name)


        for sname, poster_face, digits_and_color in self.suspects:
            if name == sname:
                self.SpeechEngine.say("Stop right there criminal scum, I'm taking you to jail")
                self.SpeechEngine.runAndWait()
                if(digits_and_color[1] != ''):
                    self.park_location = digits_and_color[1]
                self.state = "parking"


    def find_faces(self):
        #print('I got a new image!')

        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)




        image = rgb_image
        boxes = face_recognition.face_locations(image, model="hog")
        encodings = face_recognition.face_encodings(image, boxes)

        names = ""

        # poster = detect_posters(rgb_image)

        # if poster != '' and poster != 'Non-poster' and poster != 'Unknown':
        #     poster_face = detect_poster_faces(rgb_image)

        #     digits_and_color = extract_digits_and_color(rgb_image)

        #     if  (poster, poster_face, digits_and_color) not in self.suspects:
        #         if poster and poster_face not in ['unknown', ''] and digits_and_color[0] != '':
        #             self.suspects.append((poster, poster_face, digits_and_color))
        #             print("Suspect added")
        #             print(self.suspects)
        #             self.SpeechEngine.say("Suspect added")
        #             self.SpeechEngine.runAndWait()
        #             return

        poster = detect_posters(rgb_image)
        if poster == 'Poster':
            poster_face = detect_poster_faces(rgb_image)

            digits_and_color = extract_digits_and_color(rgb_image)

            print(poster, poster_face, digits_and_color)

                

            if  (poster, poster_face, digits_and_color) not in self.suspects:
                if poster_face not in ['unknown', '']:
                    self.suspects.append((poster, poster_face, digits_and_color))
                    print("Suspect added")
                    print(self.suspects)
                    self.SpeechEngine.say("Suspect added")
                    self.SpeechEngine.runAndWait()
                else:
                    self.state = "approach_poster"
            return


        if len(encodings) > 0:
            for encoding in encodings:
                matches = face_recognition.compare_faces(self.data["encodings"], encoding)
                name = "Unknown"
                if True in matches:
                    matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                    counts = {}
                    for i in matchedIdxs:
                        name = self.data["names"][i]
                        counts[name] = counts.get(name, 0) + 1
                    name = max(counts, key=counts.get)

                names = name



                
                for ((top, right, bottom, left), name) in zip(boxes, names):
                    cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
                    y = top - 15 if top - 15 > 15 else top + 15
                dist = depth_image[int((top + bottom) / 2), int((left + right) / 2)]
                
                try:
                    pose = self.get_pose((left, right, top, bottom), dist , rgb_image_message.header.stamp)
                    self.addFaceMarkerEstimation(pose)
                except:
                    pass
        
            if names not in self.faces:
                self.faces.add(names) # add face anyway, so it doesn't get added again
                print("Found new face")
                for ((top, right, bottom, left), name) in zip(boxes, names):
                    cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
                    y = top - 15 if top - 15 > 15 else top + 15

            
                    
                #calculate distance to the center of the face
                dist = depth_image[int((top + bottom) / 2), int((left + right) / 2)]
                
                pose = self.get_pose((left, right, top, bottom), dist, rgb_image_message.header.stamp)
                print(pose)
                if pose is not None and not self.faceMarkerIsNearAnotherMarker(pose, 0.25): #if the face is not too close to another face don't create a marker
                    self.current_num_faces += 1
                    self.addFaceMarker(pose)
                    self.objectLocationX = pose.position.x
                    self.objectLocationY = pose.position.y
                    if self.state != "approach_poster":
                        self.state = "approach_face"
            self.processed_image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))


    
    
    def faceMarkerIsNearAnotherMarker(self, pose, range):
        for marker in self.marker_array.markers:
            if abs(pose.position.x - marker.pose.position.x) < range and abs(pose.position.y - marker.pose.position.y) < range:
                return True
        return False

    def odom_callback(self, odom):
        return odom
    
    def mover(self):
        
        pointsX = [-0.129, -0.376, -1.051,  0.004,  0.879, -0.238, 2.714,  1.957,  3.025,  3.288, 1.326, 1.236, 1.085, 2.055, 2.169, 1.655, 0.912, -0.477, -0.058, 0.071, -1.025]
        pointsY = [0.865,  0.3750,  0.428, -0.704, -0.937, -0.934,-0.171, -1.084, -0.169, -0.142, 1.067, 0.913, 0.097, 1.072, 2.557, 2.863,  2.721, 2.783, 2.700, 2.851, 1.822]
        #8, 9, 13
        i = 0
        next_goal = None
        
        rate = rospy.Rate(1)
        
        while self.pose_pub.get_num_connections() < 1:
            rate.sleep()
        
        while not rospy.is_shutdown():
        
            if len(self.rings) >= self.number_of_rings and len(self.cylinders) >= self.number_of_cylinders and self.state != "end" and self.state != "parking" and self.state != "precise_parking" and self.state != "search_barrels":
                printStatusMsgs.info("starting parking procedure (all conditions met)")
                # 
                self.state = "search_barrels"
                # self.cancel_goal_publisher.publish(GoalID())
                # rospy.sleep(1)

                # for ring in self.rings:
                #     if ring.color == self.park_location:
                #         self.parkingX = ring.pose.pose.position.x
                #         self.parkingY = ring.pose.pose.position.y
                #         #comes close to the green ring
                #         # self.move_to_next(parkingX, parkingY, "parking")
                #         self.state = "parking"
         
               
            elif self.state == "get_next_waypoint":
                   
                if i == len(pointsX):
                    # i = 0
                    printStatusMsgs.info("starting parking procedure (all points visited)")
                    self.arm_user_command_pub.publish(String("extend"))
                    self.state = "search_barrels"
                    # self.cancel_goal_publisher.publish(GoalID())
                    rospy.sleep(1)

                    # for ring in self.rings:
                    #     if ring.color == self.park_location:
                    #         self.parkingX = ring.pose.pose.position.x
                    #         self.parkingY = ring.pose.pose.position.y
                    #         #comes close to the green ring
                    #         # self.move_to_next(parkingX, parkingY, "parking")
                    #         self.state = "parking"
                else:
                    self.move_to_next(pointsX[i], pointsY[i], "moving")
                    i += 1
                    print(i)

            elif self.state == "approach_face":
                self.move_to_nearest_accessible_point(self.objectLocationX, self.objectLocationY, "get_next_waypoint", 0.5)
                self.SpeechEngine.say("Where are they hiding?")
                self.SpeechEngine.runAndWait()

                cylinders = []
                while not rospy.is_shutdown():
                    cylinders = self.st.recognize_speech()
                    if(len(cylinders) == 2):
                        print('I recognized these barrels:', cylinders)
                        self.SpeechEngine.say("thank you for your help")
                        self.SpeechEngine.runAndWait()
                        self.susBarrels = cylinders
                        #Add annotation on top of the cylinders of the colors
                        # for cylinder in self.cylinders:
                        #     if cylinder.color in self.susBarrels:
                        #         self.addAnnotation(cylinder.pose , "SUS")
                        break
                    if len(cylinders)==1 and cylinders[0] == "no":
                        print("no information")
                        self.SpeechEngine.say("Thank you anyway")
                        self.SpeechEngine.runAndWait()
                        break
                    rospy.sleep(5)

                self.state = "get_next_waypoint"

            elif self.state == "approach_poster":
                print("approaching poster")
                self.move_to_nearest_accessible_point(self.objectLocationX, self.objectLocationY, "get_next_waypoint", 0.5)
                self.state = "get_next_waypoint"




            elif self.state == "parking":
                self.move_to_nearest_accessible_point(self.parkingX, self.parkingY, "precise_parking")
                printStatusMsgs.error("failed to park, still attempting precise parking")
                # rospy.sleep(5)
                self.state= "precise_parking"
                self.move_to_parking_zone()
            
            elif self.state == "precise_parking":
                printStatusMsgs.info("precise parking")
                self.move_to_parking_zone()

                
            
            elif self.state == "ring_found":
                #print("Numb of rings: ", len(self.rings))
                #self.cancel_goal_publisher.publish(GoalID())
                #rospy.sleep(1)
                #ringy = self.rings[-1]
                #print("Hello", ringy.color, "ring")
                self.state = "get_next_waypoint"    
                
            elif self.state == "end":
                printStatusMsgs.info("Mission complete")
                self.SpeechEngine.say("Mission complete")
                self.SpeechEngine.runAndWait()
                rospy.signal_shutdown("Mission complete")
            

            elif self.state == "search_barrels":
                printStatusMsgs.info("searching for criminals in barrels")
                self.arm_user_command_pub.publish(String("barrelsearch"))
                while len(self.susBarrels) > 0 and not rospy.is_shutdown():
                    target = self.get_cylinder_from_color(self.susBarrels.pop(0))
                    if(target is None):
                        printStatusMsgs.error("I don't know where the barrel is")
                        continue
                    printStatusMsgs.info("target: "+ target.color + "barrel")
                    self.move_to_nearest_accessible_point(target.pose.position.x, target.pose.position.y, "approach_face", 0.5)

                for ring in self.rings:
                    if ring.color == self.park_location:
                        self.parkingX = ring.pose.pose.position.x
                        self.parkingY = ring.pose.pose.position.y
                        #comes close to the green ring
                        # self.move_to_next(parkingX, parkingY, "parking")
                        # self.state = "parking"
                self.arm_user_command_pub.publish(String("extend"))
                self.state = "parking"
                
            self.find_faces()
            rate.sleep()
    
    
    def find_rings_callback(self, data):

        place_marker = True

        if self.rings:
            for ring in self.rings:
                if (np.sqrt((data.position.pose.position.x - ring.pose.pose.position.x) ** 2 + (
                        data.position.pose.position.y - ring.pose.pose.position.y) ** 2) < 1):
                    place_marker = False

        if place_marker and len(self.rings) < self.number_of_rings:
            pose = data.position
            color = data.color
            ring = Ringy(pose, self.ring_marker_num, color)
            self.ring_marker_num += 1
            self.rings.append(ring)
            self.ring_marker_array.markers.append(ring.to_marker())
            self.objectLocationX = data.position.pose.position.x
            self.objectLocationY = data.position.pose.position.y
            print("Hello", color, "ring")
            # self.SpeechEngine.say("Hello " + color + " ring")
            # self.SpeechEngine.runAndWait()
            self.state = "ring_found"

        self.ring_markers_pub.publish(self.ring_marker_array)
    
    
    def find_cylinders_callback(self, data):
        
        place_marker = True
        
        if self.cylinders :
            for cylinder in self.cylinders :
                if not(abs(data.position.point.x - cylinder.pose.point.x) > 1 or abs(data.position.point.y - cylinder.pose.point.y) > 1):
                    place_marker = False
                
        if place_marker:
            pose = data.position
            color = data.color
            cylinder = Cylindy(pose, self.cylinder_marker_num, color)
            print("Hello", color, "cylinder")
            # self.SpeechEngine.say("Hello " + color + " cylinder")

            self.cylinder_marker_num += 1
            self.cylinders.append(cylinder)
            self.cylinder_marker_array.markers.append(cylinder.to_marker())
                    
            
        self.cylinder_markers_pub.publish(self.cylinder_marker_array) 
        
               
        
    def move_to_next_waypoint(self, x_goal, y_goal, next_state):
    
        if next_state == "end":
            self.state = "end"
            print(" :) ")
        elif next_state == "moving":
            self.state = "moving"
            print("Moving")
        elif next_state == "parking":
            self.state = "parking"
            print("Parking")
            #after parking near the green ring go into the real parking state and look for the ring
            # on the ground -> place the next waypoint in the center of the circle on the ground
            # go into the parked state and stay there
            
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        
        
    def move_to_next(self, x_goal, y_goal, next_state):
        
        if next_state == "end" or self.state == "end":
            self.state = "end"
            print(" :) ")
        elif next_state == "moving":
            self.state = "moving"
            print("Moving")
        elif next_state == "parking":
            self.state = "park"
            #print("Parking")
            #self.park(x_goal,y_goal)
            
                    
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x_goal
        msg.pose.position.y = y_goal
        msg.pose.orientation.w = 1.0
        
        self.SpeechEngine.runAndWait()
        self.pose_pub.publish(msg)
        
        
        
    def result_sub_callback(self, data):
    
        res_status = data.status.status
        #print("Res status: ", res_status)
        #print("Self state: ", self.state)
        
        if self.state == "moving":
            if res_status == 3 or res_status == 4:
                self.seq += 1
                self.state = "get_next_waypoint"
        
        elif self.state == "ring_found":
            self.state = "get_next_waypoint"
            
    
    def get_pose(self, coords, dist, stamp):
        # Calculate the position of the detected face

        fx = 554.2546911911879  # kinect focal length in pixels (taken from camera_info)
        fy = 554.2546911911879
        cx = 320.5  # image center
        cy = 240.5

        x1, x2, y1, y2 = coords
        face_x = (x1 + x2) / 2
        face_y = (y1 + y2) / 2

        # Calculate the angle between the camera and the face
        alpha = np.arctan2(face_x - cx, fx)
        beta = np.arctan2(face_y - cy, fy)

        # Get the position of the face in the camera coordinate system
        x = dist * np.cos(beta) * np.sin(alpha)
        y = dist * np.sin(beta)
        z = dist * np.cos(beta) * np.cos(alpha)

        # Define a stamped message for transformation - in the "camera_rgb_optical_frame"
        point_s = PointStamped()
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp
        point_s.point.x = x
        point_s.point.y = y
        point_s.point.z = z

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        return pose
    
    def addAnnotation(self, pose, text):
        marker = Marker()
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation = Quaternion(*q)
        marker.pose.position = pose
        marker.header.frame_id = "map"
        marker.ns = "face_localizer"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.id = self.current_num_annotations
        self.current_num_annotations += 1
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = text
        self.annotation_array.markers.append(marker)
        self.annotation_markers_pub.publish(self.annotation_array)

    
    def addFaceMarker(self, pose):
        marker = Marker()
        q = tf.transformations.quaternion_from_euler(0, 0, 0)  # Roll, pitch, yaw in radians
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation = Quaternion(*q)
        marker.pose = pose
        marker.header.frame_id = "map"
        marker.ns = "face_localizer"
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://exercise6/meshes/face_marker.dae"
        marker.action = Marker.ADD
        marker.id = self.current_num_faces
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        text_marker = copy.deepcopy(marker)
        text_marker.text = "face_" + str(self.current_num_faces)
        text_marker.id = self.current_num_faces+1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.pose.position.z = 1.0
        self.marker_array.markers.append(marker)
        self.text_marker_array.markers.append(text_marker)
        self.markers_pub.publish(self.marker_array)
        self.text_markers_pub.publish(self.text_marker_array)
        
    def addFaceMarkerEstimation(self, pose):
        marker = Marker()
        q = tf.transformations.quaternion_from_euler(0, 0, 0)  # Roll, pitch, yaw in radians
        marker.header.stamp = rospy.Time.now()
        marker.pose.orientation = Quaternion(*q)
        marker.pose = pose
        marker.header.frame_id = "map"
        marker.ns = "face_localizer"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = self.est_count
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_array_est.markers.append(marker)
        self.face_markers_est_pub.publish(self.marker_array_est)
        self.est_count += 1
    

    def move_to_nearest_accessible_point(self, x, y, nextState, min_distance=float("inf")):
        map = rospy.wait_for_message("/map", OccupancyGrid)
        map_info = map.info
        map_data = map.data
        
        # Calculate the cell indices corresponding to the given coordinates
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        x_idx = int((x - origin_x) / resolution)
        y_idx = int((y - origin_y) / resolution)
        
        detecting = True
        
        # Define a circular region of interest around the given point
        roi_radius = 1.5 # meters
        roi_width = int(roi_radius / resolution)
        roi_center_idx = y_idx * map_info.width + x_idx
        roi_indices = set()
        for i in range(-roi_width, roi_width + 1):
            for j in range(-roi_width, roi_width + 1):
                idx = roi_center_idx + i + j * map_info.width #!!! check if this is correct
                if idx >= 0 and idx<len(map_data): #!!! check if this is correct
                    cell_value = map_data[idx]
                    if cell_value == 0:
                        flag = False
                        #if each adjecent cell in a radius of 5 is free, add it to the set of indices
                        for k in range(-6, 7):
                            for l in range(-6, 7):
                                idx2 = idx + k + l * map_info.width
                                if idx2 >= 0 and idx2<len(map_data):                                
                                    if map_data[idx2] != 0:
                                        flag = True
                                        break
                            if flag:
                                break
                        if not flag:
                            roi_indices.add(idx)
                                
                    
        # Find the nearest accessible point in the region of interest
        # min_distance = float("inf")
        nearest_idx = None
        for idx in roi_indices:
            x_pos = (idx % map_info.width) * resolution + origin_x
            y_pos = (idx // map_info.width) * resolution + origin_y
            distance = math.sqrt((x_pos - x)**2 + (y_pos - y)**2)
            if distance > 0.3 and distance < min_distance:
                min_distance = distance
                nearest_idx = idx

        if nearest_idx is None:
            rospy.logwarn("No accessible point found within the region of interest")
            return
                
        #Calculate the coordinates of the nearest accessible point to PoseStamped
        nearest_pose = PoseStamped()
        nearest_pose.header.frame_id = "map"
        nearest_pose.pose.position.x = (nearest_idx % map_info.width) * resolution + origin_x
        nearest_pose.pose.position.y = (nearest_idx // map_info.width) * resolution + origin_y
        nearest_pose.pose.orientation.w = 1.0
        
        # set a marker at the nearest accessible point
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.pose = nearest_pose.pose
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://exercise6/meshes/flag.dae"
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        marker.scale = Vector3(0.25, 0.25, 0.25)
        marker.id = 0
        marker.action = Marker.ADD
        self.parking_markers_array.markers.append(marker)
        self.parking_markers_pub.publish(self.parking_markers_array)
        
        client = SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose = nearest_pose
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_result():
            printStatusMsgs.ok("moved to nearest accessible point")
            self.state = nextState
        else:
            printStatusMsgs.error("failed to move to nearest accessible point")
            self.state = nextState

        self.parking_markers_array = MarkerArray()
        self.parking_markers_pub.publish(self.parking_markers_array)
            

    
    def get_cylinder_from_color(self, color):
        print(color)
        for cylinder in self.cylinders:
            print("cylinder color ", cylinder.color)
            if cylinder.color == color:
                return cylinder
        return None
    
            
        
    #Detect a black circle on the ground and move the robot inside it
    def move_to_parking_zone(self):
        twist_pub = self.twist_pub
        image_sub = self.arm_camera_sub
        bridge = self.bridge

        while not rospy.is_shutdown():
            
            # print("searching for a parking spot")
            
            img = rospy.wait_for_message("/arm_camera/rgb/image_raw", Image)
            
            frame = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 30])
            
            mask = cv2.inRange(hsv, lower_black, upper_black)
            
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=1)
            

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            MIN_CONTOUR_AREA = 500

            filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
            if len(filtered_contours) > 0:
                max_contour = max(filtered_contours, key=cv2.contourArea)

            #if len(contours) > 0:
                # Find the contour with the largest area
                #max_contour = max(contours, key=cv2.contourArea)
                print("found circle", end = "\r")

                # Fit a circle to the contour
                (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour)
                center = (int(center_x), int(center_y))
                print(center)

                # Draw a circle at the center of the circle for visualization purposes
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                print(center_x - frame.shape[1] / 2)
                # If the center of the circle is not within a certain range of the center of the image, move the robot to the center of the circle
                if abs(center_x - frame.shape[1] / 2) > 15:
                    twist = Twist()
                    print("move closer to center")
                    twist.linear.x = 0.1 # replace with your desired linear velocity
                    twist.angular.z = -((center_x - frame.shape[1] / 2)/100.0)# replace with your desired angular velocity scaling factor
                    twist_pub.publish(twist)
                    rospy.sleep(1)
                #elif 15 >= abs(center_x - frame.shape[1] / 2) > 10:
                    # turn the robot
                    #twist = Twist()
                    #twist.linear.x = 0.0
                    #twist.angular.z = 0.5
                    #twist_pub.publish(twist)
                    #rospy.sleep(0.5)
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    twist_pub.publish(twist)
                    print("I stopped")
                    self.arm_user_command_pub.publish(String("retract"))
                    self.status = "end"
                    self.state = "end"

                    # Save the coordinates of the center of the circle
                    global circle_center
                    circle_center = center
            else:
                #spin the robot around
                twist = Twist()
                #twist.linear.x = 0.1
                twist.angular.z = -1.0
                twist_pub.publish(twist)
                
                
        rospy.spin()



                
            
                

        
class Ringy:

    def __init__(self, pose, rId, color):
        self.pose = pose
        self.id = rId
        self.color = color
        if self.color == "red":             
            self.rgba = [1,0,0,1]         
        elif self.color == "green":             
            self.rgba = [0,1,0,1]         
        elif self.color == "blue":             
            self.rgba = [0,0,1,1]         
        elif self.color == "cyan":             
            self.rgba = [0,1,1,1]         
        elif self.color == "magenta":             
            self.rgba = [1,0,1,1]         
        elif self.color == "yellow":             
            self.rgba = [1,1,0,1]         
        elif self.color == "black":             
            self.rgba = [0,0,0,1]         
        elif self.color == "gray":
            self.rgba = [0,0,0,1]
    
    def to_marker(self):
        marker = Marker()
        #marker.header.stamp = self.pose.header.stamp
        #marker.header.frame_id = self.pose.header.frame_id
        marker.header.frame_id = "map"
        marker.type = Marker.MESH_RESOURCE;
        marker.mesh_resource = "package://exercise6/meshes/ring.stl";
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.scale = Vector3(0.1, 0.1, 0.1)
        #marker.scale = Vector3(1, 1, 1)
        marker.pose = self.pose.pose
        #marker.color = ColorRGBA(0, 1, 0, 1)
        marker.color = ColorRGBA(self.rgba[0],self.rgba[1],self.rgba[2],self.rgba[3])
        marker.id = self.id
        return marker
    
        
class Cylindy:

    def __init__(self, pose, cId, color):
        self.pose = pose
        self.id = cId
        self.color = color
        if self.color == "red":             
            self.rgba = [1,0,0,1]         
        elif self.color == "green":             
            self.rgba = [0,1,0,1]         
        elif self.color == "blue":             
            self.rgba = [0,0,1,1]         
        elif self.color == "cyan":             
            self.rgba = [0,1,1,1]         
        elif self.color == "magenta":             
            self.rgba = [1,0,1,1]         
        elif self.color == "yellow":             
            self.rgba = [1,1,0,1]         
        elif self.color == "black":             
            self.rgba = [0,0,0,1]         
        elif self.color == "gray":
            self.rgba = [0,0,0,1]

    def to_marker(self):
    
        marker = Marker()
        #marker.header.stamp = self.point_world.header.stamp
        #marker.header.frame_id = self.point_world.header.frame_id
        marker.header.frame_id = "map"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.scale = Vector3(0.25, 0.25, 0.25)
        marker.pose.position.x = self.pose.point.x
        marker.pose.position.y = self.pose.point.y
        marker.pose.position.z = self.pose.point.z
        #marker.color = ColorRGBA(0, 1, 0, 1)
        marker.color = ColorRGBA(self.rgba[0],self.rgba[1],self.rgba[2],self.rgba[3])
        marker.id = self.id
        return marker
    
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
class printStatusMsgs:
    def info(msg):
        # print("["+bcolors.OKCYAN +"INFO"+bcolors.ENDC+"] "+msg, end="\r")
        print("["+bcolors.OKCYAN +"INFO"+bcolors.ENDC+"] "+msg)
        
        
    def ok(msg):
        sys.stdout.write('\033[2K\033[1G')
        print("["+bcolors.OKGREEN +"OK"+bcolors.ENDC+"] "+msg)
    
    def error(msg):
        sys.stdout.write('\033[2K\033[1G')
        print("["+bcolors.FAIL +"ERROR"+bcolors.ENDC+"] "+msg)

class WaypointGenerator:
    def __init__(self):
        # Initialize the node

        # Initialize the map and waypoint list
        self.map_msg = None
        self.waypoints = []
        
        printStatusMsgs.info("Initializing waypoint generator...")
        # Subscribe to the map topic
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Spin the node
        # rospy.spin()
        
        sys.stdout.write('\033[2K\033[1G')
        printStatusMsgs.ok("Waypoint generator initialized")

    def map_callback(self, map_msg):
        self.map_msg = map_msg

    def get_neighbors(self, x, y):
        neighbors = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx = x + dx
                ny = y + dy
                if nx >= 0 and nx < self.map_msg.info.width and ny >= 0 and ny < self.map_msg.info.height:
                    index = nx + ny * self.map_msg.info.width
                    if self.map_msg.data[int(index)] == 0:
                        neighbors.append((nx, ny))
        return neighbors

    def dfs(self, x, y, visited, path):
        visited.add((x, y))
        path.append((x, y))
        neighbors = self.get_neighbors(x, y)
        for nx, ny in neighbors:
            if (nx, ny) not in visited:
                self.dfs(nx, ny, visited, path)
        if len(path) == len(self.waypoints) and (x, y) in self.get_neighbors(*self.waypoints[0]):
            self.waypoints = path

    def get_waypoints(self):
        # Wait for the map to be available
        
        printStatusMsgs.info("Generating waypoints")        
        
        while self.map_msg is None:
            rospy.sleep(0.1)

        # Get the map resolution and origin
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        
        printStatusMsgs.info("resolution: " + str(resolution) + "\norigin x: " + str(origin_x) +"\norigin y:" +str(origin_y)+"\n")
        

        # Iterate through the map and add unoccupied cells to the waypoint list
        self.waypoints = []
        for i, cell in enumerate(self.map_msg.data):
            if cell == 0:
                # Compute the (x,y) coordinates of the current cell
                x = i % self.map_msg.info.width
                y = i // self.map_msg.info.width
                x = x * resolution + origin_x
                y = y * resolution + origin_y
                self.waypoints.append((x, y))

        # Perform depth-first search to find a path that visits every waypoint and ends at the starting point
        visited = set()
        path = []
        for x, y in self.waypoints:
            if (x, y) not in visited:
                self.dfs(x, y, visited, path)

        # Return the list of waypoints
        printStatusMsgs.ok("Waypoints generated")
        print(self.waypoints)
        return self.waypoints



def detect_posters(rgb_image):

    # data = pickle.loads(open("encodings_poster.pickle", "rb").read())
    data = pickle.loads(open(join(dirname(__file__), "encodings_poster.pickle"), "rb").read())

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

    # data = pickle.loads(open("encodings_poster_faces.pickle", "rb").read())
    data = pickle.loads(open(join(dirname(__file__), "encodings_poster_faces.pickle"), "rb").read())

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

def extract_digits_and_color(rgb_image):
    gray_img = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    myResult = cv2.inRange(gray_img, 55, 155)
    all_image_text = pytesseract.image_to_string(myResult).lower()
    all_image_text_splitted = all_image_text.split()
    
    somelist = [x for x in all_image_text_splitted if x.isdigit()]
    number = "".join(somelist)
    color = ""
    print("detected text: ", all_image_text_splitted)

    if "blue" in all_image_text_splitted:
        color = "blue"

    if "green" in all_image_text_splitted:
        color = "green"

    if "red" in all_image_text_splitted:
        color = "red"
    
    if "black" in all_image_text_splitted:
        color = "black"

    return number, color


class SpeechTranscriber:
    def __init__(self):
        self.sr = sr.Recognizer()

        # An interface to the default microphone
        self.mic = sr.Microphone()
        sr.Microphone.list_microphone_names()
        


    def recognize_speech(self):
        with self.mic as source:
            print('Adjusting mic for ambient noise...')
            self.sr.adjust_for_ambient_noise(source)
            print('SPEAK NOW!')
            audio = self.sr.listen(source)
        
        detectedColors = []
        print('I am now processing the sounds you made.')
        recognized_text = ''
        try:
            recognized_text = self.sr.recognize_google(audio).lower()
            print('I recognized this sentence:', recognized_text)
            for word in recognized_text.split():
                if word in ['red', 'green', 'blue', 'yellow']:
                    detectedColors.append(word);
                if word in  ["no", "don't", "know", "nothing" ]:
                    return ["no"]
            if len(detectedColors) == 0:
                print('I did not recognize any colors.')
            elif len(detectedColors) == 2:
                return detectedColors
                print('I recognized these cylinders:', detectedColors)
            else:
                print("please say two colors")
                return ""

        except sr.RequestError as e:
            print('API is probably unavailable', e)
        except sr.UnknownValueError:
            print('Did not manage to recognize anything.')
        
        return []


def main():

    move = Movement()
    rate = rospy.Rate(1)
    move.mover()


if __name__ == '__main__':
    main()
        