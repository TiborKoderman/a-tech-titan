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
from std_msgs.msg import String, Float32, Float64, Int32, Bool, ColorRGBA
from nav_msgs.msg import OccupancyGrid
from actionlib import SimpleActionClient

from os.path import dirname, join

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

class Movement:

    def __init__(self):
        
        printStatusMsgs.info("Initializing Movement") # type: ignore

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
        
        self.arm_camera_sub = Subscriber("/arm_camera/rgb/image_raw", Image)
        self.arm_depth_sub = Subscriber("/arm_camera/depth/image_raw", Image)
        
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
        self.marker_num = 0
        self.est_count = 0
        self.processed_image_pub = rospy.Publisher('processed_image', Image, queue_size=1000)
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.text_markers_pub = rospy.Publisher('text_face_markers', MarkerArray, queue_size=1000)
        self.face_markers_est_pub = rospy.Publisher('face_markers_est', MarkerArray, queue_size=1000)
        self.markers_pub.publish(self.marker_array)
        self.text_markers_pub.publish(self.text_marker_array)
        self.face_markers_est_pub.publish(self.marker_array_est)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.numb_of_faces = 7
        self.ap = argparse.ArgumentParser()
        self.data = pickle.loads(open(join(dirname(__file__), "encodings.pickle"), "rb").read())
        self.current_num_faces = 0
        self.faces = set()
        
        self.seq = 0
        
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
        
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        self.rings = list()
        self.cylinders = list()
        
        self.number_of_rings = 4
        self.number_of_cylinders = 4
        
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

        
        printStatusMsgs.ok("speech engine initialized")
        
        self.wpGenerator = WaypointGenerator()
        #wait, than publish all initial values
        rospy.sleep(1)
        printStatusMsgs.info("Moving arm to erection position")
        self.arm_user_command_pub.publish(String("erection"))
        
        printStatusMsgs.ok("Initialization complete")
    
    def odom_callback(self, odom):
        return odom
    
    def mover(self):
        
        pointsX = [-0.129, -0.376, -1.051, -0.968, -1.321, -0.288, 0.004, 0.879, 2.714, 3.025, 3.288, 1.326, 1.236, 2.055, 2.169, 1.655, 0.912, -0.477, -0.058, 0.071, -1.025]
        pointsY = [0.865, 0.3750, 0.428, 1.754, 2.044, 0.240, -0.704, -0.937, -0.171, -0.169, -0.142, 1.067, 0.913, 1.072, 2.557, 2.863,  2.721, 2.783, 2.700, 2.851, 1.822]
        
        i = 0
        next_goal = None
        
        rate = rospy.Rate(1)
        
        while self.pose_pub.get_num_connections() < 1:
            rate.sleep()
        
        while not rospy.is_shutdown():
        
            if self.number_of_rings == len(self.rings) and self.number_of_cylinders == len(self.cylinders) and self.state != "end" and self.state != "parking" and self.state != "precise_parking":
                printStatusMsgs.info("starting parking procedure")
                self.arm_user_command_pub.publish(String("extend"))
                self.state = "park"
                # self.cancel_goal_publisher.publish(GoalID())
                rospy.sleep(1)

                for ring in self.rings:
                    if ring.color == "green":
                        self.parkingX = ring.pose.pose.position.x
                        self.parkingY = ring.pose.pose.position.y
                        #comes close to the green ring
                        # self.move_to_next(parkingX, parkingY, "parking")
                        self.state = "parking"
         
               
            elif self.state == "get_next_waypoint":
                   
                if i == len(pointsX):
                    # i = 0
                    printStatusMsgs.info("starting parking procedure")
                    self.arm_user_command_pub.publish(String("extend"))
                    self.state = "park"
                    # self.cancel_goal_publisher.publish(GoalID())
                    rospy.sleep(1)

                    for ring in self.rings:
                        if ring.color == "green":
                            self.parkingX = ring.pose.pose.position.x
                            self.parkingY = ring.pose.pose.position.y
                            #comes close to the green ring
                            # self.move_to_next(parkingX, parkingY, "parking")
                            self.state = "parking"
                else:
                    self.move_to_next(pointsX[i], pointsY[i], "moving")
                    i += 1
            
            elif self.state == "parking":
                self.move_to_nearest_accessible_point(self.parkingX, self.parkingY)
                # printStatusMsgs.error("failed to park, still attempting precise parking")
                # rospy.sleep(5)
                self.state= "precise_parking"
            
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
            self.SpeechEngine.say("Hello " + color + " ring")
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
            self.SpeechEngine.say("Hello " + color + " cylinder")

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
            
        # elif self.state == "parking":
        #     if res_status == 3:
        #         self.state = "end"

    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        
        image_1 = depth_image / np.nanmax(depth_image)
        image_1 = image_1*255
        
        image_viz = np.array(image_1, dtype=np.uint8)
    
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
                if pose is not None and not self.faceMarkerIsNearAnotherMarker(pose, 1): #if the face is not too close to another face don't create a marker
                    self.current_num_faces += 1
                    self.addFaceMarker(pose)
                    # self.state = "approach_face"
            self.processed_image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    
    def faceMarkerIsNearAnotherMarker(self, pose, range):
        for marker in self.marker_array.markers:
            if abs(pose.position.x - marker.pose.position.x) < range and abs(pose.position.y - marker.pose.position.y) < range:
                return True
        return False
    
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
        
    def move_to_nearest_accessible_point(self, x, y):
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
        min_distance = float("inf")
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
            self.state = "precise_parking"
        else:
            printStatusMsgs.error("failed to move to nearest accessible point")
            self.state = "precise_parking"
            
    
            
        
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

            
            if len(contours) > 0:
                # Find the contour with the largest area
                max_contour = max(contours, key=cv2.contourArea)
                print("found circle", end = "\r")

                # Fit a circle to the contour
                (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour)
                center = (int(center_x), int(center_y))

                # Draw a circle at the center of the circle for visualization purposes
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # If the center of the circle is not within a certain range of the center of the image, move the robot to the center of the circle
                if abs(center_x - frame.shape[1] / 2) > 10:
                    twist = Twist()
                    twist.linear.x = 0.1 # replace with your desired linear velocity
                    twist.angular.z = (center_x - frame.shape[1] / 2) / 100.0 # replace with your desired angular velocity scaling factor
                    twist_pub.publish(twist)
                    rospy.sleep(0.1)
                else:
                    # Stop the robot
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    twist_pub.publish(twist)
                    
                    self.status = "end"

                    # Save the coordinates of the center of the circle
                    global circle_center
                    circle_center = center
            else:
                #spin the robot around
                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = 0.2
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
        print("["+bcolors.OKCYAN +"INFO"+bcolors.ENDC+"] "+msg) # type: ignore
        
        
    def ok(msg):
        sys.stdout.write('\033[2K\033[1G')
        print("["+bcolors.OKGREEN +"OK"+bcolors.ENDC+"] "+msg) # type: ignore
    
    def error(msg):
        sys.stdout.write('\033[2K\033[1G')
        print("["+bcolors.FAIL +"ERROR"+bcolors.ENDC+"] "+msg)  # type: ignore

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


def main():

    move = Movement()
    rate = rospy.Rate(1)
    move.mover()


if __name__ == '__main__':
    main()
        