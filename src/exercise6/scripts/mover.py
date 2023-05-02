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

from os.path import dirname, join

from geometry_msgs.msg import PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, Twist
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
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
        """
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        ""
        """
        self.ring_markers_pub = rospy.Publisher('ring_markers', MarkerArray, queue_size=1000)
        self.cylinder_markers_pub = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=1000)    
        
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
        printStatusMsgs.info("moving arm into extended position")
        self.arm_user_command_pub.publish(String("extend"))
        
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
        
            if self.number_of_rings == len(self.rings) and self.number_of_cylinders == len(self.cylinders) and self.state != "end":
                self.state = "park"
                self.cancel_goal_publisher.publish(GoalID())
                rospy.sleep(1)

                for ring in self.rings:
                    if ring.color == "green":
                        parkingX = ring.pose.pose.position.x
                        parkingY = ring.pose.pose.position.y
                        #comes close to the green ring
                        self.move_to_next(parkingX, parkingY, "parking")
         
               
            elif self.state == "get_next_waypoint":
                   
                if i == len(pointsX):
                    i = 0
                
                self.move_to_next(pointsX[i], pointsY[i], "moving")
                i += 1
            
            elif self.state == "ring_found":
                #print("Numb of rings: ", len(self.rings))
                #self.cancel_goal_publisher.publish(GoalID())
                #rospy.sleep(1)
                #ringy = self.rings[-1]
                #print("Hello", ringy.color, "ring")
                self.state = "get_next_waypoint"       
                
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
            self.SpeechEngine.runAndWait()
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
            self.SpeechEngine.runAndWait()
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
        
        if next_state == "end":
            self.state = "end"
            print(" :) ")
        elif next_state == "moving":
            self.state = "moving"
            print("Moving")
        elif next_state == "parking":
            self.state = "parking"
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.seq = self.seq
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x_goal
            msg.pose.position.y = y_goal
            msg.pose.orientation.w = 1.0
            self.pose_pub.publish(msg)
            # Look for a non-green ring within a certain distance of the target coordinates
            search_radius = 0.5 # Modify this value to fit your specific requirements
            non_green_ring_found = False

            while not non_green_ring_found:
                for ring in self.rings:
                    distance_to_ring = math.sqrt((ring.pose.position.x - x_goal) ** 2 + (ring.pose.position.y - y_goal) ** 2)
                    if distance_to_ring <= search_radius and ring.color != "green":
                        non_green_ring_found = True
                        break

                # If a non-green ring was not found, adjust the search radius and move the robot slightly
                if not non_green_ring_found:
                    search_radius += 0.5 # Modify this value to fit your specific requirements
                    msg = PoseStamped()
                    msg.header.frame_id = "map"
                    msg.header.seq = self.seq
                    msg.header.stamp = rospy.Time.now()
                    msg.pose.position.x = x_goal + 0.1
                    msg.pose.position.y = y_goal + 0.1
                    msg.pose.orientation.w = 1.0
                    self.pose_pub.publish(msg)

            # Park the robot at the location of the non-green ring
            parkingX = ring.pose.position.x
            parkingY = ring.pose.position.y

            # Move the robot to the parking location
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.seq = self.seq
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = parkingX
            msg.pose.position.y = parkingY
            msg.pose.orientation.w = 1.0
            
            self.pose_pub.publish(msg)

            # Align the robot with the circular parking space
            # You can use computer vision techniques to detect the boundaries of the parking space and align the robot accordingly

            # Wait for the robot to stop moving
            while self.navigator.is_moving():
                rospy.sleep(0.1)

            # Update the state of the robot
            if next_state == "parking":
                self.state = "parked"
            else:
                self.state = "end"
                print("Parking")
            
                    
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x_goal
        msg.pose.position.y = y_goal
        msg.pose.orientation.w = 1.0
        
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
            
        elif self.state == "parking":
            if res_status == 3:
                self.state = "end"

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


def main():

    move = Movement()
    rate = rospy.Rate(1)
    move.mover()


if __name__ == '__main__':
    main()
        