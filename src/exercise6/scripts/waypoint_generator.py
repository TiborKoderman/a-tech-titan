#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, EmptyResponse
from exercise6.srv import Waypoints

class WaypointGenerator:
    def __init__(self):
        # Initialize the node
        rospy.init_node('waypoint_generator')

        # Initialize the map and waypoint list
        self.map_msg = None
        self.waypoints = []

        # Subscribe to the map topic
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Create the service
        self.service = rospy.Service('get_waypoints', Waypoints, self.get_waypoints)

        # Spin the node
        rospy.spin()

    def map_callback(self, map_msg):
        self.map_msg = map_msg

    def get_neighbors(self, x, y):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if nx < 0 or nx >= self.map_msg.info.width or ny < 0 or ny >= self.map_msg.info.height:
                    continue
                index = ny * self.map_msg.info.width + nx
                if self.map_msg.data[index] == 0:
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

    def get_waypoints(self, req):
        # Wait for the map to be available
        while self.map_msg is None:
            rospy.sleep(0.1)

        # Get the map resolution and origin
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

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
        Xwaypoints = []
        Ywaypoints = []
        
        Xwaypoints,Ywaypoints = [list(t) for t in zip(*self.waypoints)]
        
        waypoints = Waypoints()
        waypoints.x = Xwaypoints
        waypoints.y = Ywaypoints
        
        return waypoints

if __name__ == '__main__':
    wg = WaypointGenerator()