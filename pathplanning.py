#! /usr/bin/env python

import rospy
import actionlib
from rll_planning_project.srv import *
from rll_planning_project.msg import *
from geometry_msgs.msg import Pose2D
from heapq import heappush, heappop # for priority queue
import math
import numpy as np


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def exploration(map_width, map_length,grid, check_srv, move_srv):
    validation = search_ancient_cells(map_width, map_length, check_srv)
    grid = update_cell(validation, grid)
    move(check_srv, move_srv)

    return grid

def distance2goal():
    return math.sqrt((other.x-self.x)**2 + (other.y-self.y)**2)

def update_cell(validation, grid):
    for i in range(len(validation)):
        grid[validation[i][0]][validation[i][1]] = validation[2] 
    return grid

def move(check_srv, move_srv):
    start_pose = Pose2D()
    distance2goal = np.array([]) 
    xy_move = np.array([[ 1, 0],
                        [-1, 0],
                        [ 0, 1],
                        [ 0,-1]])
    
    for i in range(len(xy_move)):
        xGoal = start_pose.x + xy_move[i][0]
        yGoal = start_pose.y + xy_move[i][1]
        #calculate distance to goal for every ancient cell  
        if (xGoal > 0 and xGoal < map_width) and (yGoal > 0 and yGoal < map_length):
            distance = math.sqrt((pose.x-xGoal)**2 + (pose.y-yGoal)**2)
            distance2goal = np.append(distance2goal, distance)
    
    for i in range(len(distance2goal)):
        #calculate cell with min distance
        min2goal = np.argmin(distance2goal)     
        #make movement towards goal
        xGoal = start_pose.x + xy_move[min2goal][0]
        yGoal = start_pose.y + xy_move[min2goal][1]
        tGoal = start_pose.t
        resp = check_srv(pose_check_start, pose_check_goal) # checking if the arm can move to the goal pose
        if resp.valid:
            rospy.loginfo("Valid pose")
            pose_move.x, pose_move.y, pose_move.theta = xGoal, yGoal, tGoal 
            # executing a move command towards the goal pose
            resp = move_srv(pose_move)
            break
        else:
            continue

    return None

def search_ancient_cells(map_width, map_length, check_srv):
    validation = np.array([])
    start_pose = Pose2D()
    xy_move = np.array([[ 1, 0],
                        [-1, 0],
                        [ 0, 1],
                        [ 0,-1]])
    theta_move = np.array([0, 0.78, 1.57])
    xStart, yStart, tStart = start_pose.x, start_pose.y, start_pose.theta 
    
    for i in range(len(xy_move)):
        ancient_x = xStart + movement[i][0]
        ancient_y = yStart + movement[i][1]  
        resp_theta = np.array([])
        if (ancient_x > 0 and ancient_x < map_width) and (ancient_y > 0 and ancient_y < map_length):
            for j in range(len(theta_move)):
                ancient_t = tStart + theta_move[j]
                pose_check_start.x, pose_check_start.y, pose_check_start.theta= xStart, yStart, tStart
                pose_check_goal.x, pose_check_goal.y, pose_check_goal.theta= ancient_x, ancient_y, ancient_t        
                resp = check_srv(pose_check_start, pose_check_goal) # checking if the arm can move to the goal pose
                if resp == 1:
                    resp_theta = np.append(resp_theta, resp)
            if len(resp_theta ) == 3:
                validation = np.append(validation, [xGoal, yGoal, resp], axis=0)
        else:
            continue
    
    return validation    

def make_heuristic(map_width, map_length, xGoal, yGoal):
    heuristic = np.zeros(map_width, map_length)
    cost = 0
    for i in range(map_width):
        for j in range(map_length):
            heuristic[i][j] = abs(xGoal - i) + abs(yGoal - j)
        
    return heuristic

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)
        
        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

def plan_to_goal(grid):

""" Plan a path from Start to Goal """
    pose_start = Pose2D()
    pose_goal = Pose2D()
    pose_check_start = Pose2D()
    pose_check_goal = Pose2D()
    pose_move = Pose2D()

    rospy.loginfo("Got a planning request")

    pose_start = req.start
    pose_goal = req.goal

    move_srv = rospy.ServiceProxy('move', Move)
    check_srv = rospy.ServiceProxy('check_path', CheckPath, persistent=True)

    ###############################################
    # Implement your path planning algorithm here #
    ###############################################

    # Input: map dimensions, start pose, and goal pose
    # retrieving input values  
    map_width = rospy.get_param('~map_width')
    map_length = rospy.get_param('~map_length')
    xStart, yStart, tStart = pose_start.x, pose_start.y, pose_start.theta
    xGoal, yGoal, tGoal = pose_goal.x, pose_goal.y, pose_goal.theta
    grid = np.zeros(map_width,map_length)
    
    
    # printing input values
    rospy.loginfo("map dimensions: width=%1.2fm, length=%1.2fm", map_width, map_length)
    rospy.loginfo("start pose: x %f, y %f, theta %f", xStart, yStart, tStart)
    rospy.loginfo("goal pose: x %f, y %f, theta %f", xGoal, yGoal, tGoal)

    start = (xStart, yStart)
    end = (xGoal, yGoal)


    #path = createpath(grid, xStart, yStart, xGoal, yGoal, check_srv)
    grid = exploration(map_width, map_length,grid, check_srv, move_srv)
    path = astar(grid, start, end)
    if path != None:
        rospy.loginfo("Valid path")
        for point in path:
            #rospy.loginfo("point: %f, %f, %f,", point.x, point.y, point.theta)
            pose_move.x, pose_move.y, pose_move.theta = point.x, point.y, point.theta 
            # executing a move command towards the goal pose
            resp = move_srv(pose_move)

    else:
        rospy.loginfo("No path found")
        
    ###############################################
    # End of Algorithm #
    ###############################################


class PathPlanner:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("plan_to_goal", PlanToGoalAction, self.execute, False)
        self.server.start()

    def execute(self, req):
        plan_to_goal(req)
        self.server.set_succeeded()



if __name__ == '__main__':
    rospy.init_node('path_planner')

    server = PathPlanner()

    rospy.spin()
