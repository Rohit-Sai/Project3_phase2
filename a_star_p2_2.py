#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as ros_node
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import time
from tf_transformations import quaternion_from_euler
import cv2 as cv
import heapq
import math

class Map:
    
    def __init__(self):
        # Visualizing the path by writing video
        self.height=200
        self.width=600
        frameSize = (self.width, self.height)
        fourcc = cv.VideoWriter_fourcc('m','p','4','v')
        self.out = cv.VideoWriter('A_star_rohith.mp4', fourcc, 60, frameSize)
       
       # Initialising variables
        self.map=None
        self.obs_map=None
        self.clearance=5
        
    # Function to create the map and obstacle map
    def create_map(self):
        # Main map for display
        map = np.full((self.height, self.width,3),(233,2230,221), dtype=np.uint8)

        # Obstacle map for path planning
        obs_map = np.full((self.height, self.width),255, dtype=np.uint8)
        
        # Draw rectangles with self.clearance
        rectangles=[((150-self.clearance, 0), (175+self.clearance, 100+self.clearance)),((250-self.clearance, 100-self.clearance), (275+self.clearance, 200)),
                ((0, 0), (600,self.clearance)),((0, 0), (self.clearance, 200)),((0, 200-self.clearance), (600, 200)),((600-self.clearance, 0), (600, 200))]
        for rectangle in rectangles:
            cv.rectangle(map, rectangle[0], rectangle[1],(222,228,203), -1)
            cv.rectangle(obs_map, rectangle[0], rectangle[1],(0,0,0), -1)

        # Draw circle with self.clearance
        cv.circle(map, (420, 80), 60+self.clearance,(222,228,203), -1)
        cv.circle(obs_map, (420, 80), 60+self.clearance,(0,0,0), -1)

        # Draw rectangles
        cv.rectangle(map, (150, 0), (175, 100), (2, 116, 189), -1)
        cv.rectangle(map, (250, 100), (275, 200),(2, 116, 189), -1)

        # Draw circle
        cv.circle(map, (420, 80), 60,(2, 116, 189), -1)

        return map,obs_map

    # Saving the map
    def generate_path_map(self):
        print("\nGenerating the map with path:")
        for i in range(len(path)-1):
            curve=self.nodes[path[i+1]][2]
            for k in range(len(curve)):
                cv.line(self.map,curve[k][0],curve[k][1],(0,0,255),1)
            cv.circle(self.map,(path[0][0],path[0][1]),2,(0,255,0),-1)
            cv.circle(self.map,(path[-1][0],path[-1][1]),2,(0,0,255),-1)
            
            self.out.write(self.map)
                
        for i in range(300):
            self.out.write(self.map)
        print("Map saved as A_Star_rohith.mp4\n")
        cv.waitKey(500)
        cv.destroyAllWindows()
        self.out.release()

class Node():
    
    def __init__(self):
        # Dictionary to store the nodes and their costs
        self.nodes={}    
        
        # Thresholds for the nodes
        self.threshold_x = 2
        self.threshold_y = 2
        self.threshold_theta = 30
        
        # Visited nodes matrix
        self.visited_nodes=np.zeros((int(self.height//self.threshold_y)+1,int(self.width//self.threshold_x)+1,int(360//self.threshold_theta)+1),dtype=np.uint8)
        
        # Start and end nodes(Initialised to None)
        self.start_node=None            
        self.end_node=None     
        
        self.ros_path = []
        self.gazebo_start_pose = (0,0,30)
            
    # Function to insert a node into the nodes dictionary
    def insert_node(self,cost=None,node=None,parent=None,curve=None,action=None):    
        if len(self.nodes)==0:
            self.nodes.update({(self.start_node):[None,0]})
            for i in range(self.width):
                for j in range(self.height):
                    if self.obs_map[j][i]==0 or self.obs_map[j][i]==128:
                        self.nodes.update({(i,j,0):[None,float('inf'),None,None]})
                        # self.visited_nodes[int(j/self.threshold_y)][int(i/self.threshold_x)][0]=1
        else:
            self.nodes.update({node:[parent,cost,curve,action]})
            
    # Action functions to move the point robot in 8 directions 
    def Actions(self,node):
        def action(left, right):
            curve = []
            t=0
            dt = 0.1
            x, y, th = node
            theta = th*np.pi/180
            cost = 0
            
            while t<1:
                xi,yi = int(x),int(y)
                dx = 0.5 *self.R * (left + right) * np.cos(theta) * dt
                dy = 0.5 *self.R * (left + right) * np.sin(theta) * dt
                dth = (self.R / self.L) * (right - left) * dt
                x += dx
                y += dy
                theta += dth
                t+=dt
                cost+=np.sqrt(dx**2+dy**2)
                curve.append(((xi,yi),(int(x),int(y))))
            x,y,th,cost = int(x),int(y),int(theta*180/np.pi),int(cost)
            
            if check_if_duplicate((x,y,th%360)):
                return None,None,None,None
            else:
                return (x,y,th%360),cost,curve,(left,right)
        
        def check_if_duplicate(node):
            x,y,th=node
            if not self.is_valid(node):
                return True
            if self.visited_nodes[int(y/self.threshold_y)][int(x/self.threshold_x)][int(th/self.threshold_theta)]==1:
                return True
            return False
        
        return [action(0,self.RPM1),action(self.RPM1,0),action(self.RPM1,self.RPM1),action(0,self.RPM2),
                action(self.RPM2,0),action(self.RPM2,self.RPM2),action(self.RPM1,self.RPM2),action(self.RPM2,self.RPM1)]

    # Returns the parent node for a given node    
    def get_parent(self,node):
        return self.nodes[node][0]

    # Function to move the robot in simulation
    def move_waffle_robot(self):
        print("\nMoving the robot to the goal node:")
        rate = self.create_rate(10) #10hz
        rclpy.spin_once(self)
        vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        print("Publisher created")
        vel_msg = Twist()
        
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        vel_pub.publish(vel_msg)
        rate.sleep()
        time.sleep(1)
        rclpy.spin_once(self)
        
        for i in range(1,len(self.ros_path)):
            rpm1,rpm2 = self.nodes[self.ros_path[i]][3]
            vel_x = 0.5 * self.R * (rpm1 + rpm2) /100
            
            vel_theta = self.R * (rpm1 - rpm2) / (0.9 * self.L)
            # Publishing the velocity
            vel_msg.linear.x = vel_x 
            vel_msg.angular.z = vel_theta
            vel_pub.publish(vel_msg)
            print(f"\nMoving to node {i}: Velocity: {vel_x} m/s, {vel_theta} rad/s")
            
            # Waiting for the robot to reach the next node
            t=0
            while t<1:
                t+=0.1
                
            rate.sleep()
            time.sleep(1)
            rclpy.spin_once(self)
        
        # To reach the last node
        vel_msg.linear.x = vel_x
        vel_msg.angular.z = - vel_theta
        vel_pub.publish(vel_msg)
        rate.sleep()
        time.sleep(1)
        rclpy.spin_once(self)
        
        # Stopping the robot
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        vel_pub.publish(vel_msg)
        time.sleep(1)
        print("\n\nRobot reached the goal node.")
        
class A_Star(Map,Node,ros_node):
    
    def __init__(self):
        super().__init__()
        super(Map,self).__init__()
        super(Node,self).__init__('A_Star_node')
        self.get_logger().info("A_Star node has been started.")
        
        # Robot Variables for TurtleBot Waffle
        self.radius=22
        self.RPM1=10
        self.RPM2=15
        self.R=3.3
        self.L=35.4
        
        # Getting user inputs
        self.get_user_inputs()
            
        # Inserting obstacle nodes into the nodes dictionary
        self.insert_node()
               
        # start_node and end_node
        cv.circle(self.map,(self.start_node[0],self.start_node[1]),2,(0,255,0),-1)
        cv.circle(self.map,(self.end_node[0],self.end_node[1]),2,(0,0,255),-1) 
        
    # A_Star function to generate the heap tree graph of the nodes using A_Star's Algorithm
    def A_Star_algorithm(self):
        open_list=[]
        closed_list=set()         
        tot_cost=self.total_cost(self.start_node)    
        heapq.heappush(open_list,(tot_cost,self.start_node))
        return_node=None
        c=0
        
        print("\nSearching for the path:")
        while open_list:
            c+=1
            _,current_node=heapq.heappop(open_list)
            current_c2c=self.nodes[current_node][1]  
            # time.sleep(1)
            if current_node in closed_list:
                continue
            closed_list.add(current_node)    

            self.visited_nodes[int(current_node[1]/self.threshold_y)][int(current_node[0]/self.threshold_x)][int(current_node[2]/self.threshold_theta)]=1

            if current_node!=self.start_node:
                curve=self.nodes[current_node][2]
                for i in range(len(curve)):
                    cv.line(self.map,curve[i][0],curve[i][1],(0,0,0),1)
                cv.line(self.map,curve[-1][1][:2],(current_node[0],current_node[1]),(0,0,0),1)
                if c%100==0:
                    self.out.write(self.map)
            
            if self.is_goal(current_node):
                open_list=None
                return_node=current_node
                break              
            
            for action in self.Actions(current_node):
                new_node,cost,curve,act=action
                if new_node is not None and self.is_valid(new_node):
                    if self.visited_nodes[int(new_node[1]/self.threshold_y)][int(new_node[0]/self.threshold_x)][int(new_node[2]/self.threshold_theta)]==0 and self.is_valid_curve(curve):
                        self.visited_nodes[int(new_node[1]/self.threshold_y)][int(new_node[0]/self.threshold_x)][int(new_node[2]/self.threshold_theta)]=1
                        self.insert_node(cost+current_c2c,new_node,current_node,curve,act)
                        tot_cost=self.total_cost(new_node)
                        heapq.heappush(open_list,(tot_cost,new_node))
                        
                        if current_c2c+cost<self.nodes[new_node][1]:
                            self.nodes[new_node][1]=current_c2c+cost
                            self.insert_node(current_c2c+cost,new_node,current_node,curve,act)
                            heapq.heappush(open_list,(self.total_cost(new_node),new_node))

        time.sleep(0.01)
        return return_node

    # Function to check if the node is valid
    def is_valid(self, node):
        # Check if the node is within map boundaries and not in an obstacle
        if node is None:
            return False
        within_bounds = node[0] in range(self.width) and node[1] in range(self.height)
        if not within_bounds:
            return False
        not_obstacle = self.obs_map[node[1]][node[0]] != 0
        return within_bounds and not_obstacle
    
    # Function to check if the curve is valid
    def is_valid_curve(self,curve):
        for i in range(len(curve)):
            if self.obs_map[curve[i][0][1]][curve[i][0][0]]==0 or self.obs_map[curve[i][1][1]][curve[i][1][0]]==0:
                return False
        return True
    
    # Function to check if the current node is the goal node
    def is_goal(self,node):
        if math.sqrt((node[0] - self.end_node[0])**2 + (node[1] - self.end_node[1])**2) <= 5:
            return True
        else:
            return False
    
    # Function to calculate the total cost of a node f(n)=h(n)+g(n): h(n)=euclidean heuristic from node to end node, g(n)=cost of node   
    def total_cost(self,node):
        i,j,_=node
        return math.sqrt((i-self.end_node[0])**2+(j-self.end_node[1])**2) +self.nodes[node][1]

    # Returns a path from the end_node to the start_node
    def construct_path(self):
        # Searching starts here
        path=[self.A_Star_algorithm()]
        if path[0] is not None:
            total_cost=self.nodes[path[0]][1]
            parent=self.get_parent(path[0])
            while parent is not None:
                path.append(parent)
                parent = self.get_parent(parent)
            path.reverse()
            print("Path found")
            return path,total_cost
        else:
            print("\nError: Path not found\nTry changing the orientation of the nodes")
            exit()

    # Getting user inputs
    def get_user_inputs(self):
        
        # Getting the RPMs of the wheels
        rpms=input("Enter the RPM of the wheels(Left:RPM1 Right:RPM2)(eg: 10 15): ")
        rpms=rpms.split()
        self.RPM1,self.RPM2=[int(i) for i in rpms]
        
        input_flag=True
        while input_flag:
            inp=input("Enter the clearance in mm: ")
            if not inp.isdigit():
                print("Please enter a valid integer value.")
                continue
            else:
                input_flag=False
                self.clearance = int(inp) / 10 + self.radius
                self.clearance = int(self.clearance)
                                
        # Creating the map and obstacle map
        self.map,self.obs_map=self.create_map()

        input_flag=True
        print(f"Consider clearance of {self.clearance}cm.\nPlease enter the start and end nodes in the format 0 1 30 for (0,1,30)")
        
        input_flag=True
        while input_flag:
            node=input(f"Start node:")
            try:
                x,y,th=[(float(i)) for i in node.split()]
                x,y,th=int(x*100)+50,int(y*100)+100,int(th)
            except:
                print("Please enter valid coordinates.")
                continue
            if x not in range(self.width) or y not in range(self.height) or th % 30 != 0:
                print("Please enter valid coordinates.")
                continue
            elif self.obs_map[y][x]==0:
                print("Please enter a valid node(Node in obstacle place).")
            else:
                input_flag=False
                self.start_node=(x,y,th%360)
        
        input_flag=True
        while input_flag:
            node=input(f"End node:")
            try:
                x,y=[(float(i)) for i in node.split()]
                x,y=int(x*100)+50,int(y*100)+100
            except:
                print("Please enter valid coordinates.")
                continue
            if x not in range(self.width) or y not in range(self.height) or th % 30 != 0:
                print("Please enter valid coordinates.")
                continue
            elif self.obs_map[y][x]==0:
                print("Please enter a valid node(Node in obstacle place).")
            else:
                input_flag=False
                self.end_node=(x,y)
        
# Main function
if __name__ == "__main__":
    rclpy.init()
    start_time = time.time()

    # Creating the object of the class    
    a_Star=A_Star()

    # Generating the path and the total cost
    path,total_cost=a_Star.construct_path()   
    
    # Generating the path map
    a_Star.generate_path_map()
    a_Star.ros_path = path
    
    path = [(np.round(path[i][0]/100,2), np.round((-100 + path[i][1])/100),path[i][2]) for i in range(len(path))]

    # Printing the total cost and the path
    end_time = time.time()

    print(f"Time taken: {(end_time-start_time)/60} minutes.")
    print("\nPath cost: ",total_cost)
    print("\nPath: ",path)
    
    a_Star.get_logger().warn('Press any key to start the simulation')
    cv.imshow('Map',a_Star.map)
    cv.waitKey(0)

    # Moving the robot
    a_Star.move_waffle_robot()
    
    a_Star.destroy_node()
    rclpy.shutdown()