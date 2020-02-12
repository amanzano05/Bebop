#!/usr/bin/env python
#Source code for controller
#Importing libraries from rospy client API

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

#Created a class for Controller
class Controller:
    #contructor. Initialization of variables
    def __init__(self):
        rospy.init_node('Controller', anonymous=True) #Initialize ROS node
        #Variables used for coordinates for movement
        self.x = 0.0 
        self.y = 0.0
        self.z = 0.0
        
        self.throttle= 0.0 #Variable used for rotation 
        self.speed=0.02 #Variable used for speed

        # This expresses velocity in free space broken into its linear and angular parts. 
        # Geometry_msg/ twist message
        self.twist=Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))

        #Class to help run loops at a desired frequency.
        self.rate= rospy.Rate(20)
        
        #publishers for publishing our commands
        #publish a message of type std_msgs/Empty to takeoff topic
        self.pub_takeoff=rospy.Publisher("bebop/takeoff", Empty, queue_size=10 )
       
        #publish the message of type std_msgs/Empty to land topic
        self.pub_landing=rospy.Publisher("bebop/land", Empty, queue_size=10 )
        
        #publish messages of type geometry_msgs/Twist to cmd_vel topic
        self.pub_move=rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)
        
        #message empty
        self.message_empty=Empty()

        #Variable used to keep track of the drone status
        self.flying=False
        
        
    #Function for moving forward
    def front(self):
        #if the drone is flying 
        #set the speed to move forward 
        # and publish the geometry_msgs/Twist message to cmd_vel topic
        if self.flying:
            self.x=self.speed
            self.y=0.0
            self.z=0.0
            self.throttle=0.0
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nMoving forward..."+str(self.x))
        else:
            print("\nyou are not flying")
    
    #Function for moving backward
    #if the drone is flying 
    #set the speed to move backward 
    #and publish the geometry_msgs/Twist message to cmd_vel topic 
    def back(self):
        if self.flying:
            self.x=(-1*self.speed)
            self.y=0.0
            self.z=0.0
            self.throttle=0.0
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nMoving backward..."+str(self.x))
        else:
            print("\nyou are not flying")
    
    #Function for moving right
    #if the drone is flying 
    #set the speed to move right 
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def right(self):
        if self.flying:
            self.x=0.0
            self.y=(-1*self.speed)
            self.z=0.0
            self.throttle=0.0
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nMoving right...")
        else:
            print("\nyou are not flying")
    
    #Function for moving left
    #if the drone is flying 
    #set the speed to move left 
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def left(self):
        if self.flying:
            self.x=0.0
            self.y=self.speed
            self.z=0.0
            self.throttle=0.0
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nMoving left...")
        else:
            print("\nyou are not flying")
    
    #Functions for moving up
    #if the drone is flying 
    #set the speed to move up
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def up(self):
        if self.flying:
            self.x=0.0
            self.y=0.0
            self.z=self.speed
            self.throttle=0.0
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nMoving up...")
        else:
            print("\nyou are not flying")
    
    #Function for moving down
    #if the drone is flying 
    #set the speed to move down
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def down(self):
        if self.flying:
            self.x=0.0
            self.y=0.0
            self.z=(-1*self.speed)
            self.throttle=0.0
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nMoving down...")
        else:
            print("\nyou are not flying")
    
    #Function for landing (self landing)
    #if the drone is flying 
    #set the speed to zero, and land 
    #publish the message of type std_msgs/Empty to land topic
    def land(self):
        if self.flying:
            self.x=0.0
            self.y=0.0
            self.z=0.0
            self.throttle=0.0
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            self.pub_landing.publish(self.message_empty)
            print("\nLanding...")
            self.flying = False
        else:
            print("\nyou are not flying")
    
    #Function for taking off
    #Here is when we start to fly
    #if the drone is not flying 
    #set the speed to zero, and takeoff 
    #publish the message of type std_msgs/Empty to takeoff topic 
    def takeOff(self):
        if not self.flying:
            self.flying = True
            self.pub_takeoff.publish(self.message_empty)
            self.x=0.0
            self.y=0.0
            self.z=0.0
            self.throttle=0.0
            print("\nTaking off...")
        else:
            print("\nYou are flying")
    
    #Function for rotating left
    #if the drone is flying 
    #set the speed to rotate left
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def rotateLeft(self):
        if self.flying:
            self.x=0.0
            self.y=0.0
            self.z=0.0
            self.throttle=self.speed
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nRotating left...\n")
        else:
            print("\nyou are not flying\n")
    
    #Function for rotating right
    #if the drone is flying 
    #set the speed to rotate right
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def rotateRight(self):
        if self.flying:
            self.x=0.0
            self.y=0.0
            self.z=0.0
            self.throttle=(-1*self.speed)
            self.twist=Twist(Vector3(self.x,self.y,self.z), Vector3(0.0,0.0,self.throttle))
            self.pub_move.publish(self.twist)
            self.rate.sleep()
            print("\nRotating right...\n")
        else:
            print("\nyou are not flying\n")
            
    #Function to exit flying mode
    def exitMode(self):
        #Cannot exit while currently flying
        if self.flying:
            print("\nYou cannot exit the mode while flying...\n")
        else:
            print("\nExiting mode\n")
            
     #Function for incrementing speed 
    def incrementSpeed(self):
        if (self.speed>=0.00 and self.speed<0.99):
            self.speed+=0.01
        elif (self.speed>=0.99):
            self.speed=0.99
        print("speed++....."+str(self.speed))
            
        
    #Function for decrementing speed 
    def decrementSpeed(self):
        if (self.speed>=0.01 and self.speed<=1.00):
            self.speed-=0.01
        elif (self.speed<=0.1):
            self.speed=0.00
        print("speed--....."+str(self.speed))
            
    #Function for setting speed
    def setSpeed(self, speedValue=0.02):
        if (speedValue>0.00 and speedValue<1.00):
            self.speed=speedValue
        else:
            print("Value incorrect...")
        
    #Function to reset all current values
    def resetValues(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.throttle= 0.0
        self.speed=0.02

    #Function to set a waypoint
    def Setwaypoint(self):

        

    

