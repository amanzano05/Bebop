#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class Controller:
    def __init__(self):
        rospy.init_node('Controller', anonymous=True)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.throttle= 0.0
        self.speed=0.02
        self.twist=Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
        self.rate= rospy.Rate(20)
        self.pub_takeoff=rospy.Publisher("bebop/takeoff", Empty, queue_size=10 )
        self.pub_landing=rospy.Publisher("bebop/land", Empty, queue_size=10 )
        self.pub_move=rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)
        self.message_empty=Empty()
        self.flying=False
        
        
    
    def front(self):
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
            
    
    def exitMode(self):
        if self.flying:
            print("\nYou cannot exit the mode while flying...\n")
        else:
            print("\nExiting mode\n")
            
            
    def incrementSpeed(self):
        if (self.speed>=0.00 and self.speed<0.99):
            self.speed+=0.01
        elif (self.speed>=0.99):
            self.speed=0.99
        print("speed++....."+str(self.speed))
            
        
    
    def decrementSpeed(self):
        if (self.speed>=0.01 and self.speed<=1.00):
            self.speed-=0.01
        elif (self.speed<=0.1):
            self.speed=0.00
        print("speed--....."+str(self.speed))
            
    
    def setSpeed(self, speedValue=0.02):
        if (speedValue>0.00 and speedValue<1.00):
            self.speed=speedValue
        else:
            print("Value incorrect...")
        
    
    def resetValues(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.throttle= 0.0
        self.speed=0.02
        

    

