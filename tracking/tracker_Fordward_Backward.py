#!/usr/bin/env python


import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

#Creating a function that will print the control
def printControl():
    print("\n a:takeoff           t        u:up      Rotation   \n"
          "    |                  |          |                  \n"
          "    |              f-------h      |       o<----->p  \n"
          "    |                  |          |                  \n"
          " s:land                g         j:down              \n"
          "                                                     \n"
          " z:increment speed                                   \n"
          " x:decrement speed                                   \n"
          "                                                     \n"
          " b: Tracking mode                                    \n"
          " n: Tracking mode                                    \n"
          " m: Exit mode                                        \n"
          " q: Exit                                             \n")


#class for the tracker
class Tracker:
    #constructor
    def __init__(self):
        #variables for coordinates
        x=0
        y=0
        
        #speeds
        self.xSpeed=0
        self.ySpeed=0
        self.zSpeed=0
        self.rSpeed=0
        self.speed=0.07
        #flag used to keep track if the drone is flying
        self.flying=False
       
        #position and tracking
        self.centerWidth=0
        self.centerHeight=0
        self.offset=40
        self.topSpeed=0.25
        self.speedOffset=0.07
        self.offsetY=17
        self.topSpeedX=0.30
        self.speedOffsetX=0.07

        #initialize node
        rospy.init_node('tracker', anonymous=True)

        #publishers for publishing our commands
        #message empty
        self.message_empty=Empty()
        #publish a message of type std_msgs/Empty to takeoff topic
        self.pub_takeoff=rospy.Publisher("bebop/takeoff", Empty, queue_size=10 )
        #publish the message of type std_msgs/Empty to land topic
        self.pub_landing=rospy.Publisher("bebop/land", Empty, queue_size=10 )
        #publish messages of type geometry_msgs/Twist to cmd_vel topic
        self.pub_move=rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)
        # This expresses velocity in free space broken into its linear and angular parts. 
        # Geometry_msg/ twist message
        self.twist=Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))

        #create the bridge for video stream
        self.bridge=CvBridge()
        #create the subscriber using the bridge and callback function
        self.imageSub=rospy.Subscriber('bebop/image_raw', Image, self.camera_callback)
        #create the publisher for printing the direction
        self.direction_pub=rospy.Publisher('direction', String, queue_size=10)
        
        #flag for the first frame
        self.first=True
        self.stream=0 #variable used for the video stream
        self.bbox=0 # used for ROI selection
        self.track=0 #used for the tracker object
        self.tracker_type=0 # used for the type of the tracker 
        
        #for drawing the correct postion square
        self.correct_Xstart=0
        self.correct_Xend=0
        self.correct_Ystart=0
        self.correct_Yend=0
       
       #flag used for knowing when the object tracked is centered
        self.centered=False
        #flag used to keep track if the tracking mode is activated
        self.tracking=False
        #flag used to keep tack if we can exit the program with letter 'q'
        self.exit=False
        
        #for keyboard recording
        with Listener(on_press=self.on_press,
                      on_release=self.on_release) as listener:
            listener.join()  
        self.myKey=0
        
        
        
    #calculate and return the coordinates of the object being tracked with respect to 
    #to the center of the frame
    def calculateCoordinates(self, x, y):
        cX=self.centerWidth-x
        cY=self.centerHeight-y
        return cX, cY

    #calculate and return the Z speed and rotation R speed of the drone 
    #for the given coordinate
    def calculateSpeed(self, cX, cY):
        speedR=((((abs(cX)-self.offset)*(self.topSpeed-self.speedOffset))/(self.centerHeight-self.offset))+self.speedOffset)
        speedX=((((abs(cY)-self.offsetY)*(self.topSpeedX-self.speedOffsetX))/(self.centerWidth-self.offsetY))+self.speedOffsetX)
        if (cX<0):signX=-1
        else: signX=1
        if (cY<0):signY=-1
        else: signY=1
        return speedX*signY, speedR*signX


    #Function for moving forward
    def front(self):
        #if the drone is flying 
        #set the speed to move forward 
        # and publish the geometry_msgs/Twist message to cmd_vel topic
        if self.flying:
            self.xSpeed=self.speed
            self.ySpeed=0.0
            self.zSpeed=0.0
            self.rSpeed=0.0
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
            print("\nMoving forward..."+str(self.xSpeed))
        else:
            print("\nyou are not flying")
    
    #Function for moving backward
    #if the drone is flying 
    #set the speed to move backward 
    #and publish the geometry_msgs/Twist message to cmd_vel topic 
    def back(self):
        if self.flying:
            self.xSpeed=(-1*self.speed)
            self.ySpeed=0.0
            self.zSpeed=0.0
            self.rSpeed=0.0
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
            print("\nMoving backward..."+str(self.xSpeed))
        else:
            print("\nyou are not flying")
    
    #Function for moving right
    #if the drone is flying 
    #set the speed to move right 
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def right(self):
        if self.flying:
            self.xSpeed=0.0
            self.ySpeed=(-1*self.speed)
            self.zSpeed=0.0
            self.rSpeed=0.0
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
            print("\nMoving right...")
        else:
            print("\nyou are not flying")
    
    #Function for moving left
    #if the drone is flying 
    #set the speed to move left 
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def left(self):
        if self.flying:
            self.xSpeed=0.0
            self.ySpeed=self.speed
            self.zSpeed=0.0
            self.rSpeed=0.0
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
            print("\nMoving left...")
        else:
            print("\nyou are not flying")
    
    #Functions for moving up
    #if the drone is flying 
    #set the speed to move up
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def up(self):
        if self.flying:
            self.xSpeed=0.0
            self.ySpeed=0.0
            self.zSpeed=self.speed
            self.rSpeed=0.0
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
            print("\nMoving up...")
        else:
            print("\nyou are not flying")
    
    #Function for moving down
    #if the drone is flying 
    #set the speed to move down
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def down(self):
        if self.flying:
            self.xSpeed=0.0
            self.ySpeed=0.0
            self.zSpeed=(-1*self.speed)
            self.rSpeed=0.0
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
            print("\nMoving down...")
        else:
            print("\nyou are not flying")
    
    #Function for landing (self landing)
    #if the drone is flying 
    #set the speed to zero, and land 
    #publish the message of type std_msgs/Empty to land topic
    def land(self):
        if self.flying:
            self.xSpeed=0
            self.ySpeed=0.0
            self.zSpeed=0.0
            self.rSpeed=0.0
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
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
            self.xSpeed=0.0
            self.ySpeed=0.0
            self.zSpeed=0.0
            self.rSpeed=0.0
            print("\nTaking off...")
        else:
            print("\nYou are flying")
    
    #Function for rotating left
    #if the drone is flying 
    #set the speed to rotate left
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def rotateLeft(self):
        if self.flying:
            self.xSpeed=0.0
            self.ySpeed=0.0
            self.zSpeed=0.0
            self.rSpeed=self.speed
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
            print("\nRotating left...\n")
        else:
            print("\nyou are not flying\n")
    
    #Function for rotating right
    #if the drone is flying 
    #set the speed to rotate right
    #and publish the geometry_msgs/Twist message to cmd_vel topic
    def rotateRight(self):
        if self.flying:
            self.xSpeed=0.0
            self.ySpeed=0.0
            self.zSpeed=0.0
            self.rSpeed=(-1*self.speed)
            self.twist=Twist(Vector3(self.xSpeed,self.ySpeed,self.zSpeed), Vector3(0.0,0.0,self.rSpeed))
            self.pub_move.publish(self.twist)
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
        self.xSpeed = 0.0
        self.ySpeed = 0.0
        self.zSpeed = 0.0
        self.rSpeed= 0.0
        self.speed=0.02
         
        
    def on_press(self, key):
        self.myKey=str(key)  # gets the key pressed as string (variable mySelection is equal to current key being pressed)
        
            
        #Print the keys used for controlling the drone
        #Declaring decisions depending on the key pressed
        if (self.myKey=="u't'"): self.front()
        elif (self.myKey=="u'g'"): self.back()
        elif (self.myKey=="u'f'"): self.left()
        elif (self.myKey=="u'h'"): self.right()
        elif (self.myKey=="u's'"): self.land()
        elif (self.myKey=="u'a'"): self.takeOff()
        elif (self.myKey=="u'u'"): self.up()        
        elif (self.myKey=="u'j'"): self.down()
        elif (self.myKey=="u'o'"): self.rotateLeft()
        elif (self.myKey=="u'p'"): self.rotateRight()
        elif (self.myKey=="u'z'"): self.incrementSpeed()
        elif (self.myKey=="u'x'"): self.decrementSpeed()
        elif (self.myKey=="u'q'"): self.exitMode()
        elif (self.myKey == "u'n'"):
            self.tracking=True
            self.first=True   
        elif (self.myKey == "u'm'"):
            self.tracking=False;
            cv2.destroyAllWindows() 
        elif (self.myKey==Key.enter):
               print("Tracking Mode")
        else:
            print("Error!!! wrong entry") #If none of the above, print out wrong entry

    #when the key is released
    #if the key released is 'q' and is not flying, we can exit 
    #otherwise do nothing and continue
    def on_release(self, key):
        self.myKey=str(key)
        if (self.myKey == "u'q'" and not self.flying):
            self.exit=True
            return False
        
    #callback function from the CVbridge
    def camera_callback(self, data):
        try:
            #load bridge
            self.stream = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if (not self.tracking):
                printControl()
                #get the dimensions of the video stream
                height, width,channels=self.stream.shape
                self.centerWidth=width/2
                self.centerHeight=height/2
                self.correct_Xstart=int(self.centerWidth)-self.offset
                self.correct_Xend=int(self.centerWidth)+self.offset
                self.correct_Ystart=int(self.centerHeight)-self.offsetY
                self.correct_Yend=int(self.centerHeight)+self.offsetY
                #draw a rectangle for the correct position
                cv2.rectangle(self.stream, (self.correct_Xstart, self.correct_Ystart), (self.correct_Xend, self.correct_Yend),(0,255,0),2)
                cv2.imshow("Video", self.stream)
                cv2.waitKey(1) 
            elif(self.first):
                #for the first frame select the area to track
                self.bbox = cv2.selectROI(self.stream, False)
                self.first=False
               
                
                
                
                
                cv2.destroyAllWindows()
                #select the type of tacking
                tracker_types = ['BOOSTING','KCF', 'MOSSE']
                self.tracker_type = tracker_types[2]
                if self.tracker_type == 'BOOSTING':
                    self.track = cv2.TrackerBoosting_create()
                if self.tracker_type == 'KCF':
                    self.track = cv2.TrackerKCF_create()
                if self.tracker_type == 'MOSSE':
                    self.track = cv2.TrackerMOSSE_create()
             
                #Initialize tracker with first frame and bounding box
                ok = self.track.init(self.stream, self.bbox)  
                
            else:
                #update tracker
                ok, self.bbox = self.track.update(self.stream)
                #draw a rectangle for the correct position
                cv2.rectangle(self.stream, (self.correct_Xstart, self.correct_Ystart), (self.correct_Xend, self.correct_Yend),(0,255,0),2)
               #draw bounding box
                if ok:
                    # Tracking success
                    # Display tracker type on frame
                    cv2.putText(self.stream, self.tracker_type + " Tracker", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
                    p1 = (int(self.bbox[0]), int(self.bbox[1]))
                    p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
                    center_X=int(self.bbox[0]+(self.bbox[2]/2))
                    center_Y=int(self.bbox[1]+(self.bbox[3]/2))
                    #draw a rectangle of the roi being tracked
                    if (self.centered):
                        cv2.rectangle(self.stream, p1, p2, (0,255,0), 2, 1)
                        #draw a rectangle in the middle of the ROI
                        cv2.rectangle(self.stream, (center_X-2,center_Y-2), (center_X+2,center_Y+2), (0,255,0), 2, 1)
                    else:
                        cv2.rectangle(self.stream, p1, p2, (255,0,0), 2, 1)
                        #draw a rectangle in the middle of the ROI
                        cv2.rectangle(self.stream, (center_X-2,center_Y-2), (center_X+2,center_Y+2), (255,0,0), 2, 1)
                    
                    coordinateX, coordinateY= self.calculateCoordinates(center_X, center_Y)
                    #display info
                    cv2.putText(self.stream, ("x: "+str(coordinateX)+" y:"+str(coordinateX)), (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(50,170,50),2)

                    if (abs(coordinateX)>self.offset and abs(coordinateY)>self.offset):
                        print("\nFollowing target\n")
                        speedX, speedR=self.calculateSpeed(coordinateX, coordinateY)
                        self.twist=Twist(Vector3(speedX,0.0,0.0), Vector3(0.0,0.0,speedR))
                        cv2.putText(self.stream, ("Speed X: "+str("%.2f" % speedX)+" R:"+str("%.2f" % speedR)), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(50,170,50),2)
                        self.pub_move.publish(self.twist)
                        self.centered=False
                    elif(abs(coordinateX)>self.offset):
                        print("\nFollowing target\n")
                        _, speedR=self.calculateSpeed(coordinateX, coordinateY)
                        speedX=0
                        self.twist=Twist(Vector3(0.0,0.0,speedX), Vector3(0.0,0.0,speedR))
                        cv2.putText(self.stream, ("Speed X: "+str("%.2f" % speedX)+" R:"+str("%.2f" % speedR)), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(50,170,50),2)
                        self.pub_move.publish(self.twist)
                        self.centered=False
                    elif(abs(coordinateY)>self.offsetY):
                        print("\nFollowing target\n")
                        speedX,_=self.calculateSpeed(coordinateX, coordinateY)
                        speedR=0
                        cv2.putText(self.stream, ("Speed X: "+str("%.2f" % speedX)+" R:"+str("%.2f" % speedR)), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(50,170,50),2)
                        self.twist=Twist(Vector3(speedX,0.0,0.0), Vector3(0.0,0.0,speedR))
                        self.pub_move.publish(self.twist)
                        self.centered=False
                    else:
                        self.direction_pub.publish("Correct")
                        cv2.putText(self.stream, "Position Correct", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),2)
                        self.centered=True
                        print("Target reached")
                else :
                    # Tracking failure
                    cv2.putText(self.stream, "Tracking failure detected", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    self.direction_pub.publish("Tracking Failed")
                    
                
                
                cv2.imshow("Tracking", self.stream)
                k=cv2.waitKey(1) & 0xFF #ascii
            
        except CvBridgeError as e:
            print(e)







####### main ##########
if __name__=='__main__':
    try:
        #create the object tracker
        tracker=Tracker()
        while rospy.ok():
            if (self.exit): break
            rospy.spin()
    except rospy.ROSInterruptException:
        print("error")
        pass
