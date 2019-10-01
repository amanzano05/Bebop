#!/usr/bin/env python

#Importing libraries
from pynput.keyboard import Key, Listener #library used for key listene
import controller
import rospy #library for rospy client API 

from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

# Initializing global variable for controller
myCon=controller.Controller() # creates the object myCon from the class Controller
myCon.setSpeed(0.5)#Set speed


# ------functions------#



# when a key is pressed
#the correspondig method to the key pressed is called
def on_press(key):
    mySelection=str(key)  # gets the key pressed as string (variable mySelection is equal to current key being pressed)
    printControl() #Print the keys used for controlling the drone
    #Declaring decisions depending on the key pressed
    if (mySelection=="u't'"): myCon.front()
        
    elif (mySelection=="u'g'"): myCon.back()
    elif (mySelection=="u'f'"): myCon.left()
    elif (mySelection=="u'h'"): myCon.right()
    elif (mySelection=="u's'"): myCon.land()
    elif (mySelection=="u'a'"): myCon.takeOff()
    elif (mySelection=="u'u'"): myCon.up()        
    elif (mySelection=="u'j'"): myCon.down()
    elif (mySelection=="u'o'"): myCon.rotateLeft()
    elif (mySelection=="u'p'"): myCon.rotateRight()
    elif (mySelection=="u'z'"): myCon.incrementSpeed()
    elif (mySelection=="u'x'"): myCon.decrementSpeed()
    elif (mySelection=="u'q'"): myCon.exitMode()
        
    else:
        print("Error!!! wrong entry") #If none of the above, print out wrong entry

#Creating a function that will print the keys being pressed
def printKeys():
    
    print("Use the following keys to control the drone:\n" #Prints out an instruction list of commands to terminal
          "t:forward\n"
          "g:backward\n"
          "f:left\n"
          "h:right\n"
          "s:land\n"
          "a:take off\n"
          "u:up\n"
          "j:down\n"
          "o:rotate left\n"
          "p:rotate right\n"
          "q:exit mode\n")
          
#Creating a function that will print the control
def printControl():
    print("\n a:takeoff           t         u:up                \n"
          "    |                  |          |                  \n"
          "    |              f-------h      |       O<----->p  \n"
          "    |                  |          |                  \n"
          " s:land                g         j:down              \n")
          
          
#Creating a function for when a key is released. 
#if the key pressed is q and the status is not flying, the mode exists
def on_release(key):
    mySelection=str(key)

    if (mySelection == "u'q'") and not myCon.flying:
        return False


# gets the selection from the user.
def selectMode():
    print("\nSelect a mode to fly the drone:"
          "\n1)Manual"
          "\n2)Autonomous"
          "\n3)Demo"
          "\n4)Disconnect Drone")
    mode=raw_input("enter your selection:")
    return mode

#Function if we want to disconnect
#if not flying  then disconnect
def disconnect():
    if not myCon.flying: #can't disconnect while flying
        print("\nDisconnecting...\n")
        global  connected
        connected = False #Here is we offcially declare disconnected
    else:
        print("\nYou can not disconnect the drone while flying...\n")#Let users know you can't disconnect while flying

#function for taking matters into our own hands.
def manual():
    with Listener(
            on_press=on_press,#execute the commands via keys being pressed
            on_release=on_release) as listener:
        printKeys()
        listener.join()


######################################pending modes##########################################
#Function for autonomous mode
def autonomous():
    print("autonomus mode")

#Function for demo mode
def demo():
    print("demo mode")
#############################################################################################
    

# global dictionay for all available modes to select
switchModes={
    '1': manual,
    '2': autonomous,
    '3': demo,
    '4': disconnect
}


############################################main method######################################
if __name__== '__main__':
    try:
        #rospy.init_node('keyCommands2', anonymous=True)
        print("\nGetting things ready")
        print("\nConnecting...")
        global connected
        connected=True
	#while rospy is running  and the drone is connected do some work
        while not rospy.is_shutdown() and connected==True:
            mode = selectMode()#select the mode
            try:
                switchModes.get(mode)()#try to call the method corresponding to the mode
            except:
                print("\nWrong selection")
    except rospy.ROSInterruptException:
        print("error")
        pass
    
        





