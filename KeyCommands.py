#!/usr/bin/env python


from pynput.keyboard import Key, Listener #library used for key listene
import controller

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

# global variables

myCon=controller.Controller()
myCon.setSpeed(0.5)


# ------functions------#



# when a key is pressed
def on_press(key):
    mySelection=str(key)  # gets the key pressed as string
    printControl()
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
        print("Error!!! wrong entry")


def printKeys():
    print("Use the following keys to control the drone:\n"
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
          

def printControl():
    print("\n a:takeoff           t         u:up                \n"
          "    |                  |          |                  \n"
          "    |              f-------h      |       O<----->p  \n"
          "    |                  |          |                  \n"
          " s:land                g         j:down              \n")
          
          

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


def disconnect():
    if not myCon.flying:
        print("\nDisconnecting...\n")
        global  connected
        connected = False
    else:
        print("\nYou can not disconnect the drone while flying...\n")


def manual():
    with Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        printKeys()
        listener.join()

def autonomous():
    print("autonomus mode")


def demo():
    print("demo mode")
    

# global dictionay
switchModes={
    '1': manual,
    '2': autonomous,
    '3': demo,
    '4': disconnect
}




if __name__== '__main__':
    try:
        #rospy.init_node('keyCommands2', anonymous=True)
        print("\nGetting things ready")
        print("\nConnecting...")
        global connected
        connected=True
        while not rospy.is_shutdown() and connected==True:
            mode = selectMode()
            try:
                switchModes.get(mode)()
            except:
                print("\nWrong selection")
    except rospy.ROSInterruptException:
        print("error")
        pass
    
        





