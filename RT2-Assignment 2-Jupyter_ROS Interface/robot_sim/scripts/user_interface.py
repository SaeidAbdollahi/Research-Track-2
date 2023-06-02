"""
.. module:: user_interface

:platform: Unix
:synopsis: Python module to show an interface to the user

.. moduleauthor:: Saeed Abdollahi Taromsari saeed.abdollahi.t@gmail.com

Detailed Description: 
    This node creates a terminal-based interface to set the next positional goal for the robot.
    
Communication Protocole:
    It uses an action-client protocole to connect to the '/reaching_goal' server.

"""
#! /usr/bin/env python3

import sys
import os
import rospy
import actionlib
import actionlib.msg
from geometry_msgs.msg import PoseStamped
from robot_sim.msg import PlanningAction, PlanningGoal

#Global client variable to define an action-client
client = None

clearScreen = lambda: os.system('clear')
"""
Description: 
    It Clears the current console.

    Args:
        None.

    Returns:
        None.

"""

def showWelcome():
    """
    Description:
        It shows a welcome message to the user.
        
    Args: 
        None.

    Returns:
        None.
        
    """
    clearScreen()
    print("----------------------------------------------------------------------------")
    print("*                                                                          *")
    print("*                          Client Control Panel                            *")
    print("*                                                                          *")
    print("----------------------------------------------------------------------------")

def error():
    """
    Description:
        Shows an error message if the user selects an item out of the menu.
        
    Args:
        None.

    Returns:
        None.

    """
    print("Invalid input! Please enter the operation number from the menu...")

def showMenu():
    """
    Description:
        Shows an operation-select menu to the user.
        
    Args:
        None.

    Returns: 
        None.
 
    """
    global client

    #Print the menu's option
    print("\n----------------------------------------------------------------------------")
    print("*                            Operation Menu                                *")
    print("----------------------------------------------------------------------------")
    print("1-Enter a new goal position (x,y).")
    print("2-Cancel the current goal.")
    print("3-Exit.\n")

    #Read the user's choice
    userSelect = input("Please select the operation from the menu:")
    #Check the user's choice 
    if(userSelect == "1"): #Enter a new gaol
        print("Please enter the goal location x,y:")
        x = input("x:")
        y = input("y:")
        try:#Trys to receive a valid inputs
            #Converts string to float numbers
            x = float(x)
            y = float(y)

            #Shows the requested target position and send the request to the action server
            print("\nRequest for changing position to (x:%s,y:%s)"%(x, y))
            sendGoal(client, x, y) 
        except ValueError:#If the inputs not valid
            print("Invalid Input!")       
    elif(userSelect == "2"):#Cancel the current goal
        #Sends cancel request to the action server 
        cancelGoal(client)
    elif(userSelect == "3"):#End the user_interface node
        print("Exiting the user interface...Done")
        sys.exit(1)
    else:#If the user selects a number out of the menu, show an error message
        error()    

def createClient():
    """
    Description:
        Creates an action-client to connect to the '/reaching_goal' server.
        

    Message Type:
        PlanningAction
            geometry_msgs/PoseStamped target_pose

            ---

            ---

            geometry_msgs/Pose actual_pose
            string stat

    Args:
        None.

    Returns: 
        None.
 
    """
    try:#Trys to create an action-client
        #Creates an action-client
        client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    except rospy.ROSInterruptException:#If th process failed, show an error message
        print("Could not create the client", file=sys.stderr) 
    return client       

def sendGoal(client, x, y):
    """
    Description:
        Creats a goal of type "PoseStamped" as a target position for the robot.
        
    Args:
        client: The current action-client
        x: The x element of the target position
        y: The y element of the target position

    Returns: 
        None.
 
    """
    target_pose = PoseStamped() 
    
    try:#Trys to create a goal and snd it to the action-server
        # Waits until the action server has started up and started listening for goals
        client.wait_for_server()
        
       #Creates a goal to send to the action server.
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y        
        goal = PlanningGoal(target_pose)

        #Sends the goal to the action server
        client.send_goal(goal)
    except rospy.ServiceException as e: #If the process failed, show an error message
        print("Service call failed: %s"%e)

def cancelGoal(client):
    """
    Description:
        Cancel the current goal.
        
    Args:
        client: The current action-client

    Returns: 
        None.
 
    """
    client.cancel_goal()
 
if __name__ == "__main__":
    #Shows a welcome message to the user
    showWelcome()

    try:#Trys to create a ROS node
        #Initializes a rospy node so that the SimpleActionClient can
        rospy.init_node('client')

        #Creates the action client
        client = createClient()
    except rospy.ROSInterruptException:#If the process failed, show an error message
        print("program interrupted before completion", file=sys.stderr)

    #The infinite loop of the programe
    while not rospy.is_shutdown():
        #Shows the operation-select menu to the user
        showMenu()


            

        
