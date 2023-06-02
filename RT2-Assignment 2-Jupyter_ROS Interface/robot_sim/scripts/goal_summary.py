"""
.. module:: goal_summary

:platform: Unix
:synopsis: Python module for monitoring the goals status.

.. moduleauthor:: Saeed Abdollahi Taromsari saeed.abdollahi.t@gmail.com

Detailed Description: 
    This node subscribes to the 'reaching_goal/feedback' topic to check the current goal status, 
    then it creates a service to print the number of reached or cancelled goals.
    
Communication Protocole:
    It uses the client-service protocole for communication.

"""
#! /usr/bin/env python3

import sys
import rospy
from robot_sim.srv import GoalSummary
from robot_sim.msg import PlanningActionFeedback

goals_reached = 0
goals_cancelled = 0

def updateGoalSummary(msg):
    """
    Description:
        It counts the number of reached or cancelled goals.
        
    Args: 
        msg: A message of type 'PlanningActionFeedback'.

    Returns:
        None.

    """
    global goals_reached
    global goals_cancelled

    #Check the state value of the 'PlanningActionFeedback' message
    state = msg.feedback.stat
    if state == "Target reached!":#Update the goals_reached
        goals_reached = goals_reached + 1

    if state == "Target cancelled!":#Update the goals_cancelled
        goals_cancelled = goals_cancelled + 1  
       
def sendGoalSummary(res):
    """
    Description:
        Sends response to the 'goalSummary' request.
        
    Args: 
        msg: A message of type 'PlanningActionFeedback'

    Returns:
        None.

    """
    global goals_reached
    global goals_cancelled

    #Sends the number of reached or cancelled goals
    return [goals_reached,goals_cancelled]

if __name__ == "__main__":
    try:#Trys to create a ROS node
        rospy.init_node('goal_summary')

        #Creates a subscriber to receive the 'PlanningActionFeedback' message
        goalSummarySubscriber = rospy.Subscriber("reaching_goal/feedback", PlanningActionFeedback, updateGoalSummary)
        #Creates a service to print the number of reached or cancelled goals
        goalSummaryService = rospy.Service('goalSummary', GoalSummary, sendGoalSummary)

        if goalSummaryService:#Shows a message on the console that server is ready to response
            print("Server is ready :D")
        rospy.spin()

    except rospy.ROSInterruptException:#If the process failed, show an error message
        print("program interrupted before completion", file=sys.stderr)