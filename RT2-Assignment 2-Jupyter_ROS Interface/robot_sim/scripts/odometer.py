"""
.. module:: odometer

:platform: Unix
:synopsis: Python module for manipulating the odometry data.

.. moduleauthor:: Saeed Abdollahi Taromsari saeed.abdollahi.t@gmail.com

Detailed Description: 
    This node subscribes to the '/odom' topic to receive the Odometry message, 
    then it uses the publish-subscribe protocole to transfer the odometry messages.
    
Communication Protocole:
    It uses the publish-subscribe protocole to transfer the odometry messages.
    
"""
#! /usr/bin/env python3


import sys
import rospy
from nav_msgs.msg import Odometry
from robot_sim.msg import OdoSensor

"""
OdoSensor custom message

    Description:
        A custom message to send the odometry data

    Message Type:
        OdoSensor: 
            float32 x
            float32 y
            float32 vel_x
            float32 vel_y
"""
odoMsg = OdoSensor()

def setOdoMessage(msg):
    """
    Description:
        It is the 'odoMessageSubscriber' handler.
    
    Message Type:
        Odometry
            pose:
                pose:
                    position:
                        x

                        y
            twist:
                twist:
                     linear:
                        x

                        y

    Args: 
        msg: The odometry data published on the '/odom' topic. 

    Returns:
        None.

    """
    global odoMsg

    #Fills the custom odoMsg object by the odemetry data
    odoMsg.x = msg.pose.pose.position.x
    odoMsg.y = msg.pose.pose.position.y
    odoMsg.vel_x = msg.twist.twist.linear.x
    odoMsg.vel_y = msg.twist.twist.linear.y

if __name__ == "__main__":

    try:#Trys to create a ROS node
        rospy.init_node('odemeter')

        #Creates a subscriber to receive the 'Odometry' message
        odoMessageSubscriber = rospy.Subscriber("odom", Odometry, setOdoMessage)
        #Creates a publisher to publish the odometry data by a 'OdoSensor' custom message
        odoMessagePublisher = rospy.Publisher('odometer', OdoSensor, queue_size=10)

        #Sets the clock rate fo the current node
        rate = rospy.Rate(2) # 10hz

        #The infinite loop of the programe
        while not rospy.is_shutdown():
            if odoMsg:#If odoMsg is available, publish the message 
                odoMessagePublisher.publish(odoMsg)
                #Sleep for a predefined time
                rate.sleep()

    except rospy.ROSInterruptException:#If the process failed, show an error message
        print("program interrupted before completion", file=sys.stderr)


            

        
