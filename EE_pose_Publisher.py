#!/usr/bin/env python

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Point

#Talking function being called in the main().
#Pulls and published EE Pose from move_group
def talker():  

    #Declaring where our information is being published to "EE_XYZ" - Change later
    pub = rospy.Publisher('EE_XYZ', Point, queue_size=10)
    #Naming our node to EE_Pose
    rospy.init_node("EE_Pose", anonymous=True)

    rate = rospy.Rate(1) # 1 HZ

    #While loop to keep going until rospy is shutdown
    while not rospy.is_shutdown():

        #Use move_group to pull current pose posistion
        EE_Pose = move_group.get_current_pose().pose
        #EE_Pose = [EE_Pose.position.x, EE_Pose.position.y, EE_Pose.position.z]

        #Publishing that current posistion to out desired topic
        pub.publish(EE_Pose.position.x, EE_Pose.position.y, EE_Pose.position.z)

        rate.sleep()


if __name__ == '__main__':

    #Starting out moveit_commander node
    moveit_commander.roscpp_initialize(sys.argv)
    #Starting our move_group object used to pull EE posistion
    move_group = moveit_commander.MoveGroupCommander('manipulator')
    
    try:
        #Calling the talker() function
        talker()
    except rospy.ROSInterruptException:
        pass
