#! /usr/bin/env python

import rospy 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import random
from time import sleep, time
from math import pi
from copy import deepcopy


rospy.init_node('control_panda_bot')
control_publisher = rospy.Publisher('/position_joint_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

i=pi

while i>pi/2:

	msg = FollowJointTrajectoryActionGoal()

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''

	msg.goal.trajectory.header.stamp = rospy.Time.now()
	msg.goal.trajectory.header.frame_id = ''
	msg.goal.trajectory.joint_names =    ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
	
	points = JointTrajectoryPoint

	points.positions = [0.3221649,-0.0646347,-0.0661049,-2.0278500,0.00455451,i,0.6986700]
	points.velocities = []
	points.accelerations = []
	points.effort = []
	points.time_from_start = rospy.Duration(15)
	
	msg.goal.trajectory.points.append(deepcopy(points))

	i=i-0.001	 


sleep(1)

control_publisher.publish(msg)
rospy.loginfo(msg)






 

	 
            



