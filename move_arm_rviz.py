#! /usr/bin/env python

import rospy 
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import random
from time import sleep, time
from math import pi
from copy import deepcopy


rospy.init_node('control_panda_bot')
control_publisher = rospy.Publisher('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, queue_size=10)

i=0.01

sleep(1)

while i<pi:

	msg = ExecuteTrajectoryActionGoal()

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''

	msg.goal.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
	msg.goal.trajectory.joint_trajectory.header.frame_id = ''
	msg.goal.trajectory.joint_trajectory.joint_names =    ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
	
	points = JointTrajectoryPoint

	points.positions = [i, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035]
	points.velocities = []
	points.accelerations = []
	points.effort = []
	points.time_from_start = rospy.Duration(20)
	
	msg.goal.trajectory.joint_trajectory.points.append(deepcopy(points))

	i=i+0.01

	sleep(0.001)
	
	control_publisher.publish(msg)
	#rospy.loginfo(msg)






 

	 
            



