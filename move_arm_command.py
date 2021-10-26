#! /usr/bin/env python

import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
from time import sleep
from math import pi


rospy.init_node('control_panda_bot')
control_publisher = rospy.Publisher('position_joint_trajectory_controller/command', JointTrajectory, queue_size=10)

i=-0.9

c=0.5

while not rospy.is_shutdown():

	msg = JointTrajectory()

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''
	msg.joint_names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7',]

	point = JointTrajectoryPoint()

	point.positions = [0.3221649,-0.0646347,-0.0661049,-2.02785,0.00455451,pi,0.69867]
	point.velocities = []
	point.accelerations = []
	point.effort = []
	point.time_from_start = rospy.Duration(5)

	msg.points.append( point )

	control_publisher.publish( msg )
	rospy.loginfo( msg ) 

	c=c+0.5

	i=i+0.05*c

        sleep(0.5) 

	 
            



