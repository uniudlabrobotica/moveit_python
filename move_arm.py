import rospy 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
from time import sleep, time
from math import pi
from copy import deepcopy
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt

def callback(data, arg):
	rate = rospy.Rate(1000)
	arg[0].append(data.header.stamp.secs)
	arg[1].append(data.header.seq)
	arg[2].append(data.position[0])
	arg[3].append(data.velocity[0])
	arg[4].append(data.effort[0])
	rate.sleep()
	#print(data.header.seq)
	#print(data.position[6])

def my_publisher():

	rospy.init_node('control_panda_bot')
	control_publisher = rospy.Publisher('/position_joint_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

	msg = FollowJointTrajectoryActionGoal()

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''

	msg.goal.trajectory.header.stamp = rospy.Time.now()
	msg.goal.trajectory.header.frame_id = ''
	msg.goal.trajectory.joint_names =    ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

	point = JointTrajectoryPoint()

	move_group = moveit_commander.MoveGroupCommander("panda_arm")
	start_position = move_group.get_current_joint_values()

	point.positions =  start_position
	point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	point.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	point.time_from_start = rospy.Duration(1)
	msg.goal.trajectory.points.append(deepcopy(point))

	point.positions =  [0.0, -0.585, 0.0, -2.156, 0.0, 1.571, 0.285]
	point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	point.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]	
	point.time_from_start = rospy.Duration(5)
	msg.goal.trajectory.points.append(deepcopy(point))

	sleep(0.2)

	control_publisher.publish(msg)
	
	time_step_secs_m = []
	time_step_seq_m = []
	q_m = []
	q_p_m = []
	tau_m = []

	rospy.Subscriber('/joint_states_desired', JointState, callback, (time_step_secs_m, time_step_seq_m, q_m, q_p_m, tau_m))
	
	rospy.sleep(5)

	
	n = len(time_step_seq_m)
	l = time_step_seq_m[n-1] - time_step_seq_m[0]
	print(l)

if __name__ == '__main__':

	my_publisher()
	
	
	





 

	 
            



