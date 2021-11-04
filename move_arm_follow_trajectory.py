import rospy 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
from time import sleep, time
from math import pi
from copy import deepcopy
import numpy as np
import sys
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction, GraspActionGoal

def read_trajectory(folder_name):

	traj = []

	file = open('./' + folder_name + '/q.txt','r')
	q = []

	for line in file:
		v_s = line.split('\t')
		a = []
		for s in v_s:
	    		a.append(float(s))
		q.append(a)

	file.close()
	traj.append(q)

	file = open('./' + folder_name + '/q_p.txt','r')
	q_p = []

	for line in file:
		v_s = line.split('\t')
		a = []
		for s in v_s:
	    		a.append(float(s))
		q_p.append(a)
	
	file.close()
	traj.append(q_p)

	file = open('./' + folder_name + '/q_pp.txt','r')
	q_pp = []

	for line in file:
		v_s = line.split('\t')
		a = []
		for s in v_s:
	    		a.append(float(s))
		q_pp.append(a)

	file.close()
	traj.append(q_pp)

	return traj

def move_to_start():

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface', anonymous=True)

	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	group_name = "panda_arm"
	group = moveit_commander.MoveGroupCommander(group_name)

	joint_goal = group.get_current_joint_values()

	joint_goal = [0.0, -0.485, 0.0, -1.956, 0.0, 1.571, 0.785]
	

	group.go(joint_goal, wait=True)
	group.stop()


def follow_trajectory(traj):

	t = 0.01
	n = 400

	#rospy.init_node('control_panda_bot')
	control_publisher = rospy.Publisher('/position_joint_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

	msg = FollowJointTrajectoryActionGoal()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''
	msg.goal.trajectory.header.stamp = rospy.Time.now()
	msg.goal.trajectory.header.frame_id = ''
	msg.goal.trajectory.joint_names =    ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

	points = JointTrajectoryPoint()
	points.positions = [0.0, -0.485, 0.0, -1.956, 0.0, 1.571, 0.785]
	points.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	points.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	points.effort = []
	points.time_from_start = rospy.Duration(1)

	position_array = []
	velocity_array = []
	acceleration_array = []

	i = 5

	j = t+1
	
	q_r = [row[5] for row in traj[0]]
	q_d_r = [row[5] for row in traj[1]]
	q_dd_r = [row[5] for row in traj[2]]


	#print(q_r)
	#print(q_d_r)
	#print(q_dd_r)
	u = []

	for c in range(n-1):
		
		points.positions[i] = q_r[10*c]
		points.velocities[i] = q_d_r[10*c]
		points.accelerations[i] = q_dd_r[10*c]
		position_array.insert(c,deepcopy(points.positions))
		velocity_array.insert(c,deepcopy(points.velocities))
		acceleration_array.insert(c,deepcopy(points.accelerations))
		points.time_from_start = rospy.Duration(j)
		msg.goal.trajectory.points.append(deepcopy(points))
		u.append(deepcopy(points.positions))

		j += t	

	sleep(0.5)

	#print(u)
	#print(len(u))
		
	control_publisher.publish(msg)	
		
"""			
		
	for i in range(1,7):

		j = 2*t
		k = 0
		
		for c in range(n-1):
			
			points.positions = position_array[c]
			points.velocities = velocity_array[c]
			points.accelerations = acceleration_array[c]
			points.positions[i] = traj[0]
			points.velocities[i] =traj[1]
			points.accelerations[i] = traj[2]
			points.time_from_start = rospy.Duration(j)
			msg.goal.trajectory.points[c] = deepcopy(points)
			position_array[c] = deepcopy(points.positions)
			velocity_array[c] = deepcopy(points.velocities)
			acceleration_array[c] = deepcopy(points.accelerations)
			
			j += t		
			
"""
	


if __name__ == '__main__':

	
	trajectory = read_trajectory('traiettoria')
	move_to_start()
	follow_trajectory(trajectory)


 

	 
            



