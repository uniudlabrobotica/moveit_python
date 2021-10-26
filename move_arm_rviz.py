import rospy 
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
import math
from time import sleep
from copy import deepcopy

goal_position = [0.0, 0.485, 0.2, -1.356, 1.0, 1.571, 0.085]

move_group = moveit_commander.MoveGroupCommander("panda_arm")
start_position = move_group.get_current_joint_values()

error_position = []

for i in range(0, 7):
 
	error = goal_position[i] - start_position[i]
	error_position.insert(i, error)

rospy.init_node('control_panda_bot')
control_publisher = rospy.Publisher('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, queue_size=10)

msg = ExecuteTrajectoryActionGoal()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = ''
msg.goal.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
msg.goal.trajectory.joint_trajectory.header.frame_id = ''
msg.goal.trajectory.joint_trajectory.joint_names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

points = JointTrajectoryPoint()
points.positions = start_position
points.velocities = []
points.accelerations = []
points.effort = []
points.time_from_start = rospy.Duration(1)
msg.goal.trajectory.joint_trajectory.points.append(deepcopy(points))

start_position = deepcopy(start_position)
error_position = deepcopy(error_position)

print(start_position)
print(goal_position)
print(error_position)

sleep(1)

#for i in range(7):
i=0

k = 0.0
	
if (error_position[i] >= 0):

	while points.positions[i] <= goal_position[i]:

		points.positions.pop(i)

		points.positions.insert(i, start_position[i] + k)	
		msg.goal.trajectory.joint_trajectory.points.append(deepcopy(points))
		
		k += error_position[i]/1000

		
if (error_position[i] < 0):
	
	while points.positions[i] > goal_position[i]:

		points.positions.pop(i)

		points.positions.insert(i, start_position[i] + k)	
		msg.goal.trajectory.joint_trajectory.points.append(deepcopy(points))
		
		k += error_position[i]/1000


control_publisher.publish(msg)
rospy.loginfo(msg)

			
print('_______________________________________________________')	
print(points.positions[i])
print(move_group.get_current_joint_values())
print(start_position)
print(goal_position)
print(error_position)










 

	 
            



