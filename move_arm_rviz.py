import rospy 
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
import math
from time import sleep
from copy import deepcopy

h = 2

if h == 1:
 
	goal_position = [2.0, 0.485, 1.2, -1.356, 1.0, 1.571, 0.085]

if h == 2:

	goal_position = [0.0, -0.485, 0.0, -1.856, 0.0, 1.571, 0.785]


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
#msg.goal.trajectory.joint_trajectory.points.append(deepcopy(points))

start_position = deepcopy(start_position)
error_position = deepcopy(error_position)

position_array = []

i = 0

j = 0.002
k = 0

for c in range(1001):
	
	points.positions[i] = start_position[i] + k
	position_array.insert(c,deepcopy(points.positions))
	points.time_from_start = rospy.Duration(j)
	msg.goal.trajectory.joint_trajectory.points.append(deepcopy(points))

	j += 0.001		
	k += error_position[i]/1000
		
	
for i in range(1,7):

	j = 0.002
	k = 0
	
	for c in range(1001):
		
		points.positions = position_array[c]
		points.positions[i] = start_position[i] + k
		points.time_from_start = rospy.Duration(j)
		msg.goal.trajectory.joint_trajectory.points[c] = deepcopy(points)
		position_array[c] = deepcopy(points.positions)

		j += 0.001		
		k += error_position[i]/1000
		
	
sleep(1)

control_publisher.publish(msg)
#rospy.loginfo(msg)
 			











 

	 
            



