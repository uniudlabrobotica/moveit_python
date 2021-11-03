import rospy 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
from time import sleep, time
from math import pi
from copy import deepcopy

goal_position = [0.0, -0.485, 0.0, -1.856, 0.0, 1.571, 1.185]

move_group = moveit_commander.MoveGroupCommander("panda_arm")
start_position = move_group.get_current_joint_values()

error_position = []

for i in range(0, 7):
 
	error = goal_position[i] - start_position[i]
	error_position.insert(i, error)

rospy.init_node('control_panda_bot')
control_publisher = rospy.Publisher('/position_joint_trajectory_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1000)

msg = FollowJointTrajectoryActionGoal()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = ''
msg.goal.trajectory.header.stamp = rospy.Time.now()
msg.goal.trajectory.header.frame_id = ''
msg.goal.trajectory.joint_names =    ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

n = 100
t = 0.1

points = JointTrajectoryPoint()
points.positions = start_position
points.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
points.accelerations = []
points.effort = []
points.time_from_start = rospy.Duration(t)

start_position = deepcopy(start_position)
error_position = deepcopy(error_position)

position_array = []

i = 0

j = 2*t
k = 0

for c in range(n+1):
	
	points.positions[i] = start_position[i] + k
	position_array.insert(c,deepcopy(points.positions))
	points.time_from_start = rospy.Duration(j)
	msg.goal.trajectory.points.append(deepcopy(points))

	j += t		
	k += error_position[i]/n
		
	
for i in range(1,7):

	j = 2*t
	k = 0
	
	for c in range(n+1):
		
		points.positions = position_array[c]
		points.positions[i] = start_position[i] + k
		points.time_from_start = rospy.Duration(j)
		msg.goal.trajectory.points[c] = deepcopy(points)
		position_array[c] = deepcopy(points.positions)

		j += t		
		k += error_position[i]/n
		

sleep(0.5)
	
control_publisher.publish(msg)
#rospy.loginfo(msg)


 

	 
            



