import rospy 
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
from time import sleep, time
from math import pi
from copy import deepcopy


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
point.time_from_start = rospy.Duration(0)
msg.goal.trajectory.points.append(deepcopy(point))

point.positions =  [0.0, -0.585, 0.0, -1.956, 0.0, 1.571, 0.785]
point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]	
point.time_from_start = rospy.Duration(6)
msg.goal.trajectory.points.append(deepcopy(point))

sleep(1)

control_publisher.publish(msg)
rospy.loginfo(msg)






 

	 
            



