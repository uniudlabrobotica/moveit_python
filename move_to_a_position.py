import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

"""
display_trajectory_publisher =
rospy.Publisher('/move_group/display_planned_path',
moveit_msgs.msg.DisplayTrajectory, queue_size=20)
"""

'''
# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print "============ Planning frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""
'''

joint_goal = move_group.get_current_joint_values()
print(joint_goal)

"""
start
[-0.019, 0.308, -0.047, -2.371, 0.070, 2.745, 0.676]
"""

"""
2
[0, -0.140, -0.042, -1.521, 0.004, 1.437, 0.775]
"""

"""
3
[0.246, -0.126, 1.056, -1.520, 0.134, 1.544, 0.766]
"""

"""
4 plus
[1.3773112797360672, 0.43244718400637305, 0.13107205059862972, -1.9470775184464035, -0.041725576174634516, 2.3793920897973355, 0.6845375790967774]
"""

"""
4
[1.37, 0.308, -0.047, -2.371, 0.070, 2.745, 0.676]
"""

while (raw_input() == "y"):
	joint_goal=[-0.019, 0.308, -0.047, -2.371, 0.070, 2.745, 0.676]
	print(joint_goal)
	move_group.go(joint_goal, wait=True)
	move_group.stop()

	joint_goal=[0, -0.140, -0.042, -1.521, 0.004, 1.437, 0.775]
	print(joint_goal)
	move_group.go(joint_goal, wait=True)
	move_group.stop()

	joint_goal=[0.246, -0.126, 1.056, -1.520, 0.134, 1.544, 0.766]
	print(joint_goal)
	move_group.go(joint_goal, wait=True)
	move_group.stop()

	joint_goal=[1.37, 0.308, -0.047, -2.371, 0.070, 2.745, 0.676]
	print(joint_goal)
	move_group.go(joint_goal, wait=True)
	move_group.stop()

	joint_goal=[-0.019, 0.308, -0.047, -2.371, 0.070, 2.745, 0.676]
	print(joint_goal)
	move_group.go(joint_goal, wait=True)
	move_group.stop()

"""
joint_goal[3]=-2
print(joint_goal)
move_group.go(joint_goal, wait=True)
move_group.stop()

joint_goal[3]=-1
print(joint_goal)
move_group.go(joint_goal, wait=True)
move_group.stop()

joint_goal[0]=-0.5
print(joint_goal)
move_group.go(joint_goal, wait=True)
move_group.stop()
"""

