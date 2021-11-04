import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import actionlib
from franka_gripper.msg import MoveGoal, MoveAction, GraspActionGoal
from time import sleep


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = group.get_current_joint_values()

pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)
g = GraspActionGoal()
g.goal.width=0.08
g.goal.speed=0.2
pub.publish(g)

#start
joint_goal[0] = -pi/2
joint_goal[1] = pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = 3*pi/4
joint_goal[6] = pi*3/4

group.go(joint_goal, wait=True)
group.stop()

g = GraspActionGoal()
g.goal.width=0.01
g.goal.speed=0.8
pub.publish(g)
sleep(2)

#1
joint_goal[0] = -pi/2
joint_goal[1] = pi/8
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = (3*pi/4)-pi/8
joint_goal[6] = pi/4

group.go(joint_goal, wait=True)
group.stop()

#2
joint_goal[0] = pi/4
joint_goal[1] = pi/8
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = (3*pi/4)-pi/8
joint_goal[6] = pi/4

group.go(joint_goal, wait=True)
group.stop()

#end
joint_goal[0] = pi/4
joint_goal[1] = pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = 3*pi/4
joint_goal[6] = pi/4

group.go(joint_goal, wait=True)
group.stop()


g = GraspActionGoal()
g.goal.width=0.08
g.goal.speed=0.8
pub.publish(g)

joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = -pi*3/4
joint_goal[4] = 0
joint_goal[5] = 3*pi/4
joint_goal[6] = pi/4

group.go(joint_goal, wait=True)
group.stop()

"""
#movimento gripper con moveIt

group_name = "hand"
group = moveit_commander.MoveGroupCommander(group_name)

hand_goal = group.get_current_joint_values()
print(hand_goal)
hand_goal[0] = 0.01
hand_goal[1] = 0.01
group.go(hand_goal, wait=True)
group.stop()
"""












