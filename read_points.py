import rospy
from copy import deepcopy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep
import moveit_commander

toll = 0.05
target = 2
giun = 0

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

def my_publisher(trajectory):

    rospy.init_node('control_python_node')
    control_publisher = rospy.Publisher('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, queue_size=10)

    msg = ExecuteTrajectoryActionGoal()

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ''

    msg.goal.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
    msg.goal.trajectory.joint_trajectory.header.frame_id = ''
    msg.goal.trajectory.joint_trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    print('\n**************************************************\n')
    print('Giunti nella configurazione iniziale:')
    joint_start = move_group.get_current_joint_values()
    print(joint_start)

    point = JointTrajectoryPoint()
    point.positions = joint_start
    point.velocities = []
    point.accelerations = []
    point.effort = []
    point.time_from_start = rospy.Duration(0)
    msg.goal.trajectory.joint_trajectory.points.append(deepcopy(point))

    x = joint_start[giun]
    i = 1
    if joint_start[giun] < (target - toll):
        a = 1
    elif joint_start[giun] > (target + toll):
        a = -1
    while x > (target + toll) or x < (target - toll):
            x = x + a/1000.0
            i = i + 1
            point.positions[giun] = x
            point.time_from_start = rospy.Duration(i * 1.0/1000)
            msg.goal.trajectory.joint_trajectory.points.append(deepcopy(point))

    sleep(1)

    control_publisher.publish(msg)
    #rospy.loginfo(msg)

    # Da sostituire con un comando che attende la sincronizzazione
    raw_input('\nAl termine del movimento premere un tasto per continuare')

    joint_end = move_group.get_current_joint_values()
    if joint_end[0] > (target - toll) and joint_end[0] < (target + toll):
        print('\nTarget raggiunto!')
    else:
        print('\nTarget non raggiunto!')
    
    print('\nGiunti nella configurazione finale:')
    print(joint_end)

    print('\n**************************************************\n')

if __name__ == '__main__':

    traj = read_trajectory('traiettoria')

    my_publisher(traj)
