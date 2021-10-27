# moveit_python

SHELL1:
roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true

SHELL2:
roslaunch panda_moveit_config panda_moveit.launch controller:=position

SHELL3 (opzionale):
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config moveit_rviz.launch

python move_to_a_position.py

source ws_moveit/devel/setup.bash
source catkin_ws/devel/setup.bash
