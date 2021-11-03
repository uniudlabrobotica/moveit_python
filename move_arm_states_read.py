import rospy
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt


def callback(data, arg):
	rate = rospy.Rate(10)
	arg[0].append(data.header.stamp.secs)
	arg[1].append(data.header.stamp.nsecs)
	arg[2].append(data.position[0])
	arg[3].append(data.velocity[0])
	arg[4].append(data.effort[0])
	#arg[5].append(data.header.seq)
	rate.sleep()
	print(data.header.seq)

def my_listener():
	
	
	time_step_secs_m = []
	time_step_nsecs_m = []
	q_m = []
	q_p_m = []
	tau_m = []

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/joint_states_desired', JointState, callback, (time_step_secs_m, time_step_nsecs_m, q_m, q_p_m, tau_m))

	
	
	rospy.spin()

	# np.save('time_step_secs', np.asarray(time_step_secs_m))
	# np.save('time_step_nsecs', np.asarray(time_step_nsecs_m))
	# np.save('q', np.asarray(q_m))
	# np.save('q_p', np.asarray(q_p_m))
	# np.save('tau', np.asarray(q_p_m))

	#plt.plot(time_step_secs_m)
	#plt.show()
	#plt.plot(time_step_nsecs_m)
	#plt.show()
	#plt.plot(q_m)
	#plt.show()
	#plt.plot(q_p_m)
	#plt.show()
	#plt.plot(tau_m)
	#plt.show()


if __name__ == '__main__':

	my_listener()

	
