#!/user/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

def pan_tilt_callback(pan_tilt_data):
    return 0

if __name__ == '__main__':
    try:
        s = rospy.Subscriber('/pan_tilt_data', Float64MultiArray, pan_tilt_callback, queue_size=1)
        pan_tilt_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

    except rospy.ROSInterruptException:
        pass