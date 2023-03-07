import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

#現在位置位置に対する物体の位置がわかる。
#進む方向がわかる。
#距離が離れているほど速度を上げる。
#点と点を結んで移動していくイメージ
#だんだん点と点の距離が身近くなっていく。
#どうやら/joint_statesからTFができるらしい。









def pan_tilt_callback(pan_tilt_data):
    return 0

if __name__ == '__main__':
    try:
        s = rospy.Subscriber('/pan_tilt_data', Float64MultiArray, pan_tilt_callback, queue_size=10)
        pan_tilt_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    except rospy.ROSInterruptException:
        pass