import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
rospy.init_node("trajectory_gen")
t_pub = rospy.Publisher("/joint_target", JointTrajectoryPoint, queue_size=1)
msg = JointTrajectoryPoint()
msg.positions = [0, 0, 0]
msg.velocities = [0, 0, 0]
msg.accelerations = [0, 0, 0]
timer = rospy.Timer(rospy.Duration(0.1), lambda x: t_pub.publish(msg))
print("Modify 'msg' variable to control arm...")
