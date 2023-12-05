import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply


position = Point(x=0.5, y=0, z=0.5)
for angle in range(0,90,3):
    q_y = quaternion_about_axis(math.radians(angle), (0,1,0))
    orientation = Quaternion(*q_y)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.1)

for angle in range(0,90,3):
    q_z = quaternion_about_axis(math.radians(angle), (0,0,1))
    q_yz = quaternion_multiply(q_y, q_z)
    orientation = Quaternion(*q_yz)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.1)

for angle in range(0,90,3):
    q_z = quaternion_about_axis(math.radians(angle), (0,0,1))
    q_zy = quaternion_multiply(q_z, q_y)
    orientation = Quaternion(*q_zy)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.1)
