#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3, PoseWithCovarianceStamped, Transform, TransformStamped


def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('LHR_FFF91D43/pose', PoseStamped, queue_size=10)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        tmp_ps=PoseStamped()
        tmp_ps.header.frame_id="map"
        tmp_ps.header.stamp=rospy.Time.now()
        tmp_ps.pose.position.x=0
        tmp_ps.pose.orientation.x=0.0
        tmp_ps.pose.orientation.y=0.0
        tmp_ps.pose.orientation.z=0.0
        tmp_ps.pose.orientation.w=1.0

        pub.publish(tmp_ps)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

