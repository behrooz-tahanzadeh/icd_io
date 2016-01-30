#!/usr/bin/env python
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header
import rospy


publisherSetPoint = None




def publishSetPoint():
    global publisherSetPoint
    
    header = Header(0, rospy.rostime.get_rostime(), "world")
    pose = Pose()
    
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 1
    
    publisherSetPoint.publish(PoseStamped(header, pose))
    
    


def main(argv=None):
    global publisherSetPoint
    
    publisherSetPoint = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    rospy.init_node('itech_mavros_offboard', anonymous=True)
    
    rospy.loginfo("========= itech_mavros_offboard =========")
    
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        publishSetPoint()
        rate.sleep()
#eof

if __name__ == '__main__':
    main()