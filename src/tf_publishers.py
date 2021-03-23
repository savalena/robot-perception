#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import tf_conversions
import tf2_ros

class TF(object):
    def __init__(self):
        self.robot_name = "robot"
        self.lidar_x = 0
        self.lidar_y = 0
        self.lidar_a = 0

        self.tf2_broad = tf2_ros.TransformBroadcaster()

        rospy.Timer(rospy.Duration(1. / 20), self.tf_callback)

    def tf_callback(self, event):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.robot_name
        t.child_frame_id = self.robot_name + "_laser"
        t.transform.translation.x = self.lidar_x
        t.transform.translation.y = self.lidar_y
        t.transform.translation.z = .4
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.lidar_a)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf2_broad.sendTransform(t)

if __name__ == '__main__':
    try:
        rospy.init_node('tf_node', anonymous=True)
        pf_node = TF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

