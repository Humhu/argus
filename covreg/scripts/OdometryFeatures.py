#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from argus_msgs.msg import PredictionFeatures

class OdometryFeaturePublisher:

    def __init__( self ):
        rospy.init_node( 'odometry_feature_publisher' )
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )
        self.feat_pub = rospy.Publisher( 'features', PredictionFeatures,
                                         queue_size = 10 )
        self.source_name = rospy.get_param( '~source_name' )

    def OdomCallback( self, odom ):
        out = PredictionFeatures()
        out.header.stamp = odom.header.stamp
        out.header.frame_id = self.source_name

        out.features = ( odom.pose.pose.position.x, 
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z,
                         odom.pose.pose.orientation.w,
                         odom.twist.twist.linear.x,
                         odom.twist.twist.linear.y,
                         odom.twist.twist.linear.z,
                         odom.twist.twist.angular.x,
                         odom.twist.twist.angular.y,
                         odom.twist.twist.angular.z )
        self.feat_pub.publish( out )

if __name__ == '__main__':
    try:
        of = OdometryFeaturePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


