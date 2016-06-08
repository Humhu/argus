#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from argus_msgs.msg import PredictionFeatures
import math

class OdometryFeaturePublisher:
    '''Publishes features calculated from an odometry topic.'''

    def __init__( self ):
        rospy.init_node( 'odometry_feature_publisher' )
        self.source_name = rospy.get_param( '~source_name' )
        self.feat_pub = rospy.Publisher( 'features', PredictionFeatures,
                                         queue_size = 10 )
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )

    def OdomCallback( self, odom ):
        out = PredictionFeatures()
        out.header.stamp = odom.header.stamp
        out.header.frame_id = self.source_name

        # Assume 2D yaw only
        yaw = 2*math.acos( odom.pose.pose.orientation.w );

        out.features = ( odom.pose.pose.position.x, 
                         odom.pose.pose.position.y,
                         yaw,
                         odom.twist.twist.linear.x,
                         odom.twist.twist.linear.y,
                         odom.twist.twist.angular.z )
        self.feat_pub.publish( out )

if __name__ == '__main__':
    try:
        of = OdometryFeaturePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


