#!/usr/bin/env python

import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from broadcast.msg import StampedFeatures

from broadcast import Broadcasters as Trx

class OdometryFeaturePublisher:
    '''Publishes features calculated from an odometry topic.'''

    def __init__( self ):
        rospy.init_node( 'odometry_feature_publisher' )
        self.feature_name = rospy.get_param( '~feature_name' )
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )

        descriptions = [ #'x', 'y', 'yaw',
                         'x_vel', 'y_vel', 'yaw_vel' ]
        self.feat_tx = Trx.Transmitter( broadcast_name=self.feature_name,
                                        feature_size=3,
                                        feature_descriptions=descriptions )

    def OdomCallback( self, odom ):
        out = StampedFeatures()
        out.header.stamp = odom.header.stamp
        out.header.frame_id = self.feature_name

        # Assume 2D yaw only
        yaw = 2*math.acos( odom.pose.pose.orientation.w );

        out.features = ( odom.twist.twist.linear.x,
                         odom.twist.twist.linear.y,
                         odom.twist.twist.angular.z )
        self.feat_tx.publish( out )

if __name__ == '__main__':
    try:
        of = OdometryFeaturePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


