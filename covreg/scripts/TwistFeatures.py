#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from broadcast.msg import StampedFeatures

from broadcast import Broadcasters as Trx

class TwistFeaturePublisher:
    '''Publishes features calculated from an twist topic.'''

    def __init__( self ):
        rospy.init_node( 'twist_feature_publisher' )
        self.feature_name = rospy.get_param( '~feature_name' )
        self.odom_sub = rospy.Subscriber( 'twist', Twist, self.OdomCallback )

        descriptions = [ 'x_vel', 'y_vel', 'z_vel',
                         'roll_vel', 'pitch_vel', 'yaw_vel' ]
        
        self.two_dimensional = rospy.get_param( '~two_dimensional' )
        if self.two_dimensional:
            feature_dim = 3
        else:
            feature_dim = 6

        self.feat_tx = Trx.Transmitter( broadcast_name=self.feature_name,
                                        feature_size=feature_dim,
                                        feature_descriptions=descriptions )

    def OdomCallback( self, tw ):
        out = StampedFeatures()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.feature_name

        if self.two_dimensional:
            out.features = ( tw.linear.x,
                             tw.linear.y,
                             tw.angular.z )
        else:
            out.features = ( tw.linear.x,
                             tw.linear.y,
                             tw.linear.z,
                             tw.angular.x,
                             tw.angular.y,
                             tw.angular.z )

        self.feat_tx.publish( out )

if __name__ == '__main__':
    try:
        of = TwistFeaturePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


