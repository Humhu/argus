#!/usr/bin/env python

import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from argus_msgs.msg import FloatVectorStamped

from broadcast import Broadcasters as Trx

class VOPolicyFeaturePublisher:
    '''Publishes features calculated from an odometry topic.'''

    def __init__( self ):
        rospy.init_node( 'vo_policy_feature_publisher' )
        self.stream_name = rospy.get_param( '~stream_name' )
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )

        descriptions = [ 'lin_speed', 'ang_speed', 'vel_x', 'vel_y', 'vel_z' ]
        self.feat_tx = Trx.Transmitter( stream_name=self.stream_name,
                                        namespace='~',
                                        feature_size=5,
                                        descriptions=descriptions,
                                        mode='push',
                                        queue_size=0 )

    def OdomCallback( self, odom ):
        # Assume 2D yaw only
        yaw = 2*math.acos( odom.pose.pose.orientation.w );

        lin_x = odom.twist.twist.linear.x
        lin_y = odom.twist.twist.linear.y
        lin_speed = math.sqrt( lin_x * lin_x + lin_y * lin_y )

        ang_z = odom.twist.twist.angular.z
        ang_speed = abs( ang_z )

        features = ( lin_speed, 
                     ang_speed,
                     lin_x, 
                     lin_y,
                     ang_z )
        self.feat_tx.publish( odom.header.stamp, features )

if __name__ == '__main__':
    try:
        of = VOPolicyFeaturePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
