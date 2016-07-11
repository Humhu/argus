#!/usr/bin/env python

from math import sqrt
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from argus_msgs.msg import OdometryBounds

class OdometryBoundsPublisher:
    '''Publishes bounds on the odometry twist'''

    def __init__( self ):
        rospy.init_node( 'odometry_bounds_publisher' )
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )
        self.bounds_pub = rospy.Publisher( 'bounds', OdometryBounds, queue_size=10 )

        self.inds = np.diag( np.reshape( range(36), [6,6] ) )

    def OdomCallback( self, odom ):
        out = OdometryBounds()
        out.header.stamp = odom.header.stamp
        
        means = [ odom.twist.twist.linear.x,
                  odom.twist.twist.linear.y,
                  odom.twist.twist.linear.z,
                  odom.twist.twist.angular.x,
                  odom.twist.twist.angular.y,
                  odom.twist.twist.angular.z ]
        std_devs = [ sqrt( odom.twist.covariance[i]) for i in self.inds ]
        out.twist_upper = [ u + s for (u,s) in zip(means, std_devs) ]
        out.twist_lower = [ u - s for (u,s) in zip(means, std_devs) ]
        self.bounds_pub.publish( out )

if __name__ == '__main__':
    try:
        of = OdometryBoundsPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


