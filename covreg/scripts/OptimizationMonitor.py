#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from argus_utils import MatrixUtils

class OptimizationMonitor:
    '''Watches for unstable optimization.'''

    def __init__( self ):
        rospy.init_node( 'optimization_monitor' )
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )
        self.max_pos_cov = rospy.get_param( '~max_pose_cov' )
        self.max_vel_cov = rospy.get_param( '~max_vel_cov' )

    def OdomCallback( self, odom ):
        
        pos_cov = np.reshape( odom.pose.covariance, [6,6] )
        if np.any( np.abs( pos_cov ) > self.max_pos_cov ):
            rospy.logerror( 'Pose covariance greater than tolerance!' )
            exit()

        twist_cov = np.reshape( odom.twist.covariance, [6,6] )
        if np.any( np.abs( twist_cov ) > self.max_vel_cov ):
            rospy.logerror( 'Velocity covariance greater than tolerance!' )
            exit()

        
if __name__ == '__main__':
    try:
        of = OptimizationMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


