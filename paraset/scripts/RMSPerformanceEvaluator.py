#!/usr/bin/env python

import rospy
import numpy as np
from math import sqrt
from nav_msgs.msg import Odometry
from percepto_msgs.msg import RewardStamped

class RMSPerformanceEvaluator:
    '''Publishes reward messages from odometry RMS error.'''

    def __init__( self ):
        rospy.init_node( 'rms_performance_evaluator' )

        self.pose_weights = np.array( rospy.get_param('~pose_weights', [0,0,0,0,0,0]) )
        self.pose_sum = self.pose_weights.sum()
        if self.pose_sum == 0.0:
            self.pose_sum = 1.0

        self.vel_weights = np.array( rospy.get_param('~vel_weights', [0,0,0,0,0,0]) )
        self.vel_sum = self.vel_weights.sum()
        if self.vel_sum == 0.0:
            self.vel_sum = 1.0

        self.reward_pub = rospy.Publisher( '~reward', RewardStamped, queue_size=0 )
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )

    def OdomCallback( self, odom ):
        out = RewardStamped()
        out.header.stamp = odom.header.stamp

        pose_cov = np.reshape( odom.pose.covariance, [6,6] )
        pose_vars = pose_cov.diagonal()
        pose_mse = ( self.pose_weights * pose_vars ).sum()

        vel_cov = np.reshape( odom.twist.covariance, [6,6] )
        vel_vars = vel_cov.diagonal()
        vel_mse = ( self.vel_weights * vel_vars ).sum()

        out.reward = -( pose_mse + vel_mse )
        self.reward_pub.publish( out )

if __name__ == '__main__':
    try:
        rms = RMSPerformanceEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



