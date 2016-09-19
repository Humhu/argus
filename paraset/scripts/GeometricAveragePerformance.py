#!/usr/bin/env python

import rospy
import numpy as np
from math import sqrt
from percepto_msgs.msg import RewardStamped

class RunningAveragePerformance:
    '''Outputs a geometric average reward.'''

    def __init__( self ):
        rospy.init_node( 'running_average_performance' )

        self.gamma = rospy.get_param( '~gamma' )
        if self.gamma < 0 or self.gamma > 1.0:
            raise ValueError( 'gamma must be between 0 and 1.' )

        self.acc = rospy.get_param( '~initial_average', None )

        self.reward_pub = rospy.Publisher( '~average', RewardStamped, queue_size=0 )
        self.odom_sub = rospy.Subscriber( 'reward', RewardStamped, self.RewardCallback )

    def RewardCallback( self, msg ):
        if self.acc is None:
            self.acc = msg.reward
        else:
            self.acc = self.acc * self.gamma + (1.0 - self.gamma) * msg.reward

        out = RewardStamped()
        out.header.stamp = msg.header.stamp
        out.reward = self.acc

        self.reward_pub.publish( out )

if __name__ == '__main__':
    try:
        rms = RunningAveragePerformance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



