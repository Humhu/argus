#!/usr/bin/env python

import rospy
import psutil
from percepto_msgs.msg import RewardStamped

class SumReward:
    '''Publishes summed reward messages.'''

    def __init__( self ):

        self.driver_sub = rospy.Subscriber( 'reward_driver', RewardStamped, 
                                            self.CallbackDriver, queue_size = 10 )
        self.follower_sub = rospy.Subscriber( 'reward_follower', RewardStamped,
                                              self.CallbackFollower, queue_size = 10 )
        self.last_follower = None
        self.scale_driver = rospy.get_param( '~scale_driver', 1.0 )
        self.scale_follower = rospy.get_param( '~scale_follower', 1.0 )

        self.reward_pub = rospy.Publisher( '~reward', RewardStamped, queue_size=0 )

    def CallbackDriver( self, msg ):
        if self.last_follower is None:
            return

        out = RewardStamped()
        out.header.stamp = msg.header.stamp
        out.reward = self.scale_driver * msg.reward + self.scale_follower * self.last_follower
        self.reward_pub.publish( out )

    def CallbackFollower( self, msg ):
        self.last_follower = msg.reward

if __name__ == '__main__':
    rospy.init_node( 'sum_reward_evaluator' )
    try:
        sr = SumReward()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

