#!/usr/bin/env python

import rospy
import psutil
from percepto_msgs.msg import RewardStamped

class CPUReward:
    '''Publishes reward messages based on CPU utilization.'''

    def __init__( self ):

        self.reward_pub = rospy.Publisher( '~reward', RewardStamped, queue_size=0 )
        self.scale = rospy.get_param( '~scale', 1.0 )
        publish_rate = rospy.get_param( '~update_rate' )
        self.timer = rospy.Timer( rospy.Duration( 1.0/publish_rate ), self.TimerCallback )

    def TimerCallback( self, event ):
        msg = RewardStamped()

        msg.header.stamp = event.current_real
        msg.reward = -self.scale * psutil.cpu_percent()
        self.reward_pub.publish( msg );

if __name__ == '__main__':
    rospy.init_node( 'cpu_reward_evaluator' )
    try:
        cpu = CPUReward()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

