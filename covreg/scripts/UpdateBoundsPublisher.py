#!/usr/bin/env python

from math import sqrt
import numpy as np
import rospy
from argus_utils import MatrixUtils
from argus_msgs.msg import FilterUpdate, FilterUpdateBounds

class UpdateBoundsPublisher:
    '''Publishes bounds on a filter update.'''

    def __init__( self ):
        rospy.init_node( 'update_bounds_publisher' )
        self.odom_sub = rospy.Subscriber( 'updates', FilterUpdate, self.UpdateCallback,
                                          queue_size=10 )
        self.bounds_pub = rospy.Publisher( 'bounds', FilterUpdateBounds, queue_size=10 )

    def UpdateCallback( self, update ):
        obs = np.array( update.observation )
        R = MatrixUtils.MsgToMatrix( update.observation_cov )
        variances = np.diag ( R )
        three_std_devs = np.sqrt( variances )
        
        out = FilterUpdateBounds()
        out.header = update.header
        out.bounds_centered_upper = obs + three_std_devs
        out.bounds_centered_lower = obs - three_std_devs
        out.bounds_zeroed_upper = three_std_devs
        out.bounds_zeroed_lower = -three_std_devs

        self.bounds_pub.publish( out )

if __name__ == '__main__':
    try:
        of = UpdateBoundsPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


