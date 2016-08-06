#!/usr/bin/env python

import rospy
from broadcast.msg import StampedFeatures
from broadcast import Broadcasters

class ConstantFeatures:
    '''Publishes a constant feature at a fixed rate or caches
       a constant feature for all time.'''

    def __init__( self ):
        rospy.init_node( 'constant_feature_publisher' )
        
        stream_name = rospy.get_param( '~stream_name' )
        descriptions = rospy.get_param( '~descriptions' )
        self.features = rospy.get_param( '~features' )
        self.mode = rospy.get_param( '~mode' )

        opt_args = {}
        if self.mode == 'push':
            opt_args['queue_size'] = rospy.get_param( '~queue_size', 0 )
            self.publish_rate = rospy.Rate( rospy.get_param( '~publish_rate' ) )
        elif self.mode == 'pull':
            opt_args['cache_time'] = float('inf')
        else:
            raise ValueError( 'Mode must be pull or push' )

        rate = rospy.get_param( '~publish_rate', 1.0 )
        self.publish_rate = rospy.Rate( rate );

        self.feat_tx = Broadcasters.Transmitter( stream_name = stream_name,
                                                 namespace = '~',
                                                 feature_size = len( self.features ),
                                                 descriptions = descriptions,
                                                 mode = self.mode,
                                                 **opt_args )

    def Execute( self ):

        if self.mode == 'push':
            
            while not rospy.is_shutdown():
                self.feat_tx.publish( rospy.Time.now(), self.features )
                self.publish_rate.sleep()
        else:
            self.feat_tx.publish( rospy.Time(0), self.features )
            # NOTE: ROS doesn't like printing numbers larger than 4294967295
            self.feat_tx.publish( rospy.Time(4294967295), self.features ) 
            rospy.spin()

if __name__ == '__main__':
    try:
        of = ConstantFeatures()
        of.Execute()
    except rospy.ROSInterruptException:
        pass


