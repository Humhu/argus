#!/usr/bin/env python

import rospy
from broadcast.msg import StampedFeatures
from broadcast import Broadcasters as Trx

class ConstantFeatures:
    '''Publishes a constant feature at a fixed rate.'''

    def __init__( self ):
        rospy.init_node( 'constant_feature_publisher' )
        
        feature_name = rospy.get_param( '~feature_name' )
        descriptions = rospy.get_param( '~feature_descriptions' )
        features = rospy.get_param( '~features' )

        self.feature_msg = StampedFeatures()
        self.feature_msg.header.frame_id = feature_name
        self.feature_msg.features = features

        rate = rospy.get_param( '~publish_rate', 1.0 )
        self.publish_rate = rospy.Rate( rate );

        self.feat_tx = Trx.Transmitter( broadcast_name=feature_name,
                                        feature_size=len( features ),
                                        feature_descriptions=descriptions )

    def Execute( self ):

        while not rospy.is_shutdown():

            self.feature_msg.header.stamp = rospy.Time.now()
            self.feat_tx.publish( self.feature_msg )
            self.publish_rate.sleep()

if __name__ == '__main__':
    try:
        of = ConstantFeatures()
        of.Execute()
    except rospy.ROSInterruptException:
        pass


