#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from broadcast.msg import StampedFeatures
from broadcast import Broadcasters as Trx

class FeatureNoiser:
    '''Publishes features calculated from an twist topic.'''

    def __init__( self ):
        rospy.init_node( 'feature_noiser' )

        self.feature_name = rospy.get_param( '~feature_name' )
        self.feat_sub = rospy.Subscriber( 'features', StampedFeatures, self.FeaturesCallback )

        self.abs_val = rospy.get_param( '~absolute_value' )
        self.offset = np.array( rospy.get_param( '~offset' ) )
        self.noise_mag = np.array( rospy.get_param( '~noise_magnitude' ) )

        self.feature_dim = rospy.get_param( '~feature_dim' )
        self.feat_tx = Trx.Transmitter( broadcast_name=self.feature_name,
                                        feature_size=self.feature_dim,
                                        feature_descriptions=[] )

    def FeaturesCallback( self, tw ):
        out = StampedFeatures()
        out.header = tw.header
        out.header.frame_id = self.feature_name
        feats = np.array( tw.features )

        if self.abs_val:
            feats = np.abs( feats )

        noise = self.noise_mag * ( np.random.rand( len( feats ) ) - 0.5 )
        feats = feats + self.offset + noise

        out.features = feats
        self.feat_tx.publish( out )

if __name__ == '__main__':
    try:
        of = FeatureNoiser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


