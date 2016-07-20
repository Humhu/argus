#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from broadcast.msg import StampedFeatures
from broadcast import Broadcasters as Trx

class OnlineMoments:
    '''Implementation of the Welford online moment algorithm.'''

    def __init__( self ):
        self.Reset()

    def Reset( self ):
        self.M1 = None
        self.M2 = None
        self.count = 0

    def Accumulate( self, x ):
        self.count += 1

        if self.M1 is None:
            self.M1 = x;
            self.M2 = np.zeros(len(x))
            return
        
        delta = x - self.M1
        self.M1 += delta / self.count
        self.M2 += delta * (x - self.M1)

    def GetMean( self ):
        return self.M1

    def GetVariance( self ):
        if self.count < 2:
            return np.ones( len(self.M2) )
        else:
            return self.M2 / (self.count - 1)

class OnlineMax:

    def __init__( self, slew_rate=1.0 ):
        self.max_seen = None
        self.slew_rate = slew_rate

    def Accumulate( self, x ):
        if self.max_seen is None:
            self.max_seen = x
        else:
            seen_less = self.max_seen < x
            deltas = x - self.max_seen
            deltas[ deltas < 0 ] = 0
            self.max_seen += deltas * self.slew_rate
        return self.max_seen

class MinMaxConditioner:

    def __init__( self, slew_rate ):
        self.max_seen = OnlineMax( slew_rate )
        self.min_seen = OnlineMax( slew_rate )

    def Condition( self, x ):
        max_scale = self.max_seen.Accumulate( x )
        min_scale = self.min_seen.Accumulate( -x )
        scale = (max_scale - min_scale)/2
        scale[ scale == 0 ] = 1.0

        offset = (max_scale + min_scale)/2
        return (x - offset)/scale

class UnitVarianceConditioner:

    def __init__( self ):
        self.moments = OnlineMoments()

    def Condition( self, x ):
        self.moments.Accumulate( x )
        offset = self.moments.GetMean()
        scale = self.moments.GetVariance()
        scale[ scale == 0 ] = 1.0
        return (x - offset)/scale

class FeatureNoiser:
    '''Publishes features calculated from an twist topic.'''

    def __init__( self ):
        rospy.init_node( 'feature_noiser' )

        self.feature_name = rospy.get_param( '~feature_name' )
        self.feat_sub = rospy.Subscriber( 'features', StampedFeatures, self.FeaturesCallback )

        self.abs_val = rospy.get_param( '~absolute_value' )
        # self.conditioner = MinMaxConditioner( 0.1 )
        # self.conditioner = UnitVarianceConditioner()
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

        # First absolute value before any operations
        if self.abs_val:
            feats = np.abs( feats )

        # Whiten features
        # feats = self.conditioner.Condition( feats )

        # Adding noise disabled for now
        noise = self.noise_mag * ( np.random.rand( len( feats ) ) - 0.5 )
        feats = feats + noise

        out.features = feats
        self.feat_tx.publish( out )

if __name__ == '__main__':
    try:
        of = FeatureNoiser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


