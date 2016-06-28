#!/usr/bin/env python

import rospy
import numpy as np
from covreg.srv import QueryCovariance

# cmd_features: [x, y, angz]
# floor_mode: [carpet, concrete] (one-hot)
# cam_features: [resolution, framerate] (kpx, hhz)

class CmdFeatures:

    def __init__( self ):
        self.feats = np.zeros(3)

    def set_lin_x( self, x ):
        self.feats[0] = abs( x )

    def set_lin_y( self, y ):
        self.feats[1] = abs( y )

    def set_lin_y( self, z ):
        self.feats[2] = abs( z )

    def get_feats( self ):
        # Return a deep copy
        return np.array( self.feats )

class FloorMode:

    def __init__( self ):
        self.feats = np.zeros(2)
        self.feats[0] = 1

    def set_carpet( self ):
        self.feats.fill( 0 )
        self.feats[0] = 1

    def set_concrete( self ):
        self.feats.fill( 0 )
        self.feats[1] = 1

    def get_feats( self ):
        # Return a deep copy
        return np.array( self.feats )

class CamFeatures:

    def __init__( self ):
        self.feats = np.zeros(2)

    def set_resolution( self, res ):
        # kilo-pixels
        self.feats[0] = res/1000; 

    def set_framerate( self, hz ):
        # hecto-hertz
        self.feats[1] = hz/100; 

    def get_feats( self ):
        # Return a deep copy
        return np.array( self.feats )

class TransitionFeatures:

    def __init__( self ):
        self.cmd = CmdFeatures()
        self.floor = FloorMode()

    def get_feats( self ):
        return np.concatenate( ( self.cmd.get_feats(), 
                                 self.floor.get_feats() ) )

class VisualOdometryFeatures:

    def __init__( self ):
        self.cmd = CmdFeatures()
        self.floor = FloorMode()
        self.cam = CamFeatures()

    def get_feats( self ):
        return np.concatenate( ( self.cmd.get_feats(), 
                                 self.floor.get_feats(),
                                 self.cam.get_feats() ) )

if __name__ == '__main__':

    rospy.init_node( 'covariance_estimator_probe' )

    estimator_ns = rospy.get_param( '~estimator_namespace' )
    if estimator_ns[:-1] != '/':
        estimator_ns += '/'

    query_service_name = estimator_ns + 'query_covariance'
    rospy.wait_for_service( query_service_name )
    query_service = rospy.ServiceProxy( query_service_name, QueryCovariance )

    # Test transition model
    # cmd_features, floor_mode
    trans_feats = TransitionFeatures()

    # Test response to cmd_vel_x
    cmd_x_range = np.arange( 0, 1.0, 0.1 )
    cmd_y_range = [ 0, 0.2, 0.4 ]

    trans_cmd_Qs = []
    for x in cmd_x_range:
        trans_feats.cmd.set_x( x )
        trans_cmd_Qs.append( [] )
        for y in cmd_y_range:
            trans_feats.cmd.set_y( y )
            trans_cmd_Qs[-1].append( query_service( trans_feats ) )


