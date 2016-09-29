#!/usr/bin/env python

import rospy, sys
import numpy as np
from percepto_msgs.srv import GetCritique, GetCritiqueRequest, GetCritiqueResponse
from collections import namedtuple
from itertools import product, izip

TestSample = namedtuple( 'TestSample', ['input', 'output'] )

def PrintSample( ces ):
    return 'Input: %s Output: %f' % ( ces.input, ces.output )

class GridSearchParameterOptimization:

    def __init__( self ):

        # Create critique service proxy
        critique_topic = rospy.get_param( '~critic_service' )
        rospy.wait_for_service( critique_topic )
        self.critique_service = rospy.ServiceProxy( critique_topic, GetCritique, True )

        # Read initialization
        self.grid_mins = rospy.get_param( '~grid_lower_bounds' )
        self.grid_max = rospy.get_param( '~grid_upper_bounds' )
        if len(self.grid_mins) != len(self.grid_max):
            raise ValueError( 'Grid mins and max must have same number of elements' )

        # Set up the grid
        grid_values = []
        if rospy.has_param( '~grid_step_sizes' ):
            step_sizes = rospy.get_param( '~grid_step_sizes' )
            for (lower,upper,step) in izip(self.grid_mins, self.grid_max, step_sizes):
                grid_values.append( np.arange( lower, upper, step ) )
        elif rospy.has_param( '~grid_num_points' ):
            num_points = rospy.get_param( '~grid_num_points' )
            for (lower,upper,num) in izip(self.grid_mins, self.grid_max, num_points):
                grid_values.append( np.linspace( lower, upper, num ) )
        else:
            raise ValueError( 'Must specify step sizes or number of points in grid dimensions.' )
        self.param_iterator = product( *grid_values )

        self.num_samples = rospy.get_param( '~num_samples', 1 )

        # I/O initialization
        output_path = rospy.get_param( '~output_log_path' )
        self.output_log = open( output_path, 'w' )
        if self.output_log is None:
            raise RuntimeError( 'Could not open output log at path: ' + output_path )

    def __del__( self ):
        if self.output_log is not None:
            self.output_log.close()

    def Log( self, msg ):
        rospy.loginfo( msg )
        if self.output_log is not None:
            self.output_log.write( msg + '\n' )
            self.output_log.flush()

    def Execute( self ):

        while not rospy.is_shutdown():
            
            sample_num = 0
            for params in self.param_iterator:

                for i in range( self.num_samples ):
                    req = GetCritiqueRequest()
                    req.input = params
                    try:
                        res = self.critique_service.call( req )
                    except rospy.ServiceException:
                        raise RuntimeError( 'Could not evaluate params: ' + PrintArray(params) )

                    sample = TestSample( input=params, output=res.critique )
                    self.Log( 'Sample: %d %s' % (sample_num, PrintSample( sample ) ) )
                    sample_num += 1
            exit( -1 )

if __name__ == '__main__':
    rospy.init_node( 'grid_parameter_optimizer' )
    cepo = GridSearchParameterOptimization()
    cepo.Execute()
