#!/usr/bin/env python

import rospy, sys
import numpy as np
from percepto_msgs.srv import GetCritique, GetCritiqueRequest, GetCritiqueResponse
from collections import namedtuple

CrossEntropySample = namedtuple( 'CrossEntropySample', ['input', 'output'] )

def PrintArray( a ):
    return np.array_str( a, max_line_width=sys.maxint )

def PrintCES( ces ):
    return 'Input: %s Output: %f' % ( PrintArray( ces.input ), ces.output )

class CrossEntropyParameterOptimization:

    def __init__( self ):

        # Create critique service proxy
        critique_topic = rospy.get_param( '~critic_service' )
        rospy.wait_for_service( critique_topic )
        self.critique_service = rospy.ServiceProxy( critique_topic, GetCritique, True )

        # Read initialization
        self.mean = np.array( rospy.get_param( '~initial_mean' ) )
        self.cov = np.diag( rospy.get_param( '~initial_variances' ) )
        if self.mean.shape[0] != self.cov.shape[0]:
            errstr = 'Mean is %d dimensional but covariance is %d dimensional' % ( self.mean.shape[0],
                                                                                   self.cov.shape[0] )
            raise ValueError( errstr )

        # Read optimization parameters
        self.population_size = rospy.get_param( '~population_size' )
        self.elite_size = rospy.get_param( '~elite_size' )
        self.inflation_scale = rospy.get_param( '~inflation_scale' )
        self.inflation_offset = float( rospy.get_param( '~inflation_offset' ) ) * \
                                np.identity( self.mean.shape[0] )
        
        self.norm_eps = float(rospy.get_param( '~convergence/norm_eps', -float('Inf') ))
        self.max_iters = rospy.get_param( '~convergence/max_iters', float('Inf') )
        self.max_runtime = float(rospy.get_param( '~convergence/max_time', float('Inf') ))
        self.use_correlations = rospy.get_param( '~use_correlations', False )

        self.enable_decay = rospy.get_param( '~enable_decay', False )

        # Initialize state
        self.start_time = rospy.Time.now()
        self.iter_counter = 0
        self.best_seen = CrossEntropySample( input=None, output=-float('Inf') )

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
            self.Log( 'Iteration: %d\n' % (self.iter_counter,) )
            self.Log( 'Mean: %s\n' % PrintArray(self.mean) )
            self.Log( 'Covariance:\n%s\n' % PrintArray(self.cov) )

            # Sample population
            population = np.random.multivariate_normal( self.mean, self.cov, self.population_size )

            # Evaluate population
            samples = []
            for item in population:
                req = GetCritiqueRequest()
                req.input = item
                try:
                    res = self.critique_service.call( req )
                except rospy.ServiceException:
                    raise RuntimeError( 'Could not evaluate item: ' + PrintArray(item) )

                sample = CrossEntropySample( input=item, output=res.critique )
                if sample.output > self.best_seen.output:
                    self.best_seen = sample
                samples.append( sample )
                self.Log( 'Evaluated: ' + PrintCES( sample ) )

            # Get elites from population
            sort_key = lambda x : x.output

            # Note: we want the largest, python sort is default ascending order 
            samples.sort( key=sort_key, reverse=True )
            elites = [ x for x in samples[0:self.elite_size] ]
            elite_inputs = [ x.input for x in elites ]
            elite_outputs = [ x.output for x in elites ]
            
            self.Log( 'Elites for iteration:' )
            for elite in elites:
                self.Log( '\t' + PrintCES( elite ) )

            updated_mean = np.mean( elite_inputs, axis=0 )
            mean_delta = np.linalg.norm( self.mean - updated_mean )
            self.mean = updated_mean

            # Both the offset and the scale decay towards 0 and 1, respectively, over root t
            offset = self.inflation_offset / math.sqrt( self.iter_counter + 1 )
            scale = 1.0 + ( self.inflation_scale - 1.0 ) / math.sqrt( self.iter_counter + 1)
            self.cov = np.cov( elite_inputs, rowvar=False ) * self.inflation_scale \
                       + self.inflation_offset
            if not self.use_correlations:
                self.cov = np.diag( np.diag( self.cov ) )

            avg_output = np.mean( elite_outputs )
            self.Log( 'Average input: %s output: %f' % (PrintArray(self.mean ), avg_output) )
            self.Log( 'Covariance:\n' + PrintArray(self.cov) )
            self.Log( 'Best seen: ' + PrintCES( self.best_seen ) )
            self.iter_counter += 1

            # Check for convergence
            # Check runtime
            now = rospy.Time.now()
            time_elapsed = ( now - self.start_time ).to_sec()
            if time_elapsed > self.max_runtime:
                self.Log( 'Runtime of %f (s) exceeds max runtime of %f.' % ( time_elapsed, self.max_runtime ) )
                return

            # Check num iterations
            if self.iter_counter >= self.max_iters:
                self.Log( 'Iterations %d exceeds max iterations of %d.' % ( self.iter_counter, self.max_iters ) )
                return

            # Check norm
            if mean_delta < self.norm_eps:
                self.Log( 'Mean change norm %f less than min %f.' % ( mean_delta, self.norm_eps ) )
                return

        # Clean up
        self.output_log.close()

if __name__ == '__main__':
    rospy.init_node( 'cross_entropy_parameter_optimizer' )
    cepo = CrossEntropyParameterOptimization()
    cepo.Execute()
