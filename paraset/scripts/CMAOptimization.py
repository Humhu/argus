#!/usr/bin/env python

import rospy, sys, cma
import numpy as np
from percepto_msgs.srv import GetCritique, GetCritiqueRequest, GetCritiqueResponse
from collections import namedtuple

CrossEntropySample = namedtuple( 'CrossEntropySample', ['input', 'output'] )

def PrintArray( a ):
    return np.array_str( a, max_line_width=sys.maxint )

def PrintCES( ces ):
    return 'Input: %s Output: %f' % ( PrintArray( ces.input ), ces.output )

class CMAOptimization:
    '''Wrapper around a Covariance Matrix Adaptation (CMA) evolution strategy optimizer.'''

    def __init__( self ):

        # Create critique service proxy
        critique_topic = rospy.get_param( '~critic_service' )
        rospy.wait_for_service( critique_topic )
        self.critique_service = rospy.ServiceProxy( critique_topic, GetCritique, True )

        # Read initialization
        self.mean = rospy.get_param( '~initial_mean' )
        self.cov = rospy.get_param( '~initial_variances' )

        # Custom termination conditions
        self.max_iters = rospy.get_param( '~convergence/max_iters', float('Inf') )
        self.max_runtime = float(rospy.get_param( '~convergence/max_time', float('Inf') ) )

        # Read optimization parameters
        self.num_restarts = rospy.get_param( '~num_restarts', 0 )

        self.cma_options = cma.CMAOptions()
        self.cma_options['popsize'] = rospy.get_param( '~population_size' )
        self.cma_options['tolfun'] = float( rospy.get_param( '~convergence/output_change', -float('Inf') ) )
        self.cma_options['tolx'] = float( rospy.get_param( '~convergence/input_change', -float('Inf') ) )
        if rospy.get_param( '~diagonal_only', True ):
            self.cma_options['CMA_diagonal'] = True
        self.cma_options['termination_callback'] = self.HasTerminated

        # I/O initialization
        output_path = rospy.get_param( '~output_log_path' )
        self.output_log = open( output_path, 'w' )
        if self.output_log is None:
            raise RuntimeError( 'Could not open output log at path: ' + output_path )
        self.loggers = []

    def __del__( self ):
        if self.output_log is not None:
            self.output_log.close()

    def Log( self, msg ):
        rospy.loginfo( msg )
        if self.output_log is not None:
            self.output_log.write( msg + '\n' )
            self.output_log.flush()

    def OptimizationIteration( self ):
        
        # Initialize state
        self.start_time = rospy.Time.now()
        self.iter_counter = 0
        optimizer = cma.CMAEvolutionStrategy( self.mean, self.cov, self.cma_options )
        while not optimizer.stop() and not rospy.is_shutdown():
            inputs = optimizer.ask()
            optimizer.tell( [ inputs, self.EvaluateInput( inval ) for inval in inputs ] )
            optimizer.logger.add()
            optimizer.disp()
        self.loggers.append( optimizer.logger )
        return optimizer.best

    def EvaluateInput( self, inval ):
        req = GetCritiqueRequest()
        req.input = inval
        try:
            res = self.critique_service.call( req )
        except rospy.ServiceException:
            raise RuntimeError( 'Could not evaluate item: ' + PrintArray( inval ) )
        sample = CrossEntropySample( input=, output=res.critique )
        self.Log( 'Evaluated: ' + PrintCES( sample ) )
        return res.critique

    def HasTerminated( self ):
        runtime_exceeded = (rospy.Time.now() - self.start_time).to_sec() > self.max_runtime
        iters_exceeded = self.iter_counter > self.max_iters
        return runtime_exceeded or iters_exceeded

    def Execute( self ):

        best_solution = cma.BestSolution()

        while not rospy.is_shutdown():
            for restart_iter in range( 0, self.num_restarts + 1 ):
                result = OptimizationIteration()
                self.Log( 'Iteration best: input: %s output: %f' % ( PrintArray( result.x ), 
                                                                                 result.f ) )
                best_solution.update( results )
                self.Log( 'Best so far: input: %s output: %f' % ( PrintArray( best_solution.x ), 
                                                                  best_solution.f ) )
        # Clean up
        self.output_log.close()

if __name__ == '__main__':
    rospy.init_node( 'cma_optimizer' )
    cepo = CMAOptimization()
    cepo.Execute()
