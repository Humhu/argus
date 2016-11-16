#!/usr/bin/env python

import rospy, sys, random
import numpy as np
import cPickle as pickle
from percepto_msgs.srv import SetParameters, SetParametersRequest
from percepto_msgs.msg import RewardStamped
from itertools import izip
from collections import namedtuple
from threading import Lock
from paraset.srv import StartEvaluation

Sample = namedtuple( 'Sample', ['init_input', 'end_input', 'output'] )

def PrintSample( ces ):
    return 'init_input: %s end_input: %s output: %f' % ( str( ces.init_input ),
                                                         str( ces.end_input ), 
                                                         ces.output )

class StepResponse:

    def __init__( self ):

        # I/O initialization
        output_path = rospy.get_param( '~output_data_path' )
        self.output_data = open( output_path, 'wb' )
        if self.output_data is None:
            raise RuntimeError( 'Could not open output data at path: ' + output_path )

        # Step parameters
        self.input_lower = rospy.get_param( '~init_lower_bound' )
        self.input_upper = rospy.get_param( '~init_upper_bound' )
        if len(self.input_lower) != len(self.input_upper):
            raise ValueError( 'Upper and lower bounds must have same length.' )

        self.end_input = rospy.get_param( '~end_mean' )
        self.num_samples = rospy.get_param( '~num_samples' )
        self.step_time = rospy.Duration( float( rospy.get_param( '~step_time') ) )

        if rospy.has_param( '~random_seed' ):
            seed = rospy.get_param( '~random_seed')
            random.seed( seed )
            print( 'Seeding RNG with: %d' % seed )

        # Recording state
        self.mutex = Lock()
        self.traj_start_time = rospy.Time.now()
        self.current_input = None
        self.last_reward = None
        self.traj = []

        # Create evaluation trigger proxy
        evaluation_topic = rospy.get_param( '~start_evaluation_service' )
        rospy.wait_for_service( evaluation_topic )
        self.evaluation_proxy = rospy.ServiceProxy( evaluation_topic, StartEvaluation )

        # Create parameter setting service proxy
        set_topic = rospy.get_param( '~set_service' )
        rospy.wait_for_service( set_topic )
        self.set_service = rospy.ServiceProxy( set_topic, SetParameters, True )

       # Subscribe to reward topic
        self.reward_sub = rospy.Subscriber( 'reward', RewardStamped, self.RewardCallback )

    def SampleInput( self ):
        return [ random.uniform(bound[0], bound[1]) 
                 for bound in izip( self.input_lower, self.input_upper ) ]

    def SetParameters( self, p ):
        req = SetParametersRequest()
        req.parameters = p
        try:
            self.set_service.call( req )
        except rospy.ServiceException:
            rospy.logerror( 'Could not set parameters to %s' % str(req.parameters) )
        
        with self.mutex:
            self.current_input = p

    def RewardCallback( self, msg ):
        with self.mutex:
            dt = (msg.header.stamp - self.traj_start_time).to_sec()
            self.traj.append( (dt, self.current_input, msg.reward) )

    def StepTimerCallback( self, event ):
        self.SetParameters( self.end_input )

    def RunTrajectory( self ):
        # Init 
        self.SetParameters( self.SampleInput() )
        self.traj = []
        with self.mutex:
            self.traj_start_time = rospy.Time.now()

        timer = rospy.Timer( self.step_time, self.StepTimerCallback, True )

        try:
            self.evaluation_proxy.call()
        except rospy.ServiceException:
            rospy.logerror( 'Could not begin evaluation.' )
            return None

        return list(self.traj)

    def Execute( self ):
        trials = []
        for i in range(self.num_samples):
            print( 'Generating sample %d...' % i )
            trials.append( self.RunTrajectory() )

        # Clean up
        pickle.dump( trials, self.output_data )
        self.output_data.close()

if __name__ == '__main__':
    rospy.init_node( 'step_response_characterizer' )
    cepo = StepResponse()
    cepo.Execute()
    sys.exit(0)