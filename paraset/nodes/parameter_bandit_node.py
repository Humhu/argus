#!/usr/bin/env python

import rospy, random, sys
import numpy as np
from bandito import bandits, criterions
from percepto_msgs.srv import GetCritique, GetCritiqueRequest, GetCritiqueResponse
from itertools import izip

class ParameterBanditNode:
    
    def __init__( self ):
        
        # Seed RNG if specified
        seed = rospy.get_param('~random_seed', None)
        if seed is None:
            rospy.loginfo('No random seed specified. Using system time.')
        else:
            rospy.loginfo('Initializing with random seed: ' + str(seed) )
        random.seed( seed )

        self.num_rounds = rospy.get_param('~num_rounds', float('Inf'))

        # Sample arms
        num_arms = rospy.get_param('~num_arms')
        lower_limits = np.array(rospy.get_param('~params_lower_limits'))
        upper_limits = np.array(rospy.get_param('~params_upper_limits'))
        if len( lower_limits ) != len( upper_limits ):
            raise ValueError( 'Lower and upper limits must have save length.' )
        
        self.arms = [ [ random.uniform(low,upp) for (low,upp) 
                        in izip( lower_limits, upper_limits ) ]
                      for i in range(num_arms) ]

        b = rospy.get_param('~reward_scale', 1.0)
        c = rospy.get_param('~criteria_c', 1.0)
        self.criterion = lambda h,e: criterions.ucbv_criterion(h,e,b,c)
        self.bandit = bandits.BanditInterface( self.criterion,
                                               num_arms = num_arms )
        
        # Output log
        log_path = rospy.get_param('~output_log')
        self.out_log = open( log_path, 'w' )
        if self.out_log is None:
            raise IOError('Could not open output log at: ' + log_path)

        # Print header
        self.out_log.write('Random seed: %s\n' % str(seed))
        self.out_log.write('Reward scale: %f\n' % b)
        self.out_log.write('Criteria c: %f\n' % c)
        self.out_log.write('Num arms: %d\n' % num_arms)
        self.out_log.write('Num rounds: %f\n' % self.num_rounds)

        # Print arms
        for i,arm in enumerate(self.arms):
            msg = 'Arm: %d Params: %s\n' % (i, str(arm))
            rospy.loginfo( msg )
            self.out_log.write( msg )
        self.out_log.flush()

        # Create critique service proxy
        critique_topic = rospy.get_param( '~critic_service' )
        rospy.wait_for_service( critique_topic )
        self.critique_service = rospy.ServiceProxy( critique_topic, GetCritique, True )

    def evaluate_input( self, inval ):
        req = GetCritiqueRequest()
        req.input = inval
        try:
            res = self.critique_service.call( req )
        except rospy.ServiceException:
            raise RuntimeError( 'Could not evaluate item: ' + str( inval ) )
        return res.critique

    def execute( self ):
        i = 0
        while not rospy.is_shutdown() and i < self.num_rounds:
            ind = self.bandit.ask()
            rospy.loginfo( 'Evaluating arm %d...' % ind )
            reward = self.evaluate_input( self.arms[ind] )
            rospy.loginfo( 'Arm %d returned reward %f' % (ind,reward) )
            self.bandit.tell( ind, reward )
            
            self.out_log.write( 'Round: %d Arm: %d Reward: %f\n' % (i, ind,reward) )
            self.out_log.flush()
            i += 1
        rospy.loginfo('Max number of rounds %d reached', i)
        self.out_log.close()

if __name__=='__main__':
    rospy.init_node( 'parameter_bandit_node' )
    pbn = ParameterBanditNode()
    pbn.execute()
    sys.exit(0)