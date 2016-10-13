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
        
        # Output log
        log_path = rospy.get_param('~output_log')
        self.out_log = open( log_path, 'w' )
        if self.out_log is None:
            raise IOError('Could not open output log at: ' + log_path)

        num_arms = rospy.get_param('~num_arms', 0)
        b = rospy.get_param('~reward_scale', 1.0)
        c = rospy.get_param('~criteria_c', 1.0)
        beta = rospy.get_param('~beta')

        # Print header
        self.out_log.write('Random seed: %s\n' % str(seed))
        self.out_log.write('Reward scale: %f\n' % b)
        self.out_log.write('Criteria c: %f\n' % c)
        self.out_log.write('Hardness beta: %f\n' % beta)
        self.out_log.write('Init arms: %d\n' % num_arms)
        self.out_log.write('Num rounds: %d\n' % self.num_rounds)

        # Sample arms
        self.param_lower_lims = np.array(rospy.get_param('~params_lower_limits'))
        self.param_upper_lims = np.array(rospy.get_param('~params_upper_limits'))
        if len( self.param_lower_lims ) != len( self.param_upper_lims ):
            raise ValueError( 'Lower and upper limits must have save length.' )
        
        self.arms = []
        for i in range(num_arms):
            self.add_arm()

        self.criterion = lambda h,e: criterions.ucb_v_criterion( history=h,
                                                                 exp_factor=e,
                                                                 reward_scale=b,
                                                                 c=c )
        anytime_mode = rospy.get_param( '~anytime_mode', False )
        if anytime_mode:
            self.air = lambda K, i: criterions.ucb_air_criterion( num_arms=K,
                                                                  round_num=i,
                                                                  beta=beta,
                                                                  max_attainable=True )
        else:
            self.air = None

        self.bandit = bandits.BanditInterface( self.criterion,
                                               arm_func = self.air,
                                               num_arms = num_arms )

        # Create critique service proxy
        critique_topic = rospy.get_param( '~critic_service' )
        rospy.wait_for_service( critique_topic )
        self.critique_service = rospy.ServiceProxy( critique_topic, GetCritique, True )

    def add_arm( self ):
        arm = [ random.uniform(low,upp) for (low,upp) 
                in izip( self.param_lower_lims, self.param_upper_lims ) ]
        self.arms.append( arm )
        msg = 'Arm: %d Params: %s\n' % (len(self.arms)-1, str(arm))
        rospy.loginfo( msg )
        self.out_log.write( msg )
        self.out_log.flush()

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
            if ind >= len( self.arms ):
                self.add_arm()

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