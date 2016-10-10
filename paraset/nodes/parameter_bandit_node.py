#!/usr/bin/env python

import rospy, random
import numpy as np
from bandito import bandits, criterions
from percepto_msgs.srv import GetCritique, GetCritiqueRequest, GetCritiqueResponse
from itertools import izip

class ParameterBanditNode:
    
    def __init__( self ):
        
        # Sample arms
        num_arms = rospy.get_param('~num_arms')
        lower_limits = np.array(rospy.get_param('~params_lower_limits'))
        upper_limits = np.array(rospy.get_param('~params_upper_limits'))
        if len( lower_limits ) != len( upper_limits ):
            raise ValueError( 'Lower and upper limits must have save length.' )
        
        self.arms = [ [ random.uniform(low,upp) for (low,upp) 
                        in izip( lower_limits, upper_limits ) ]
                      for i in range(num_arms) ]
        # Output log
        log_path = rospy.get_param('~output_log')
        self.out_log = open( log_path, 'w' )
        if self.out_log is None:
            raise IOError('Could not open output log at: ' + log_path)

        for i,arm in enumerate(self.arms):
            msg = 'Arm: %d Params: %s\n' % (i, str(arm))
            rospy.loginfo( msg )
            self.out_log.write( msg )
        self.out_log.flush()

        # Create critique service proxy
        critique_topic = rospy.get_param( '~critic_service' )
        rospy.wait_for_service( critique_topic )
        self.critique_service = rospy.ServiceProxy( critique_topic, GetCritique, True )

        self.bandit = bandits.BanditInterface( criterions.ucbv_criterion,
                                               num_arms = num_arms )

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
        while not rospy.is_shutdown():
            ind = self.bandit.ask()
            rospy.loginfo( 'Evaluating arm %d...' % ind )
            reward = self.evaluate_input( self.arms[ind] )
            rospy.loginfo( 'Arm %d returned reward %f' % (ind,reward) )
            self.bandit.tell( ind, reward )
            
            self.out_log.write( 'Round: %d Arm: %d Reward: %f\n' % (i, ind,reward) )
            self.out_log.flush()
            i += 1
        self.out_log.close()

if __name__=='__main__':
    rospy.init_node( 'parameter_bandit_node' )
    pbn = ParameterBanditNode()
    pbn.execute()
