#!/usr/bin/env python

import rospy
from threading import Lock
from percepto_msgs.srv import GetCritique, GetCritiqueRequest, GetCritiqueResponse
from percepto_msgs.srv import SetParameters, SetParametersRequest
from percepto_msgs.msg import RewardStamped
from paraset.srv import StartEvaluation

class EmpiricalParameterEvaluator:

    def __init__( self ):

        # Create parameter setter proxy
        setter_topic = rospy.get_param( '~parameter_set_service' )
        rospy.wait_for_service( setter_topic )
        self.setter_proxy = rospy.ServiceProxy( setter_topic, SetParameters )

        # Create evaluation trigger proxy
        evaluation_topic = rospy.get_param( '~start_evaluation_service' )
        rospy.wait_for_service( evaluation_topic )
        self.evaluation_proxy = rospy.ServiceProxy( evaluation_topic, StartEvaluation )

        # Subscribe to reward topic
        self.reward_lock = Lock()
        self.reward_acc = 0
        self.reward_duration = 0
        self.last_reward = None
        self.reward_sub = rospy.Subscriber( 'reward', RewardStamped, self.RewardCallback )

        # Create critique service
        self.critique_service = rospy.Service( '~get_critique', GetCritique, self.CritiqueCallback )

    def CritiqueCallback( self, req ):
        
        # First reset reward accumulator
        with self.reward_lock:
            self.reward_acc = 0
            self.reward_duration = 0

        # Call parameter setter
        preq = SetParametersRequest()
        preq.parameters = req.input
        try:
            self.setter_proxy.call( preq )
        except rospy.ServiceException:
            rospy.logerror( 'Could not set parameters to %s', (str(req.parameters),) )
            return None

        # Wait until evaluation is done
        try:
            self.evaluation_proxy.call()
        except rospy.ServiceException:
            rospy.logerror( 'Could not begin evaluation.' )
            return None

        res = GetCritiqueResponse()
        with self.reward_lock:
            if self.reward_duration == 0:
                rospy.logerr( 'No rewards received during evaluation period!' )
                return None
            res.critique = self.reward_acc / self.reward_duration
        return res

    def RewardCallback( self, msg ):
        with self.reward_lock:
            if self.last_reward is None:
                self.last_reward = msg
                return

            dt = ( msg.header.stamp - self.last_reward.header.stamp ).to_sec()
            self.reward_acc += ( msg.reward + self.last_reward.reward ) * 0.5 * dt
            self.reward_duration += dt
            self.last_reward = msg

if __name__ == '__main__':
    rospy.init_node( 'empirical_parameter_evaluator' )
    try:
        epe = EmpiricalParameterEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
