#!/usr/bin/env python

from lookup import LookupInterface
from broadcast.msg import StampedFeatures

import rospy

# TODO Write receiver class 

class Transmitter:
    '''A wrapper that sets up a feature broadcast and allows publishing to it.'''

    def __init__( self, broadcast_name, feature_size, feature_descriptions,
                  namespace='~', topic='features_raw', outgoing_queue_size=10 ):
        '''Create a feature broadcast transmitter. 

        Arguments:
        broadcast_name -- Unique name for this broadcast
        feature_size -- Dimensionality of the broadcast feature vector
        feature_descriptions -- Description of each feature dimension
        namespace -- The namespace this broadcast should be placed in (~)
        topic -- The broadcast topic name, default (features_raw)
        outgoing_queue_size -- Publish queue size (10)
        '''

        # By default we always publish on topic '~features_raw'
        if namespace != '~' and namespace[-1] != '/':
            namespace += '/'
        topic_name = rospy.resolve_name( namespace + topic )
        
        # Look for an alternate lookup namespace param
        lookup_ns = rospy.get_param( '~lookup_namespace', '/lookup/' )
        if lookup_ns[-1] != '/':
            lookup_ns += '/'

        rospy.loginfo( 'Registering broadcast (' + broadcast_name + ') to topic ('
                       + topic_name + ') to registry (' + lookup_ns + ')' )

        # Register our namespace to our broadcast name on the global lookup
        LookupInterface.register_lookup_target( target_name=broadcast_name,
                                                target_namespace=namespace,
                                                lookup_namespace=lookup_ns )

        self.publisher = rospy.Publisher( topic_name,
                                          StampedFeatures, 
                                          queue_size=outgoing_queue_size )
        rospy.set_param( namespace + 'feature_size', feature_size )
        rospy.set_param( namespace + 'feature_descriptions', feature_descriptions )
    
    def publish( self, msg ):
        '''Publish a message to the broadcast topic.'''

        self.publisher.publish( msg )


