#!/usr/bin/env python

from lookup import LookupInterface
from argus_msgs.msg import FloatVectorStamped
from broadcast.srv import QueryFeatures, QueryFeaturesRequest, QueryFeaturesResponse
from broadcast.Utils import TimeSeries, ros_time_diff

import rospy

# TODO Write receiver class 

class Transmitter:
    '''A wrapper that sets up a feature broadcast and allows publishing to it.'''

    def __init__( self, stream_name, namespace, feature_size, descriptions, mode, topic=None, **kwargs ):
        '''Create a feature broadcast transmitter. 

        Arguments:
        stream_name:  Unique name for this broadcast
        namespace:    Namespace for the transmitter
        feature_size: Dimensionality of the broadcast feature vector
        descriptions: Description of each feature dimension
        topic:        The broadcast topic/service name. Defaults to stream_raw or pull_stream
        mode:         Either 'push' or 'pull'

        Push mode keyword args:
        queue_size:   Publish queue size (10)

        Pull mode keyword argus:
        cache_time:   Length of time to keep data
        '''

        if len(namespace) > 0 and namespace != '~' and namespace[-1] != '/':
            namespace += '/'

        # Register in the lookup directory
        LookupInterface.register_lookup_target( target_name=stream_name,
                                                target_namespace=namespace )

        if topic is None:
            if mode == 'push':
                topic = 'stream_raw'
            elif mode == 'pull':
                topic = 'pull_stream'

        # Populate field info
        topic_path = rospy.resolve_name( namespace + topic )
        rospy.loginfo( 'Registering broadcast (' + stream_name + ') to topic ('
                       + topic_path + ')' )

        self.stream_name = stream_name
        self.mode = mode
        if mode == 'push':
            self.publisher = rospy.Publisher( topic_path,
                                              FloatVectorStamped, 
                                              queue_size=kwargs['queue_size'] )
        elif mode == 'pull':
            self.cache = TimeSeries( diff = ros_time_diff )
            self.cache_time = kwargs['cache_time']
            self.server = rospy.Service( topic_path, 
                                         QueryFeatures, 
                                         self.query_callback )
        else:
            raise ValueError( 'Invalid mode: ' + mode )

        rospy.set_param( namespace + 'feature_size', feature_size )
        rospy.set_param( namespace + 'descriptions', descriptions )
        rospy.set_param( namespace + 'topic', topic_path )
        rospy.set_param( namespace + 'mode', mode )
    
    def publish( self, time, feats ):
        '''Publish a message to the broadcast topic.'''

        if self.mode == 'push':
            msg = FloatVectorStamped()
            msg.header.stamp = time
            msg.header.frame_id = self.stream_name
            msg.values = feats
            self.publisher.publish( msg )

        elif self.mode == 'pull':
            item = FloatVectorStamped()
            item.header.stamp = time
            item.header.frame_id = self.stream_name
            item.values = feats
            self.cache.insert( time, item )

    def query_callback( self, req ):
        if self.mode == 'push':
            raise RuntimeError( 'Query callback called in push mode!' )
        res = QueryFeaturesResponse()
        
        if req.time_mode == QueryFeaturesRequest.CLOSEST_BEFORE:
            res.features = self.cache.get_closest_before( req.query_time ).data
        elif req.time_mode == QueryFeaturesRequest.CLOSEST_AFTER:
            res.features = self.cache.get_closest_after( req.query_time ).data
        elif req.time_mode == QueryFeaturesRequest.CLOSEST_EITHER:
            res.features = self.cache.get_closest( req.query_time ).data
        else:
            raise ValueError( 'Invalid time query mode received' )
        return res



