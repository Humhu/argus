#!/usr/bin/env python

import rospy, rosbag
import cPickle as pickle
from argus_msgs.msg import EstimatePerformance
from collections import namedtuple

EstimateData = namedtuple( 'EstimateData', ['timestamps', 
                                            'linear_rms',
                                            'angular_rms' ] )

if __name__ == '__main__':

    rospy.init_node( 'performance_comparator' )

    bag_path = rospy.get_param( '~bag_path' )
    output_path = rospy.get_param( '~output_path' )
    est_topic = rospy.get_param( '~est_perf_topic' )
    true_topic = rospy.get_param( '~true_perf_topic' )
    start_offset = rospy.get_param( '~start_offset' )
    end_offset = rospy.get_param( '~end_offset' )

    bag = rosbag.Bag( bag_path )
    topics_dict = bag.get_type_and_topic_info().topics
    if est_topic not in topics_dict:
        print 'Estimated performance topic: ' + est_topic +
              ' not found in bag: ' + bag_path
    if true_topic not in topics_dict:
        print 'True performance topic: ' + true_topic +
              ' not found in bag: ' + bag_path
    
    est = EstimateData()
    est.timestamps = []
    est.linear_rms = []
    est.angular_rms = []

    true = EstimateData()
    true.timestamps = []
    true.linear_rms = []
    true.angular_rms = []

    bag_topics = [ est_topic, true_topic ]
    
    start_time = bag.get_start_time()
    offset_start_time = start_time + rospy.Duration( start_offset )
    end_time = bag.get_end_time()
    offset_end_time = end_time - rospy.Duration( end_offset )

    for topic, msg, t in bag.read_messages( topics=bag_topics ):
        
        if t < offset_start_time:
            continue
        elif t > offset_end_time:
            break

        if topic == true_topic:
            true.timestamps.append( t.toSec() )
            true.linear_rms.append( msg.rms_linear_vel )
            true.angular_rms.append( msg.rms_angular_vel )

        elif topic == est_topic:
            est.timestamps.append( t.toSec() )
            est.linear_rms.append( msg.rms_linear_vel )
            est.angular_rms.append( msg.rms_angular_vel )

    bag.close()

    print 'Read {} estimated messages and {} true messages'.format( len( est.timestamps ),
                                                                    len( true.timestamps ) )

    print 'Storing output as dict at: ' + output_path
    outdict = { 'est' : est, 'true' : true }
    pickle.dump( outdict, output_path )