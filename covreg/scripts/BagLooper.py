#!/usr/bin/env python

import rospy, rosbag, roslib
from collections import namedtuple

class BagLooper:
    '''Loops a bag repeatedly, adjusting header timestamps to be continuous.'''

    def __init__( self ):
        rospy.init_node( 'bag_looper' )
        self.bag_path = rospy.get_param( '~bag_path' )
        self.publishers = {}

        bag = rosbag.Bag( self.bag_path )
        for (topic,topicInfo) in bag.get_type_and_topic_info().topics.iteritems():
            msg_class = roslib.message.get_message_class( topicInfo.msg_type )
            self.publishers[ topic ] = rospy.Publisher( topic, msg_class, 
                                                        queue_size=10 )

        self.bag_start_time = rospy.Time.from_sec( bag.get_start_time() )

    def Execute( self ):

        while not rospy.is_shutdown():

            bag = rosbag.Bag( self.bag_path );
            
            round_start = rospy.Time.now()
            time_offset = round_start - self.bag_start_time
            for topic, msg, t in bag.read_messages():
                
                # Try and avoid getting stuck in the publish call and 
                # timing out on shutdown
                if rospy.is_shutdown():
                    return

                msg_time_from_start = t - self.bag_start_time
                
                try:
                    msg.header.stamp += time_offset
                except AttributeError:
                    pass
                curr_time_from_start = rospy.Time.now() - round_start
                rospy.sleep( msg_time_from_start - curr_time_from_start )

                self.publishers[ topic ].publish( msg )

            bag.close()

if __name__ == '__main__':
    try:
        bl = BagLooper()
        bl.Execute()
    except rospy.ROSInterruptException:
        pass
