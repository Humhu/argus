#!/usr/bin/env python

import rospy, rosbag, roslib, time, random
from fieldtrack.srv import ResetFilter

class BagLooper:
    '''Loops a bag repeatedly, adjusting header timestamps to be continuous.'''

    def __init__( self ):
        rospy.init_node( 'bag_looper' )
        bag_path = rospy.get_param( '~bag_paths' )
        
        if type( bag_path ) is str:
            self.bag_paths = [ bag_path ]
        elif type( bag_path ) is list or tuple:
            self.bag_paths = bag_path

        self.start_times = {}
        self.publishers = {}

        reset_srv = rospy.get_param( '~reset_service' )
        rospy.wait_for_service( reset_srv )
        self.reset_service = rospy.ServiceProxy( reset_srv, ResetFilter )
        self.reset_time = rospy.get_param( '~reset_time', 0.0 )

        self.start_offset = rospy.Duration( rospy.get_param( '~start_offset', 0.0 ) )
        self.end_offset = rospy.Duration( rospy.get_param( '~end_offset', 0.0 ) )

        # TODO Debug this!
        self.time_rate = 1.0 / rospy.get_param( '~playback_speed', 1.0 )

        for path in self.bag_paths:
            bag = rosbag.Bag( path )
            rospy.loginfo( 'Checking bag: ' + path )
            
            for (topic,topicInfo) in bag.get_type_and_topic_info().topics.iteritems():
                msg_class = roslib.message.get_message_class( topicInfo.msg_type )
                
                if topic not in self.publishers:
                    rospy.loginfo( 'Advertising topic: ' + topic )
                    self.publishers[ topic ] = rospy.Publisher( topic, 
                                                                msg_class, 
                                                                queue_size=10 )


    def Execute( self ):

        zero_dur = rospy.Duration( 0 )

        while not rospy.is_shutdown():

            self.reset_service( self.reset_time )

        #for path in self.bag_paths:
            path = random.sample( self.bag_paths, 1 )[0]
            rospy.loginfo( 'Playing bag: ' + path )
            bag = rosbag.Bag( path )
            
            bag_start = rospy.Time.from_sec( bag.get_start_time() )
            message_start = bag_start + self.start_offset
            bag_end = rospy.Time.from_sec( bag.get_end_time() )
            message_end = bag_end - self.end_offset
            bag_duration = bag_end - bag_start

            round_start = rospy.Time.now()
            last_time = None
            for topic, msg, t in bag.read_messages():
                
                # Try and avoid getting stuck in the publish call and 
                # timing out on shutdown
                if rospy.is_shutdown():
                    return

                if t < message_start:
                    continue
                if t > message_end:
                    break

                time_from_message_start = t - message_start


                try:
                    msg.header.stamp = time_from_message_start + round_start
                except AttributeError:
                    pass

                curr_from_round_start = rospy.Time.now() - round_start
                sleep_time = time_from_message_start - curr_from_round_start
                rospy.sleep( sleep_time )

                self.publishers[ topic ].publish( msg )

            bag.close()

if __name__ == '__main__':
    try:
        bl = BagLooper()
        bl.Execute()
    except rospy.ROSInterruptException:
        pass
