#!/usr/bin/env python

import rospy, rosbag, roslib, time, random, sys
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
        self.reset_lead = rospy.Duration( rospy.get_param( '~reset_lead', 0.0 ) )
        self.sample_random = rospy.get_param( '~sample_random', False )
        self.offset_random = rospy.get_param( '~offset_random', False )
        self.playback_length = rospy.Duration( rospy.get_param( '~playback_length', sys.maxint ) )

        self.start_offset = rospy.Duration( rospy.get_param( '~start_offset', 0.0 ) )
        self.end_offset = rospy.Duration( rospy.get_param( '~end_offset', 0.0 ) )

        # TODO Debug this!
        self.time_rate = 1.0 / rospy.get_param( '~playback_speed', 1.0 )

        self.bags = {}
        self.times = {}

        for path in self.bag_paths:
            bag = rosbag.Bag( path )

            rospy.loginfo( 'Parsing bag: ' + path )
            self.bags[path] = list( bag.read_messages() )
            rospy.loginfo( 'Parsing complete.' )

            self.times[path] = ( bag.get_start_time(), bag.get_end_time() )

            for (topic,topicInfo) in bag.get_type_and_topic_info().topics.iteritems():
                msg_class = roslib.message.get_message_class( topicInfo.msg_type )
                
                if topic not in self.publishers:
                    rospy.loginfo( 'Advertising topic: ' + topic )
                    self.publishers[ topic ] = rospy.Publisher( topic, 
                                                                msg_class, 
                                                                queue_size=10 )


    def Execute( self ):

        zero_dur = rospy.Duration( 0 )

        ind = 0
        while not rospy.is_shutdown():

            round_start = rospy.Time.now()
            rospy.loginfo( 'Round start: %f', round_start.to_sec() )

            self.reset_service( 0.0, round_start - self.reset_lead )

            if self.sample_random:
                path = random.sample( self.bag_paths, 1 )[0]
            else:
                if ind >= len( self.bag_paths ):
                    ind = 0
                path = self.bag_paths[ind]
                ind += 1

            rospy.loginfo( 'Playing bag: ' + path )
            bag = self.bags[path]

            ( bag_start, bag_end ) = self.times[path]
            bag_start = rospy.Time.from_sec( bag_start )
            bag_end = rospy.Time.from_sec( bag_end )

            message_start = bag_start + self.start_offset
            message_end = bag_end - self.end_offset
            message_duration = message_end - message_start

            if self.playback_length < message_duration:
                play_duration = self.playback_length
            else:
                play_duration = message_duration

            nonplay_duration = message_duration - play_duration
            if self.offset_random:
                offset = rospy.Duration( nonplay_duration.to_sec() * random.random() )
            else:
                offset = rospy.Duration( 0 )
            rospy.loginfo( 'Playing %f seconds offset %f', play_duration.to_sec(), offset.to_sec() )
            trunc_bag_start = message_start + offset
            trunc_bag_end = trunc_bag_start + play_duration

            last_time = None

            rospy.loginfo('Playing %d messages', len(bag))

            for (topic, msg, t) in bag:
                
                # Try and avoid getting stuck in the publish call and 
                # timing out on shutdown
                if rospy.is_shutdown():
                    return

                try:
                    stamp = msg.header.stamp
                except AttributeError:
                    stamp = t

                if stamp < trunc_bag_start:
                    rospy.loginfo('Skipping message stamp %f from start %f', stamp.to_sec(), trunc_bag_start.to_sec() )
                    continue
                elif stamp > trunc_bag_end:
                    rospy.loginfo('Reached end of bag at stamp %f with end %f', stamp.to_sec(), trunc_bag_end.to_sec() )
                    break

                time_from_trunc_start = stamp - trunc_bag_start
                if time_from_trunc_start.to_sec() < 0:
                    raise RuntimeError('Time less than 0!')
                
                # NOTE Since msg is a reference, we need to restore its timestamp afterwards
                try:
                    old_stamp = msg.header.stamp
                    msg.header.stamp = time_from_trunc_start + round_start
                except AttributeError:
                    pass

                curr_from_round_start = rospy.Time.now() - round_start
                sleep_time = time_from_trunc_start - curr_from_round_start
                rospy.sleep( sleep_time )

                # try:
                #     print 'Publishing %s at time %f' % (topic, msg.header.stamp.to_sec() )
                # except AttributeError:
                #     pass

                self.publishers[ topic ].publish( msg )

                try:
                    msg.header.stamp = old_stamp
                except AttributeError:
                    pass

            # bag.close()

if __name__ == '__main__':
    try:
        bl = BagLooper()
        bl.Execute()
    except rospy.ROSInterruptException:
        pass
