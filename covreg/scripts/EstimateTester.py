#!/usr/bin/env python

import rospy, rosbag, roslib, time, random
from argus_msgs.msg import EstimatePerformance

class EstimatorTester:
    '''Records performance estimates.'''

    def __init__( self ):
        rospy.init_node( 'estimate_tester' )
        self.perf_sub = rospy.Subscriber( 'performance', EstimatePerformance,
                                          self.performance_callback, queue_size=-1 )

        self.avg_lin = 0
        self.avg_ang = 0
        self.counter = 0

        outpath = rospy.get_param( '~output_path' )
        self.outfile = open( outpath, 'w' )

    def write_output( self ):
        print 'Writing output!'
        self.outfile.write( 'average linear vel rms: %f\n' % self.avg_lin )
        self.outfile.write( 'average angular vel rms: %f\n' % self.avg_ang )
        self.outfile.close()

    def performance_callback( self, msg ):
        self.counter += 1
        self.avg_lin = self.avg_lin * (self.counter-1)/self.counter + msg.rms_linear_vel/self.counter
        self.avg_ang = self.avg_ang * (self.counter-1)/self.counter + msg.rms_angular_vel/self.counter

if __name__ == '__main__':
    try:
        bl = EstimatorTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    bl.write_output()
