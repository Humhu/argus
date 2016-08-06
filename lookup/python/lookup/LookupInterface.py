#!/usr/bin/env python

import rospy

def register_lookup_target( target_name, target_namespace, lookup_namespace='/lookup' ):
    if lookup_namespace[-1] != '/':
        lookup_namespace += '/'
    
    resolved_namespace = rospy.resolve_name( target_namespace )
    rospy.loginfo( 'Registering (' + target_name + ') to namespace ('
                   + resolved_namespace + ') to registry (' + lookup_namespace + ')' )
    rospy.set_param( lookup_namespace + target_name, resolved_namespace )

