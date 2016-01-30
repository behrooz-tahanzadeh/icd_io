#!/usr/bin/env python
import rospy
import copy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg._Point import Point
from interactive_markers import menu_handler




def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo("button click")
        
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( "menu item: " + str(feedback.menu_entry_id))
        
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo("pose changed")
    """
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
    """
#eof




def makeBox():
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = 0.6
    marker.scale.y = 0.6
    marker.scale.z = 0.2
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.5

    return marker
#eof




def makeBoxControl(intMarker):
    
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox())
    
    intMarker.controls.append( control )
    
    return control
#eof





if __name__=="__main__":
    rospy.init_node("itech_rviz_dest")
    rospy.loginfo("========= itech_rviz_dest =========")
    
    server = InteractiveMarkerServer("rviz_dest")
    
    # create an interactive marker for our server
    intMarker = InteractiveMarker()
    intMarker.header.frame_id = "world"
    intMarker.name = "destination"
    intMarker.description = "Define Quadcopter Destination"
    
    makeBoxControl(intMarker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    intMarker.controls.append(copy.deepcopy(control))
    
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    intMarker.controls.append(control)
    
    mh = MenuHandler()
    mh.insert("Accept Destination", callback=processFeedback)
    mh.insert("Landing Mode", callback=processFeedback)

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Commands"
    control.name = "commands_menu"
    intMarker.controls.append(control)

    
    server.insert(intMarker, processFeedback)
    mh.apply(server, intMarker.name)
    server.applyChanges()

    rospy.spin()