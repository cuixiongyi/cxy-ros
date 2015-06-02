#!/usr/bin/env python

import roslib 
import argparse
import numpy as np
import sys
import yaml
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from interactive_markers import *
from interactive_markers.interactive_marker_server import *
import tf
from tf.broadcaster import TransformBroadcaster
from visualization_msgs.msg import *
#import transform
class CalibrationHelper():
    """
    Calibration Helper class.
    This class allows a calibration to occur! It sets up an interactive marker
    server with use with rviz for manual calibration. It keeps publishing the
    desired transform, as it moves.
    """
    def __init__(self, parent_frame="/camera_link", child_frame="/robot", pose_initial=None, rate=100.0, invert=False):
        # Store useful fields
        self._server = InteractiveMarkerServer("calibration_helper_{}_{}".format(parent_frame, child_frame))
        self._tf_broadcaster = TransformBroadcaster()
        self._parent_frame = parent_frame
        self._child_frame = child_frame
        self.pose = pose_initial
        self._rate = rate
        self._invert = invert
    def start(self):
        """
        Starts the the calibration process - creates a box to calibrate
        with in rviz, and also sets up a timer to regularly publish pose
        messages.
        """
        # Create a box
        self._make6DofMarker()
        # Set up a regular timer to publish the pose!
        rospy.Timer(rospy.Duration(1.0 / self._rate), self._publish_pose)
    def _publish_pose(self, msg=None):
        """ Called regularly on a timer to publish transform over tf """
        (x, y, z, q_x, q_y, q_z, q_w) = (self.pose.position.x,
                                         self.pose.position.y,
                                         self.pose.position.z,
                                         self.pose.orientation.x,
                                         self.pose.orientation.y,
                                         self.pose.orientation.z,
                                         self.pose.orientation.w)
        self._tf_broadcaster.sendTransform((x, y, z),
                    (q_x, q_y, q_z, q_w),
                    rospy.Time.now(),
                    self._child_frame,
                    self._parent_frame)
    def _marker_moved_callback(self, feedback):
        """ Callback method for when an interactive marker moved in rviz """
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if not self._invert:
                self.pose = feedback.pose
            else:
                self.pose = invert_pose(feedback.pose)
    def _makeBox(self, msg):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        return marker
    def _makeBoxControl(self, msg):
        """ Helper to make a box control """
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self._makeBox(msg))
        msg.controls.append(control)
        return control
    def _make6DofMarker(self):
        """ Helper to make a 6 DOF marker for rviz, taken from ROS tutorial."""
        int_marker = InteractiveMarker()
        # Set the initial pose and parent frame for the marker
        if not self._invert:
            int_marker.header.frame_id = self._parent_frame
            int_marker.pose = self.pose
        else:
            int_marker.header.frame_id = self._child_frame
            int_marker.pose = invert_pose(self.pose)
        int_marker.scale = 0.25
        int_marker.name = "calibration_handle"
        int_marker.description = "Move to calibrate"
        self._makeBoxControl(int_marker)
        # Add X rotation
#        control = InteractiveMarkerControl()
#        control.orientation.w = 1
#        control.orientation.x = 1
#        control.orientation.y = 0
#        control.orientation.z = 0
#        control.name = "rotate_x"
#        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
#        int_marker.controls.append(control)
        # Add X translation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        # Add Z rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        # Add Z translation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        # Add Y rotation
#        control = InteractiveMarkerControl()
#        control.orientation.w = 1
#        control.orientation.x = 0
#        control.orientation.y = 0
#        control.orientation.z = 1
#        control.name = "rotate_y"
#        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
#        int_marker.controls.append(control)
        # Add Y translation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        self._server.insert(int_marker, self._marker_moved_callback)
        self._server.applyChanges()
class StaticTransformPublisher():
    """
    Static Transform publisher class.
    This class doesn't do any calibrating - but it does use the last previously
    saved calibration. It just keeps publishing it over and over.
    """
    def __init__(self, parent_frame="/camera_link",
                 child_frame="/robot", pose=None, rate=100.0):
        # Store useful fields
        self._tf_broadcaster = TransformBroadcaster()
        self._parent_frame = parent_frame
        self._child_frame = child_frame
        self.pose = pose
        self._rate = rate
    def start(self):
        """
        Begins publishing the static pose at the desired rate!
        """
        # Set up a regular timer to publish the pose!
        rospy.Timer(rospy.Duration(1.0 / self._rate), self._publish_pose)
    def _publish_pose(self, msg=None):
        """ Called regularly via a timer to publish transform over tf """
        (x, y, z, q_x, q_y, q_z, q_w) = (self.pose.position.x,
                                         self.pose.position.y,
                                         self.pose.position.z,
                                         self.pose.orientation.x,
                                         self.pose.orientation.y,
                                         self.pose.orientation.z,
                                         self.pose.orientation.w)
        self._tf_broadcaster.sendTransform((x, y, z),
                    (q_x, q_y, q_z, q_w),
                    rospy.Time.now(),
                    self._child_frame,
                    self._parent_frame)
def load_pose_from_file(filename):
    """
    Helper function that loads a pose from a yaml file if it exists, otherwise
    returns the identity pose.
    """
    p = Pose()
    from_file = False
    try:
        with open(filename) as f:
            d = yaml.safe_load(f.read())
        p.position.x = d['x']
        p.position.y = d['y']
        p.position.z = d['z']
        p.orientation.x = d['q_x']
        p.orientation.y = d['q_y']
        p.orientation.z = d['q_z']
        p.orientation.w = d['q_w']
        from_file = True
    except:
        # Caught an exception - probably the file doesn't exist. Return identity
        p.position.x = 0
        p.position.y = 0
        p.position.z = 0
        p.orientation.x = 0
        p.orientation.y = 0
        p.orientation.z = 0
        p.orientation.w = 1
        from_file = False
    return (p, from_file)
def save_pose_to_file(p, filename):
    """ Helper function that saves a pose to a yaml file. """
    # Convert to a dictionary which will be yaml-ified
    d = {}
    d['x'] = float(p.position.x) # We must explicitly cast to a float, otherwise
    d['y'] = float(p.position.y) # we get a numpy float type that doesn't serialize
    d['z'] = float(p.position.z) # well with yaml.
    d['q_x'] = float(p.orientation.x)
    d['q_y'] = float(p.orientation.y)
    d['q_z'] = float(p.orientation.z)
    d['q_w'] = float(p.orientation.w)
    with open(filename, 'w') as f:
        f.write(yaml.dump(d))
#def invert_pose(pose):
#    """ Helper function to invert a transform """
#    T = transform.pose_to_homogeneous_matrix(pose)
#    T_inv = tf.transformations.inverse_matrix(T)
#    p_inv = transform.homogeneous_matrix_to_pose(T_inv)
#    return p_inv
"""
Main
"""
if __name__=="__main__":
    rospy.init_node("rviz_transform_publisher")
    # Retrieve parameters via ROS parameter server.
    # calibration_mode - If this flag is set, will enter calibration mode and
    # publish interactive markers to rviz. Otherwise, will just statically
    # publish the same transform over and over.
    calib_mode = rospy.get_param('~calibration_mode', False)
    calib_mode = True
    # parent_frame. The name of the parent frame that this node will
    # publish from. This is the parent frame in ROS.
    if rospy.has_param('~parent_frame'):
        parent_frame = rospy.get_param('~parent_frame')
    else:        
        parent_frame = "head"
#        rospy.logerr('Error: parent_frame ROS parameter not specified.')
#        sys.exit(1)
    # child_frame. The name of the child frame that this node will publish
    if rospy.has_param('~child_frame'):
        child_frame = rospy.get_param('~child_frame')
    else:
        child_frame = "model_guess_frame"
#        rospy.logerr('Error: child_frame ROS parameter not specified.')
#        sys.exit(1)
    # calibration_file. The name of the file where the calibration data will be
    # either stored or read from.
    if rospy.has_param('~transform_file'):
        calib_file = rospy.get_param('~transform_file')
    else:
        calib_file = "/home/nbanerjee/calib.txt"
#        rospy.logerr('Error: calibration file not specified.')
#        sys.exit(1)        
    # invert. If this option is specified, the transform read in the calibration
    # file will be inverted. The parent and child frames are not modified.
    invert = rospy.get_param('~invert', False)
    #invert = True
    # rate. The rate (in Hz) that this the transform will be published.
    rate = rospy.get_param('~rate', 100.0)
    # Load the transform from the file, and invert it if necessary
    pose, s = load_pose_from_file(calib_file)
    if invert:
        pose = invert_pose(pose)
    rospy.loginfo("Calibration Helper!")
    if calib_mode:
        rospy.loginfo("Operating in calibration mode")
    else:
        rospy.loginfo("Operating in static transform mode")
    rospy.loginfo("    Parent: {}".format(parent_frame))
    rospy.loginfo("    Child:    {}".format(child_frame))
    rospy.loginfo("    Rate:      {} Hz".format(rate))
    if s:
        rospy.loginfo("    Loaded transform from {}".format(calib_file))
    else:
        rospy.logerr("    Error: Couldn't load transform from {} - defaulting to identify".format(calib_file))
    if invert:
        rospy.loginfo("    Inverting the transform found in file")
    # Time to start. Depending on the mode, start doing stuff.
    if calib_mode:
        rospy.loginfo("     In calib mode.")
        calib = CalibrationHelper(parent_frame=parent_frame, child_frame=child_frame, pose_initial=pose, rate=rate, invert=invert)
        calib.start()
    else:
        rospy.loginfo("     Not in calib mode. Just publishing the static transform.")
        stat = StaticTransformPublisher(parent_frame=parent_frame, child_frame=child_frame, pose=pose, rate=rate)
        stat.start()
    rospy.spin()
    if calib_mode:
        # Save the calibration to a file!
        # Save the *un-inverted* transform. That way, when we invert it upon
        # reading it back in again (if desired), things will work out.
        if not invert:
            save_pose_to_file(calib.pose, calib_file)
        else:
            save_pose_to_file(invert_pose(calib.pose), calib_file)
        rospy.loginfo("Saved calibration from {} to {} in file {}.".format(parent_frame, child_frame, calib_file))

