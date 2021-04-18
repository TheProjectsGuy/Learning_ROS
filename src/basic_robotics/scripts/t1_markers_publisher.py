"""
    A node that publishes markers and a fixed transformation
        1. Publish a fixed transformation (on /tf_static) for {f2} -> {f3}
        2. Publish a sphere marker
"""

# Basic imports
from numpy.core.numerictypes import ScalarType
import rospy
import sys
import math
# Module for broadcasting static transformations
"""
    Static transformations are published on /tf_static. Just like how transform_broadcaster.h
    is for /tf, static_transform_broadcaster is for /tf_static. Static transforms are meant to
    be published only once

    Docs (Python API): https://docs.ros.org/en/melodic/api/tf2_ros/html/python/
"""
from tf2_ros import StaticTransformBroadcaster
# Transform geometry message
from geometry_msgs.msg import TransformStamped
# Import transformations libraries
from tf import transformations
# Marker class (for visualization markers)
from visualization_msgs.msg import Marker


# Main function
def main():
    # Initialize node
    rospy.init_node("t1_py_markers_publisher", argv=sys.argv)

    # Create transform broadcaster
    stf_broadcaster = StaticTransformBroadcaster()
    stf_msg = TransformStamped()    # Static transformation message for {f2} -> {f3} ({f3} in {f2})
    stf_msg.header.seq = 1
    stf_msg.header.stamp = rospy.Time.now()
    stf_msg.header.frame_id = "f2"  # From {f2}
    stf_msg.child_frame_id = "f3"   # To {f3}
    # {f3} is at [0, 0, 1] in {f2}
    stf_msg.transform.translation.x = 0
    stf_msg.transform.translation.y = 0
    stf_msg.transform.translation.z = 1
    # Rotate along Z (yaw) by -45 deg
    f2_f3_quat = transformations.quaternion_about_axis(math.radians(-45), (0, 0, 1))
    stf_msg.transform.rotation.x = f2_f3_quat[0]
    stf_msg.transform.rotation.y = f2_f3_quat[1]
    stf_msg.transform.rotation.z = f2_f3_quat[2]
    stf_msg.transform.rotation.w = f2_f3_quat[3]
    # Publish a static transform
    stf_broadcaster.sendTransform(stf_msg)

    # Create a publisher for the markers (topic name must match that in Rviz)
    marker_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    # Sphere marker
    sphere_marker = Marker()
    sphere_marker.header.seq = 0
    sphere_marker.header.frame_id = "f3"    # In {f3}
    sphere_marker.ns = "frames" # Namespace for flitering
    sphere_marker.id = 10   # A unique ID in the namespace
    sphere_marker.type = Marker.SPHERE  # This is a sphere
    sphere_marker.action = Marker.ADD   # Add this object (same as modify)
    sphere_marker.color.a = 1
    sphere_marker.color.r = 1
    sphere_marker.color.g = 0
    sphere_marker.color.b = 0   # Red color marker (with alpha = 1)
    sphere_marker.pose.orientation.w = 1
    sphere_marker.pose.orientation.x = 0
    sphere_marker.pose.orientation.y = 0
    sphere_marker.pose.orientation.z = 0    # Assign orientation (no rotation)
    sphere_marker.scale.x = 0.1
    sphere_marker.scale.y = 0.1
    sphere_marker.scale.z = 0.1 # 0.1m diameter
    # Sphere marker parameters
    freq = 2.0 * math.pi / 5.0    # 5 second time period
    start_t = rospy.Time.now()  # Starting time
    rate_hdlr = rospy.Rate(30)  # 30 Hz for publishing rate

    # Main loop
    while (not rospy.is_shutdown()):
        curr_time = rospy.Time.now()
        # Update header
        sphere_marker.header.seq += 1
        sphere_marker.header.stamp = curr_time
        dt = (curr_time - start_t).to_sec()
        sphere_marker.pose.position.x = 1.0 * math.cos(dt*freq)
        sphere_marker.pose.position.y = 1.0 * math.sin(dt*freq)
        sphere_marker.pose.position.z = 0
        # Publishing the sphere marker
        marker_publisher.publish(sphere_marker)
        # Sleep
        rate_hdlr.sleep()


# Entry point
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("Fatal error occurred: {}".format(exc))
