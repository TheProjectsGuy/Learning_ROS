"""
    The purpose of this file is as follows:
        1. To be used as a dummy tf publisher (also called transform broadcaster) for Tutorial 1
        2. Create a transform broadcaster that can publish data for `TF` type
"""

# Import basic modules
import rospy
import sys

import tf2_ros
# Import the module for transformations
"""
    Include the wrapper for broadcasting transformations. This will make the direct publisher 
    abstract and the whole process would become easier to deal with.

    Docs (Py API): https://docs.ros.org/en/melodic/api/tf2_ros/html/python/
"""
from tf2_ros import transform_broadcaster
# Import the module for creating quaternions
"""
    The mathematical side of creating and handling quaternion transformations is handled

    Docs (Py API): https://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
"""
from tf import transformations
# Import the module for message type that will be published
"""
    The topic '/tf' has messages of type 'tf2_msgs/TFMessage' which is basically an array of messages
    of type 'geometry_msgs/TransformStamped'. Include that message library.

    Ref (.msg file): https://docs.ros.org/en/api/geometry_msgs/html/msg/TransformStamped.html
"""
from geometry_msgs.msg import TransformStamped
# Math module
import math


# Main function
def main():
    # Initialize the node
    rospy.init_node("t1_py_tf_broadcaster", argv=sys.argv)
    
    # Create a node handler
    rate_handler = rospy.Rate(1)    # 1 Hz

    # An object that will broadcase transformations
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Create transformations
    """
        Transformations: {global} -> {f1} -> {f2}
        {f1} is +2m in X w.r.t. {global}
        {f2} is +1m in X, +1m in Y and 45 deg Z (yaw) rotated w.r.t. {f1}
    """
    trans1 = TransformStamped()
    trans1.header.seq = 0   # Starting sequence
    trans1.header.frame_id = "global"   # From {global}
    trans1.child_frame_id = "f1"    # To {f1}
    trans1.transform.translation.x = 1  # {global} to {f1}: +1m in X
    trans1.transform.rotation.w = 1 # This is required to normalize the quaternion
    trans2 = TransformStamped()
    trans2.header.seq = 0
    trans2.header.frame_id = "f1"   # From {f1}
    trans2.child_frame_id = "f2"    # To {f2}
    trans2.transform.translation.x = 1  # +1m in X
    trans2.transform.translation.y = 1  # +1m in Y
    quat = transformations.quaternion_about_axis(math.radians(45), (0, 0, 1))
    # Assign quaternion to the message
    trans2.transform.rotation.x = quat[0]
    trans2.transform.rotation.y = quat[1]
    trans2.transform.rotation.z = quat[2]
    trans2.transform.rotation.w = quat[3]

    # Publish messages (in a continuous loop)
    while not rospy.is_shutdown():
        # Update items
        trans1.header.seq += 1
        trans1.header.stamp = rospy.Time.now()
        trans2.header.seq += 1
        trans2.header.stamp = rospy.Time.now()
        # Send the transformations
        tf_broadcaster.sendTransform(trans1)
        tf_broadcaster.sendTransform(trans2)
        # Handle rate
        rate_handler.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("Fatal error occurred: {err}".format(err=exc))
    pass
