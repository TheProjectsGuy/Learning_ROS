"""
    The purpose of this file is as follows:
        1. Read "/gazebo/link_states"
        2. Publish equivalent transformation on "/tf"
"""

# Import basic modules
import rospy
import sys
import traceback
# We'll need to lookup transformation
"""
    Transform lookup is done using tf2 here (more recommended method)
    
    Ref: https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
"""
import tf2_ros
# For spatial transformations
"""
    Mathematical computations

    Ref: https://numpy.org/doc/stable/
"""
import numpy as np
"""
    Module `scipy.spatial.transform.Rotation` allows dealing
    with 3D rotations (representation and transformation) [1]

    [1]: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html#scipy.spatial.transform.Rotation
"""
from scipy.spatial.transform import Rotation as R
# Messages for topics
"""
    Link states message from gazebo. The link states are 
    published by the Gazebo simulator

    Src: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_msgs/msg/LinkStates.msg
    Info: Run `rosmsg info gazebo_msgs/LinkStates`
"""
from gazebo_msgs.msg import LinkStates
"""
    Joint states messages for joint_state_publisher. This
    will later be translated to /tf

    Src: https://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html
    Info: Run `rosmsg info sensor_msgs/JointState`
"""
from sensor_msgs.msg import JointState
# Utilities
"""
    For synchronizing data, as subscriber calls happen through
    multi-threading, it is safe to use Locks.

    Ref: https://docs.python.org/3/library/threading.html#threading.Lock
"""
from threading import Lock


# Transformations class (wrapper for Rotations and Translation)
class Transform:
    """
        Transformation class for handling both rotation and translation

        Constructor
        -------

        Creates and returns a Transform object
        Parameters:
        - pos: list     (default: [0.0, 0.0, 0.0])
            [Px, Py, Pz] position for transformation
        - quat: list    (default: [0.0, 0.0, 0.0, 1.0])
            [Qx, Qy, Qz, Qw] quaternion for transformation
        Returns:
        - self: Transform
            Transform object
    """
    # Constructor
    def __init__(self, pos = [0.0, 0.0, 0.0], quat = [0.0, 0.0, 0.0, 1.0]):
        self.pos = np.array(pos)    # Position
        self.rot = R.from_quat(quat)   # Rotation

    # As a numpy matrix
    def as_matrix(self):
        """
            Return the Transform object as a 4 by 4 homogenized
            transformation matrix.

            Returns:
            - mat: np.ndarray   (size: 4,4; type: np.float64)
                4 x 4 Homogeneous Transformation matrix
        """
        rot_mat = self.rot.as_matrix()
        mat = np.vstack((
            np.hstack((
                rot_mat, self.pos.reshape(3, 1)
            )),
            np.array([0.0, 0.0, 0.0, 1.0])
        ))
        return mat
    
    # Multiply two transformations
    def __mul__(self, other):
        mat_left = self.as_matrix()
        mat_right = other.as_matrix()
        mat_mul = np.matmul(mat_left, mat_right)
        # Get result
        pos = mat_mul[0:3, 3].tolist()
        quat = R.from_matrix(mat_mul[0:3, 0:3]).as_quat().tolist()
        return Transform(pos=pos, quat=quat)
    
    # Inverse matrix
    def inv(self):
        """
            Returns the inverse transformation object

            Returns:
            - itf: Transform
                A homogeneous transform that is the inverse of `self`
        """
        rot = self.as_matrix()[0:3, 0:3]
        pos = self.as_matrix()[0:3, 3].reshape(3, 1)
        inv_rot = rot.transpose()
        inv_pos = -np.matmul(inv_rot, pos)
        # Inverse transformation
        itf_pos = inv_pos.tolist()
        itf_quat = R.from_matrix(inv_rot).as_quat().tolist()
        itf = Transform(pos=itf_pos, quat=itf_quat)
        return itf


# Class for translating links
class Translator:
    """
        Class for performing the following tasks
        - Subscribe messages of LinkStates and publish messages of
            type JointState
        - Reads /tf for "odom" to "body"

        Constructor
        -------

        Returns a Translator object

        Parameters:
        - pub_js: str
            Topic name for publishing JointState messages, after
            parsing LinkStates message
        - trate: float
            Translation rate (for publishing JointState)
        - subs_ls: str
            Topic name for subscribing to LinkStates messages, which
            will be parsed
        - jmap: dict    (default: None)
            Joint mapping: name in Gazebo (keys) to name in URDF
            (values)
        - jzeros: dict  (default: None)
            Joint zero transformations: Joint name in URDF mapped
            to Transform for the Zero pose 
    """
    # Constructor
    def __init__(self, pub_js, trate, subs_ls, jmap = None, jzeros = None):
        self.js_publisher = rospy.Publisher(pub_js, JointState, 
            queue_size=5)  # Joint state publisher
        # Lookup Transformations
        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)
        self.translation_rate = trate
        self.jmap = jmap    # Joint map
        self.jzeros = jzeros    # Joint zeros
        # Common data across threads (subscribers)
        self.wheel_tfs = None  # Transform objects for each wheel in global frame (dict like jzeros but real time position)
        self.wheel_tfs_lock = Lock()    # For wheel_tfs
        self.ls_subs = rospy.Subscriber(subs_ls, LinkStates,
            callback=self.update_wheel_wrapper(), queue_size=10)  # Link states subscriber
    
    # Update wheel tfs
    def update_wheel_wrapper(self):
        """
            A wrapper for returning the callback to update the wheel transforms

            Returns:
            - update_wheel(msg: LinkStates)
                The callback function for the subscriber
        """
        def update_wheel(msg: LinkStates):
            # Get Transforms
            updated_wheel_tfs = dict()
            for (i, gz_name) in enumerate(msg.name):
                if gz_name in self.jmap:    # Update the joint pose
                    jname = self.jmap[gz_name]  # Name in URDF
                    pos = [msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z]  # Position
                    quat = [msg.pose[i].orientation.x, msg.pose[i].orientation.y,
                        msg.pose[i].orientation.z, msg.pose[i].orientation.w]   # Quaternion
                    updated_wheel_tfs[jname] = Transform(pos=pos, quat=quat)
            self.wheel_tfs_lock.acquire()   # Get lock
            self.wheel_tfs = updated_wheel_tfs
            self.wheel_tfs_lock.release()   # Updated, release lock
        return update_wheel
    
    # Runner for the class
    def run_forever(self):
        # Wait for the first message to update the wheel_tfs to not None
        rospy.loginfo("Waiting for first wheel transform update")
        while self.wheel_tfs is None:
            pass
        rospy.loginfo("First wheel transforms received, starting runner")
        joint_state_msg = JointState()
        joint_state_msg.header.seq = 0
        joint_state_msg.name.clear()
        joint_state_msg.position.clear()
        joint_state_msg.velocity.clear()
        joint_state_msg.effort.clear()
        # Rate handler
        rate_hdlr = rospy.Rate(self.translation_rate)
        # Run till ROS is okay
        while not rospy.is_shutdown():
            # Get transform for body (in /odom)
            try:
                # Dummy and body are attached
                tf_body = self.tf_buffer.lookup_transform("odom", "dummy", rospy.Time())
                pos = [
                    tf_body.transform.translation.x, 
                    tf_body.transform.translation.y, 
                    tf_body.transform.translation.z,
                ]
                quat = [
                    tf_body.transform.rotation.x,
                    tf_body.transform.rotation.y,
                    tf_body.transform.rotation.z,
                    tf_body.transform.rotation.w,
                ]
                tf_odom_body = Transform(pos=pos, quat=quat)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as exc:
                rospy.logerr("Transform exception from odom to base: {}".format(exc))
                continue
            # Get the expected transform (wheels in odom)
            tf_exp_wheels_odom = dict()
            for wheel in self.jzeros:
                tf_exp_wheels_odom[wheel] = tf_odom_body * self.jzeros[wheel]
            # Get the real wheel transfor
            self.wheel_tfs_lock.acquire()
            tf_wheels_odom = self.wheel_tfs # Real wheel transform (wheels in odom)
            self.wheel_tfs_lock.release()
            # Get the joint angles
            joint_state_msg.name.clear()
            joint_state_msg.position.clear()
            for urdf_jname in tf_wheels_odom:
                exp_tf = tf_exp_wheels_odom[urdf_jname]
                real_tf = tf_wheels_odom[urdf_jname]
                # exp * change = real -> left mul by exp.inv()
                change_tf = exp_tf.inv() * real_tf
                # Get the rotation about Z
                yaw = change_tf.rot.as_euler("XYZ")[2]
                # Attach to the message
                joint_state_msg.name.append(urdf_jname)
                joint_state_msg.position.append(yaw)
            # Publish the message
            joint_state_msg.header.seq += 1
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.header.frame_id = "body"
            # Joint state publisher
            self.js_publisher.publish(joint_state_msg)
            rate_hdlr.sleep()   # Delay


# Load parameters
def load_params():
    """
        Loads parameters from ROS parameter server.

        Returns:
        - ret_dict: dict
            A dictionary with the following keys
            - "pub_topic": str
                Topic name for publishing messages of type
                sensor_msgs/JointState for translating gazebo
                messages on '/gazebo/link_states'
            - "subs_link_topic": str
                Topic name on which `gazebo_msgs/LinkStates`
                messages are being transmitted. Usually the topic
                /gazebo/link_states
            - "joint_map": dict
                A dictionary where keys are link names (str) in
                gazebo_msgs/LinkStates and values (str) are
                corresponding joint names (for JointState)
            - "tf_zjoints: dict
                Transform objects for each joint. Joint names
                are keys and Transform object are values
            - "translation_rate": float
                The translation rate for the translator (in hz)
    """
    ret_dict = dict()   # Dictionary to return
    # Load topic name (for publishing)
    pub_topic = "fwb_gz_msgs"
    pname = "{}/{}".format(rospy.get_name(), "source_list")
    pub_topic = str(list(rospy.get_param(pname))[0])
    if not pub_topic.startswith("/"):   # Add namespace
        pub_topic="{ns}{pt}".format(ns=rospy.get_namespace(), pt=pub_topic)
    rospy.loginfo("Publishing to topic (JointState messages): '{}'".format(pub_topic))
    ret_dict["pub_topic"] = pub_topic
    # Read topic (for gazebo link states)
    subs_link_topic = "/gazebo/link_states"
    rospy.loginfo("Subscribing to topic (gazebo link states): '{}'".format(subs_link_topic))
    ret_dict["subs_link_topic"] = subs_link_topic
    # Joint mappings and zero Transform objects
    joint_maps = dict() # Key: Gazebo link -> Value: URDF
    zero_tfs = dict()   # Key: URDF Joint name -> Value: Transform for zero
    pname = "{}/{}".format(rospy.get_name(), "gz_translation")
    gz_translation = dict(rospy.get_param(pname))
    translation_rate = gz_translation["rate"]   # Translation rate
    gz_tmap = gz_translation["map"]
    for k in gz_tmap:
        gz_fname = gz_tmap[k]["gz_name"]    # Link name in gazebo
        joint_maps[gz_fname] = k    # Joint name in Gazebo (Key) => URDF (Value)
        zero_pose = gz_tmap[k]["zero"]  # Zero pose for the joint URDF
        tf_zero = Transform(pos=zero_pose[0:3], quat=zero_pose[3:7])
        zero_tfs[k] = tf_zero   # Key: URDF Joint name, Value: zero Transform for the joint
        # Gazebo frame -> Joint name in Gazebo; zero at: Transform.as_matrix()
        rospy.loginfo("'{}' -> '{}'; zero at: '{}'".format(gz_fname, k, tf_zero.as_matrix()))
    ret_dict["joint_map"] = joint_maps  # Joint names for name in Gazebo
    ret_dict["tf_zjoints"] = zero_tfs    # Zero transforms for Joint names
    ret_dict["translation_rate"] = translation_rate
    # Return dictionary
    return ret_dict


# Main function
def main():
    # Initialize the node
    rospy.init_node("robot_joint_translator", argv=sys.argv)
    # Load parameters
    parameters = load_params()
    # Create Translator object
    gz_urdf_translator = Translator(
        parameters["pub_topic"], parameters["translation_rate"],
        parameters["subs_link_topic"], jmap=parameters["joint_map"],
        jzeros=parameters["tf_zjoints"])    # Translator
    gz_urdf_translator.run_forever()


# Entry point
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("A ROS exception occurred (msg: {msg})".format(msg=exc))
        traceback.print_exc()
    except Exception as exc:
        rospy.logfatal("A non-ROS exception occurred (msg: {msg})".format(msg=exc))
        traceback.print_exc()

