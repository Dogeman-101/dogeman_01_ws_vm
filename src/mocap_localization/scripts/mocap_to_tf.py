#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, TransformStamped


class MocapToTF:

    def __init__(self):
        rospy.init_node('mocap_to_tf')

        self.mocap_topic = rospy.get_param('~mocap_pose_topic', '/mocap_node/Robot_1/pose')
        self.robot_ns = rospy.get_param('~robot_namespace', 'robot1')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)

        self.odom_frame = '{}/odom'.format(self.robot_ns) if self.robot_ns else 'odom'
        self.base_frame = '{}/base_link'.format(self.robot_ns) if self.robot_ns else 'base_link'

        rospy.loginfo('[mocap_to_tf] mocap_pose_topic : %s', self.mocap_topic)
        rospy.loginfo('[mocap_to_tf] robot_namespace  : %s', self.robot_ns)
        rospy.loginfo('[mocap_to_tf] map_frame        : %s', self.map_frame)
        rospy.loginfo('[mocap_to_tf] odom_frame       : %s', self.odom_frame)
        rospy.loginfo('[mocap_to_tf] base_frame       : %s', self.base_frame)
        rospy.loginfo('[mocap_to_tf] publish_rate     : %.1f Hz', self.publish_rate)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.latest_pose = None
        self.latest_stamp = None
        self.published_once = False

        rospy.Subscriber(self.mocap_topic, PoseStamped, self._mocap_cb)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_tf)

    # ------------------------------------------------------------------ callbacks

    def _mocap_cb(self, msg):
        self.latest_pose = msg.pose
        self.latest_stamp = rospy.Time.now()

    def _publish_tf(self, _event):
        if self.latest_pose is None:
            rospy.logwarn_throttle(5.0, '[mocap_to_tf] Waiting for mocap pose on %s', self.mocap_topic)
            return

        age = (rospy.Time.now() - self.latest_stamp).to_sec()
        if age > 1.0:
            rospy.logwarn_throttle(5.0, '[mocap_to_tf] Mocap pose stale (%.1fs), still broadcasting last frame', age)

        try:
            t_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, '[mocap_to_tf] TF lookup %s -> %s failed: %s', self.odom_frame, self.base_frame, e)
            return

        T_map_base = self._pose_to_matrix(self.latest_pose)
        T_odom_base = self._transform_to_matrix(t_odom_base.transform)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = self.map_frame
        ts.child_frame_id = self.odom_frame
        self._matrix_to_transform(T_map_odom, ts.transform)
        self.tf_broadcaster.sendTransform(ts)

        if not self.published_once:
            rospy.loginfo('[mocap_to_tf] TF publishing started: %s -> %s', self.map_frame, self.odom_frame)
            self.published_once = True

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _pose_to_matrix(pose):
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        mat = quaternion_matrix(q)
        mat[0, 3] = pose.position.x
        mat[1, 3] = pose.position.y
        mat[2, 3] = pose.position.z
        return mat

    @staticmethod
    def _transform_to_matrix(tf_msg):
        q = [tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z, tf_msg.rotation.w]
        mat = quaternion_matrix(q)
        mat[0, 3] = tf_msg.translation.x
        mat[1, 3] = tf_msg.translation.y
        mat[2, 3] = tf_msg.translation.z
        return mat

    @staticmethod
    def _matrix_to_transform(mat, tf_msg):
        tf_msg.translation.x = mat[0, 3]
        tf_msg.translation.y = mat[1, 3]
        tf_msg.translation.z = mat[2, 3]
        q = quaternion_from_matrix(mat)
        tf_msg.rotation.x = q[0]
        tf_msg.rotation.y = q[1]
        tf_msg.rotation.z = q[2]
        tf_msg.rotation.w = q[3]


def main():
    node = MocapToTF()
    rospy.spin()


if __name__ == '__main__':
    main()
