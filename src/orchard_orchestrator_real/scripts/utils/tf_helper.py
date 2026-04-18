#!/usr/bin/env python3
"""
tf_helper.py — TF 查询工具

单车默认查 `map → base_footprint`；多车通过构造参数传 `robotN/base_footprint`。

禁止进入本模块：分配算法 / yaml 加载 / 业务逻辑。
"""

import math

import rospy
import tf
from tf.transformations import euler_from_quaternion


def _classify_bearing(delta_heading_deg):
    """
    把"目标相对车头的角度差"分类为中文方向词。

    Δ>0 → 左侧；Δ<0 → 右侧。
    |Δ| ≤ 45°  → 前方
    |Δ| ≥ 135° → 后方
    其余 → 左前/右前
    """
    a = abs(delta_heading_deg)
    if a <= 45.0:
        return "前方"
    if a >= 135.0:
        return "后方"
    return "左前方" if delta_heading_deg > 0 else "右前方"


class TFHelper(object):
    """
    tf.TransformListener 的薄封装。

    失败策略：查询失败返回 None，不抛异常；由调用方决定重试或退出，
    避免调度节点在 TF 偶发 timeout 时整个挂掉。
    """

    def __init__(self, target="base_footprint", source="map"):
        self.listener = tf.TransformListener()
        self.target = target
        self.source = source

    # ------------------------------------------------------------ 就绪
    def wait_ready(self, timeout=10.0):
        """
        轮询 waitForTransform。

        返回 True 表示 map→base_footprint 已可查；False 表示超时。
        """
        rospy.loginfo("[tfh] 等待 TF '%s → %s' (<=%ds) ...",
                      self.source, self.target, int(timeout))
        try:
            self.listener.waitForTransform(
                self.source, self.target, rospy.Time(0), rospy.Duration(timeout)
            )
            rospy.loginfo("[tfh] TF '%s → %s' 已就绪", self.source, self.target)
            return True
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr("[tfh] TF '%s → %s' 等待失败：%s",
                         self.source, self.target, e)
            return False

    # ------------------------------------------------------------ 查位
    def get_pose(self):
        """
        返回 (x, y, yaw)；失败返回 None。
        yaw 为弧度，范围 (-π, π]。
        """
        try:
            trans, rot = self.listener.lookupTransform(
                self.source, self.target, rospy.Time(0)
            )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "[tfh] lookupTransform 失败：%s", e)
            return None
        _, _, yaw = euler_from_quaternion(rot)
        return float(trans[0]), float(trans[1]), float(yaw)

    # ------------------------------------------------------------ 打印
    def print_pose(self, prefix=""):
        """打印当前位姿（弧度+度数），查不到就打 'N/A'。"""
        pose = self.get_pose()
        if pose is None:
            rospy.loginfo("%s 位姿: N/A（TF 查询失败）", prefix)
            return
        x, y, yaw = pose
        rospy.loginfo("%s 位姿: x=%.3f y=%.3f yaw=%.3f rad (%.1f°)",
                      prefix, x, y, yaw, math.degrees(yaw))

    def print_bearing_to(self, gx, gy, prefix=""):
        """
        打印到目标 (gx, gy) 的距离、绝对方位、相对车头的角度差、方向分类。

        定义：
          absolute_bearing = atan2(gy - y, gx - x)              （全局系 rad）
          delta_heading    = normalize(absolute_bearing - yaw)  （车身系 rad）
        """
        pose = self.get_pose()
        if pose is None:
            rospy.loginfo("%s 到目标方位: N/A", prefix)
            return
        x, y, yaw = pose
        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)
        absolute_bearing = math.atan2(dy, dx)
        delta = absolute_bearing - yaw
        delta = math.atan2(math.sin(delta), math.cos(delta))  # 归一化到 [-π, π]
        delta_deg = math.degrees(delta)
        direction = _classify_bearing(delta_deg)
        rospy.loginfo(
            "%s 目标 (%.3f, %.3f) 距离=%.3fm  绝对方位=%.1f°  "
            "需转=%+.1f°  相对车身=%s",
            prefix, gx, gy, distance,
            math.degrees(absolute_bearing), delta_deg, direction,
        )
