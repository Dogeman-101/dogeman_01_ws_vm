#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_controller.py — 动捕闭环简单控制器（单车单点）

本模块不是 ROS 节点入口，由节点脚本实例化：

    ctrl = SimpleController(mocap_topic, cmd_vel_topic,
                            goal_x, goal_y, goal_yaw)
    rospy.on_shutdown(ctrl.stop)
    while not rospy.is_shutdown() and not ctrl.arrived:
        ...  # 节点负责打印/轮询 ctrl 的公开状态

设计取舍：
- 控制环由 rospy.Timer 驱动（非 pose 回调），动捕中断也能发零速。
- 不 import tf / numpy / move_base / actionlib，四元数→yaw 内联算。
- 一个节点可实例化多个 SimpleController，为多车留路。
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist


class SimpleController(object):
    def __init__(self, mocap_topic, cmd_vel_topic,
                 goal_x, goal_y, goal_yaw,
                 robot_name="robot1",
                 kp_linear=0.5, kp_angular=1.0,
                 max_linear=0.25, max_angular=1.0,
                 dist_tolerance=0.15, yaw_tolerance=0.1,
                 mocap_timeout=1.0, control_rate=20.0,
                 differential_mode=True):
        # differential_mode:
        #   True  → 差速模式，_tick 计算后强制 vy=0（不侧移，适配普通两轮差速车）
        #   False → 全向模式，保留 vy（麦轮 / 全向底盘）
        # 默认 True，要切麦轮直接传 False 或实例化后改 self.differential_mode。
        self.robot_name = robot_name
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.goal_yaw = float(goal_yaw)

        self.kp_linear = float(kp_linear)
        self.kp_angular = float(kp_angular)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.dist_tolerance = float(dist_tolerance)
        self.yaw_tolerance = float(yaw_tolerance)
        self.mocap_timeout = float(mocap_timeout)
        self.differential_mode = bool(differential_mode)

        # 公开状态（节点查询）
        self.current_pose = None           # (x, y, yaw) 或 None
        self.last_pose_time = None         # rospy.Time 或 None
        self.arrived = False
        self.state = "WAITING"             # WAITING / MOVING / ROTATING / ARRIVED
        self.distance_to_goal = float("inf")
        self.last_cmd = (0.0, 0.0, 0.0)    # (vx, vy, wz) 最后一次发出

        # 用于降频的告警（避免刷屏）
        self._last_timeout_warn = rospy.Time(0)
        self._last_outlier_warn = rospy.Time(0)

        # 订阅 / 发布
        self._pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._sub = rospy.Subscriber(mocap_topic, PoseStamped, self._pose_cb,
                                     queue_size=1)

        rospy.loginfo("[%s] SimpleController 已启动。mocap=%s cmd_vel=%s "
                      "goal=(%.3f, %.3f, yaw=%.3f rad / %.1f°)",
                      self.robot_name, mocap_topic, cmd_vel_topic,
                      self.goal_x, self.goal_y, self.goal_yaw,
                      math.degrees(self.goal_yaw))

        # 控制环定时器（独立于 pose 到达节奏）
        period = rospy.Duration(1.0 / float(control_rate))
        self._timer = rospy.Timer(period, self._tick)

    # ------------------------------------------------------------------
    #  回调与控制环
    # ------------------------------------------------------------------
    def _pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        # 动捕丢失时 Nokov 有时跳到极大值，简单屏蔽
        if abs(x) > 5000.0 or abs(y) > 5000.0:
            now = rospy.Time.now()
            if (now - self._last_outlier_warn).to_sec() > 1.0:
                rospy.logwarn("[%s] 丢弃异常 pose x=%.1f y=%.1f（动捕丢失？）",
                              self.robot_name, x, y)
                self._last_outlier_warn = now
            return

        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.current_pose = (x, y, yaw)
        self.last_pose_time = rospy.Time.now()

    def _tick(self, _evt):
        if self.arrived:
            self._publish(0.0, 0.0, 0.0)
            return

        if self.current_pose is None or self.last_pose_time is None:
            self.state = "WAITING"
            self._publish(0.0, 0.0, 0.0)
            return

        gap = (rospy.Time.now() - self.last_pose_time).to_sec()
        if gap > self.mocap_timeout:
            if (rospy.Time.now() - self._last_timeout_warn).to_sec() > 1.0:
                rospy.logwarn("[%s] 动捕 %.2fs 未更新，发零速保护。",
                              self.robot_name, gap)
                self._last_timeout_warn = rospy.Time.now()
            self._publish(0.0, 0.0, 0.0)
            return

        cx, cy, cyaw = self.current_pose
        dx = self.goal_x - cx
        dy = self.goal_y - cy
        distance = math.hypot(dx, dy)
        self.distance_to_goal = distance

        yaw_err = self._normalize_angle(self.goal_yaw - cyaw)

        if distance >= self.dist_tolerance:
            # world → body：旋转 −yaw
            vx_body = dx * math.cos(cyaw) + dy * math.sin(cyaw)
            vy_body = -dx * math.sin(cyaw) + dy * math.cos(cyaw)
            speed = min(distance * self.kp_linear, self.max_linear)
            vx = speed * vx_body / distance
            vy = speed * vy_body / distance
            # 差速模式：不允许侧移，车体坐标系的 vy 直接置零
            if self.differential_mode:
                vy = 0.0
            wz = self._clamp(self.kp_angular * yaw_err,
                             -self.max_angular, self.max_angular)
            self.state = "MOVING"
            self._publish(vx, vy, wz)
        elif abs(yaw_err) >= self.yaw_tolerance:
            wz = self._clamp(self.kp_angular * yaw_err,
                             -self.max_angular, self.max_angular)
            self.state = "ROTATING"
            self._publish(0.0, 0.0, wz)
        else:
            self.state = "ARRIVED"
            self.arrived = True
            self._publish(0.0, 0.0, 0.0)
            rospy.loginfo("[%s] 到达目标。", self.robot_name)

    # ------------------------------------------------------------------
    #  公开接口
    # ------------------------------------------------------------------
    def stop(self):
        """节点 shutdown 时调一次；再发一次零速以防 cmd_vel 卡在非零值。"""
        try:
            self._publish(0.0, 0.0, 0.0)
        except Exception:
            pass

    # ------------------------------------------------------------------
    #  内部工具
    # ------------------------------------------------------------------
    def _publish(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._pub.publish(msg)
        self.last_cmd = (vx, vy, wz)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))
