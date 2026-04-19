#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TF 诊断监控脚本 —— 纯观察者。

跑在 VM 上，在 move_base 导航期间持续采样 TF、动捕、/odom、/cmd_vel、
/move_base/status，事后写一份可读的日志文件并打印摘要。

本脚本：
  - 只订阅，不发布；不广播 TF。
  - 单文件，不依赖 scripts/algorithms 或 scripts/utils。
  - Ctrl-C 或 ~max_duration 到期后触发 on_shutdown，打印摘要并关闭日志。

用法：
  roslaunch orchard_orchestrator_real tf_monitor.launch
或：
  rosrun orchard_orchestrator_real tf_monitor.py _max_duration:=60.0 _log_dir:=/tmp
"""

import math
import os
import datetime

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray


STATE_NAMES = {
    0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED',
    4: 'ABORTED', 5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING',
    8: 'RECALLED', 9: 'LOST',
}


def _yaw_from_quat(q):
    """从四元数（有 x/y/z/w 属性）抽 yaw，返回弧度。"""
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def _wrap_angle(a):
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


class TFMonitor(object):
    def __init__(self):
        rospy.init_node('tf_monitor')

        # ---- 参数 ----
        self.mocap_topic = rospy.get_param('~mocap_topic', '/mocap_node/Robot_1/pose')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.mb_status_topic = rospy.get_param('~move_base_status_topic', '/move_base/status')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom_combined')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.tick_rate = float(rospy.get_param('~tick_rate', 1.0))
        self.jump_pos = float(rospy.get_param('~jump_pos_threshold', 1.0))
        self.jump_yaw = math.radians(float(rospy.get_param('~jump_yaw_threshold_deg', 90.0)))
        self.dropout = float(rospy.get_param('~dropout_threshold', 0.5))
        self.log_dir = os.path.expanduser(rospy.get_param('~log_dir', '~'))
        self.max_duration = float(rospy.get_param('~max_duration', 0.0))

        # 要采样的三条 TF 链
        self.tf_chains = [
            (self.map_frame, self.odom_frame),
            (self.odom_frame, self.base_frame),
            (self.map_frame, self.base_frame),
        ]

        # ---- 日志文件 ----
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        if not os.path.isdir(self.log_dir):
            os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, 'tf_diagnosis_{}.log'.format(stamp))
        self.log_fh = open(self.log_path, 'w', buffering=1)  # 行缓冲，Ctrl-C 时不丢尾部

        # ---- 状态 ----
        self.start_time = rospy.Time.now()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # mocap
        self.mocap_count = 0
        self.mocap_jumps = 0
        self.mocap_dropouts = 0
        self.prev_mocap = None   # (x, y, yaw, wall_t)
        # odom
        self.odom_count = 0
        self.odom_dropouts = 0
        self.prev_odom_t = None
        # cmd_vel
        self.cmdvel_count = 0
        self.cmdvel_reverse = 0
        # move_base
        self.last_mb_state = None
        self.mb_changes = []     # [(elapsed_s, prev_code, prev_name, new_code, new_name)]
        # TF 每条链：{'ok':n, 'fail':n, 'ages':[...]}
        self.tf_stats = {
            chain: {'ok': 0, 'fail': 0, 'ages': []}
            for chain in self.tf_chains
        }
        # 心跳
        self.last_heartbeat_t = 0.0

        # ---- 订阅 + 定时器 ----
        rospy.Subscriber(self.mocap_topic, PoseStamped, self._mocap_cb, queue_size=50)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=50)
        rospy.Subscriber(self.mb_status_topic, GoalStatusArray, self._mb_status_cb, queue_size=10)

        tick_period = max(0.05, 1.0 / max(self.tick_rate, 1e-3))
        rospy.Timer(rospy.Duration(tick_period), self._tick)

        rospy.on_shutdown(self._on_shutdown)

        if self.max_duration > 0.0:
            rospy.Timer(rospy.Duration(self.max_duration),
                        self._max_duration_cb, oneshot=True)

        self._log('START', 'mocap={} odom={} cmd_vel={} mb_status={} '
                           'tf_chain={}->{}->{}  log={}'.format(
                      self.mocap_topic, self.odom_topic,
                      self.cmd_vel_topic, self.mb_status_topic,
                      self.map_frame, self.odom_frame, self.base_frame,
                      self.log_path))
        rospy.loginfo('[tf_monitor] 日志文件: %s', self.log_path)
        rospy.loginfo('[tf_monitor] 按 Ctrl-C 结束并输出摘要')

    # ------------------------------------------------------------------ helpers
    def _elapsed(self):
        return (rospy.Time.now() - self.start_time).to_sec()

    def _log(self, tag, msg):
        line = '[t={:7.2f}s] {:<12s}: {}\n'.format(self._elapsed(), tag, msg)
        try:
            self.log_fh.write(line)
        except (ValueError, IOError):
            pass  # 文件已关

    # ------------------------------------------------------------------ callbacks
    def _mocap_cb(self, msg):
        self.mocap_count += 1
        now = rospy.Time.now()
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = _yaw_from_quat(msg.pose.orientation)

        if self.prev_mocap is not None:
            px, py, pyaw, pt = self.prev_mocap
            gap = (now - pt).to_sec()
            if gap > self.dropout:
                self.mocap_dropouts += 1
                self._log('MOCAP_DROPOUT',
                          'gap={:.3f}s (>{:.2f}s)'.format(gap, self.dropout))
            dpos = math.hypot(x - px, y - py)
            dyaw = abs(_wrap_angle(yaw - pyaw))
            if dpos > self.jump_pos or dyaw > self.jump_yaw:
                self.mocap_jumps += 1
                self._log('MOCAP_JUMP',
                          'dpos={:.3f}m (thr={:.2f}) dyaw={:.1f}deg (thr={:.1f}) '
                          '(prev=({:.3f},{:.3f},{:.1f}deg) cur=({:.3f},{:.3f},{:.1f}deg))'.format(
                              dpos, self.jump_pos,
                              math.degrees(dyaw), math.degrees(self.jump_yaw),
                              px, py, math.degrees(pyaw),
                              x, y, math.degrees(yaw)))
        self.prev_mocap = (x, y, yaw, now)

    def _odom_cb(self, msg):
        self.odom_count += 1
        now = rospy.Time.now()
        if self.prev_odom_t is not None:
            gap = (now - self.prev_odom_t).to_sec()
            if gap > self.dropout:
                self.odom_dropouts += 1
                self._log('ODOM_DROPOUT',
                          'gap={:.3f}s (>{:.2f}s)'.format(gap, self.dropout))
        self.prev_odom_t = now

    def _cmd_vel_cb(self, msg):
        self.cmdvel_count += 1
        if msg.linear.x < -1e-3:
            self.cmdvel_reverse += 1
            self._log('CMDVEL',
                      'lin_x={:+.3f} lin_y={:+.3f} ang_z={:+.3f} REVERSE'.format(
                          msg.linear.x, msg.linear.y, msg.angular.z))

    def _mb_status_cb(self, msg):
        if not msg.status_list:
            return
        code = msg.status_list[-1].status
        if code != self.last_mb_state:
            prev_code = self.last_mb_state
            prev_name = STATE_NAMES.get(prev_code, 'NONE') if prev_code is not None else 'NONE'
            new_name = STATE_NAMES.get(code, 'UNKNOWN')
            self.mb_changes.append((self._elapsed(), prev_code, prev_name, code, new_name))
            self._log('MBSTATE', '{} -> {} (code={})'.format(prev_name, new_name, code))
            self.last_mb_state = code

    # ------------------------------------------------------------------ tick
    def _tick(self, _event):
        for chain in self.tf_chains:
            self._sample_tf(*chain)

        # 每 5 秒写一行心跳（累计计数），便于排查"那一秒到底有没有数据"
        t = self._elapsed()
        if t - self.last_heartbeat_t >= 5.0:
            self._log('HEARTBEAT',
                      'mocap_n={} odom_n={} cmdvel_n={} (reverse {}) mb_state={}'.format(
                          self.mocap_count, self.odom_count,
                          self.cmdvel_count, self.cmdvel_reverse,
                          STATE_NAMES.get(self.last_mb_state, 'NONE')))
            self.last_heartbeat_t = t

    def _sample_tf(self, src, tgt):
        stats = self.tf_stats[(src, tgt)]
        try:
            t = self.tf_buffer.lookup_transform(
                src, tgt, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            stats['fail'] += 1
            self._log('TF_FAIL', '{}->{} : {}'.format(src, tgt, e))
            return

        age = (rospy.Time.now() - t.header.stamp).to_sec()
        stats['ok'] += 1
        stats['ages'].append(age)
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = _yaw_from_quat(t.transform.rotation)
        self._log('TF_OK',
                  '{}->{} age={:.3f}s x={:+.3f} y={:+.3f} yaw={:+.1f}deg'.format(
                      src, tgt, age, x, y, math.degrees(yaw)))

    # ------------------------------------------------------------------ shutdown
    def _max_duration_cb(self, _event):
        rospy.loginfo('[tf_monitor] max_duration %.1fs 到期，自动退出', self.max_duration)
        rospy.signal_shutdown('max_duration')

    def _on_shutdown(self):
        elapsed = self._elapsed()
        lines = []
        lines.append('')
        lines.append('===== TF 诊断摘要 =====')
        lines.append('运行时长: {:.1f}s'.format(elapsed))

        # mocap
        lines.append('动捕数据: 共 {} 帧, 断流 {} 次, 异常跳变 {} 次'.format(
            self.mocap_count, self.mocap_dropouts, self.mocap_jumps))
        # odom
        lines.append('/odom: 共 {} 帧, 断流 {} 次'.format(
            self.odom_count, self.odom_dropouts))
        # tf
        for (src, tgt) in self.tf_chains:
            s = self.tf_stats[(src, tgt)]
            total = s['ok'] + s['fail']
            if s['ages']:
                max_age = max(s['ages'])
                avg_age = sum(s['ages']) / len(s['ages'])
                age_str = 'max age {:.3f}s, avg {:.3f}s'.format(max_age, avg_age)
            else:
                age_str = 'no successful samples'
            lines.append('TF {}->{}: 成功 {}/{}, 失败 {}/{} ({})'.format(
                src, tgt, s['ok'], total, s['fail'], total, age_str))
        # cmd_vel
        if self.cmdvel_count > 0:
            ratio = 100.0 * self.cmdvel_reverse / self.cmdvel_count
            lines.append('cmd_vel: 共 {} 条, 倒车 {} 次 ({:.1f}%)'.format(
                self.cmdvel_count, self.cmdvel_reverse, ratio))
        else:
            lines.append('cmd_vel: 共 0 条 (未收到 /cmd_vel; 确认 W4A bringup 已启动)')
        # move_base
        if self.mb_changes:
            lines.append('move_base 状态变化:')
            for (t_s, pc, pn, nc, nn) in self.mb_changes:
                lines.append('  t={:7.2f}s: {:9s} -> {:9s} (code {}->{})'.format(
                    t_s, pn, nn, pc if pc is not None else -1, nc))
        else:
            lines.append('move_base 状态变化: 无 (未收到 /move_base/status)')
        lines.append('日志: {}'.format(self.log_path))
        lines.append('========================')
        block = '\n'.join(lines)

        # stdout
        print(block)
        # 文件末尾
        try:
            self.log_fh.write(block + '\n')
            self.log_fh.flush()
            self.log_fh.close()
        except (ValueError, IOError):
            pass


def main():
    TFMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
