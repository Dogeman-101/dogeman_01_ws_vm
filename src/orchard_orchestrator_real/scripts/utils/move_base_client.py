#!/usr/bin/env python3
"""
move_base_client.py — move_base action 客户端封装

供 Phase 1/2/3 复用。单车默认连 `move_base`，多车传 `/robotN/move_base`。

禁止进入本模块：分配算法 / yaml 加载 / 业务逻辑。
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


# actionlib GoalStatus 的可读映射（actionlib_msgs/GoalStatus.msg）
STATE_NAMES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}

# 终态集合：循环轮询场景 A 时判断是否该退出
TERMINAL_STATES = {3, 4, 5, 6, 7, 8, 9}


def _yaw_to_quaternion(yaw):
    """偏航角（弧度）→ geometry_msgs/Quaternion。"""
    q = quaternion_from_euler(0.0, 0.0, float(yaw))
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class MoveBaseClient(object):
    """
    SimpleActionClient 的薄封装。

    构造后建议先 `wait_server(timeout)` 再发 goal；send_goal 是非阻塞，
    场景 A 用 send_goal + 轮询 get_state；场景 B 用 go_to（阻塞）。
    """

    def __init__(self, action_name="move_base"):
        self.action_name = action_name
        self.client = actionlib.SimpleActionClient(action_name, MoveBaseAction)

    # ------------------------------------------------------------ 连接
    def wait_server(self, timeout=10.0):
        """
        等 action server 上线。

        返回 True 表示就绪；False 表示超时，调用者应自行处理（打印并退出）。
        """
        rospy.loginfo("[mbc] 等待 action server '%s' (<=%ds) ...",
                      self.action_name, int(timeout))
        ok = self.client.wait_for_server(rospy.Duration(timeout))
        if ok:
            rospy.loginfo("[mbc] action server '%s' 已连接", self.action_name)
        else:
            rospy.logerr("[mbc] action server '%s' 超时未响应", self.action_name)
        return ok

    # ------------------------------------------------------------ 发 goal
    def _make_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = _yaw_to_quaternion(yaw)
        return goal

    def send_goal(self, x, y, yaw):
        """非阻塞：立即返回，用 get_state / cancel 控制。"""
        goal = self._make_goal(x, y, yaw)
        rospy.loginfo("[mbc] send_goal  x=%.3f y=%.3f yaw=%.3f", x, y, yaw)
        self.client.send_goal(goal)

    # ------------------------------------------------------------ 状态
    def get_state(self):
        return self.client.get_state()

    def get_state_name(self):
        return STATE_NAMES.get(self.get_state(), "UNKNOWN")

    # ------------------------------------------------------------ 阻塞
    def go_to(self, x, y, yaw, timeout=60.0):
        """
        阻塞等待目标完成。

        返回：(success: bool, state: int, state_name: str)
          success=True 仅当 state == SUCCEEDED(3)
          state=-1 表示超时被 cancel（actionlib 不会再更新状态）
        """
        self.send_goal(x, y, yaw)
        finished = self.client.wait_for_result(rospy.Duration(timeout))
        if not finished:
            rospy.logwarn("[mbc] goal 超时 %.1fs，取消", timeout)
            self.client.cancel_goal()
            return False, -1, "TIMEOUT"
        state = self.client.get_state()
        name = STATE_NAMES.get(state, "UNKNOWN")
        return state == 3, state, name

    def cancel(self):
        try:
            self.client.cancel_all_goals()
        except Exception as e:
            rospy.logwarn("[mbc] cancel_all_goals 异常：%s", e)
