#!/usr/bin/env python3
"""两轮 Hungarian 公平性回归测试：wait_rounds 惩罚后 3 个 picker 全被服务。"""
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "algorithms")))

from cost_matrix import build_cost_matrix
from hungarian import assign_tasks


def _step(wait_rounds, assigned):
    for pid in wait_rounds:
        wait_rounds[pid] = 0 if pid in assigned else wait_rounds[pid] + 1


class TestFairness(unittest.TestCase):
    TRANSPORTERS = [(-1.3, -0.5), (-1.3,  0.5)]
    PICKERS_POS  = [( 1.5, -0.5), ( 1.5,  0.5), ( 1.5, 1.25)]
    PICKER_IDS   = ["robot1", "robot2", "robot3"]
    ALPHA        = 2.5

    def _assign(self, wait_rounds):
        C = build_cost_matrix(self.TRANSPORTERS, self.PICKERS_POS,
                              task_ids=self.PICKER_IDS,
                              wait_rounds=wait_rounds,
                              alpha=self.ALPHA)
        pairs = assign_tasks(C)
        return {self.PICKER_IDS[t] for _, t in pairs}

    def test_without_fairness_picker3_never_served(self):
        r1 = self._assign(None)
        r2 = self._assign(None)
        self.assertEqual(r1, r2)
        self.assertNotIn("robot3", r1 | r2,
                         "几何假设失效，picker3 应始终被跳")

    def test_with_fairness_all_pickers_served_within_two_rounds(self):
        wait_rounds = {pid: 0 for pid in self.PICKER_IDS}
        r1 = self._assign(wait_rounds)
        _step(wait_rounds, r1)
        r2 = self._assign(wait_rounds)
        _step(wait_rounds, r2)
        self.assertEqual(r1 | r2, set(self.PICKER_IDS),
                         "两轮合起来应覆盖 3 个 picker: r1={}, r2={}".format(r1, r2))


if __name__ == "__main__":
    unittest.main(verbosity=2)
