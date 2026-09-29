#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race6_comm / race6_mintraj_test.py —— 竞赛主通道端到端验证（controller_msgs/MinTraj）
=====================================================================

【为什么还要单独测这一条】
  race6_comm_test.py 用的是 std_msgs/Float64MultiArray，它只能证明「网络通」。
  但竞赛真正跑的是【规划器广播的 MinTraj】这一条通道，它有两个额外坑：
    1. 话题名必须和实验室 launch 里的 remap 完全一致：
         advanced_param_swarm.xml:50  ~planning/broadcast_traj_send → /broadcast_traj_from_planner
         advanced_param_swarm.xml:51  ~planning/broadcast_traj_recv → /broadcast_traj_to_planner
    2. 类型必须是 controller_msgs/MinTraj（斜杠），而且两端都编译了这个包 ——
       md5sum 随消息过网络（topic_manager.cpp:455），一端没有这个包就必然收不到。
  本脚本就压这条通道，并且【顺带复现 FSM 的时间同步判据】：

      dt = 本机 now - msg.start_time          ← 就是 ego_replan_fsm.cpp:1223 的判据
      |dt| <= 0.25 s → FSM 不告警
      |dt| >  0.25 s → FSM 刷 "Time stamp diff" 告警
      |dt| >  10  s  → FSM 直接丢弃该轨迹（ego_replan_fsm.cpp:1231-1236）

  所以本脚本 PASS 就等于「把 planner 接上来，轨迹广播这一环是通的」。

【报文】话题 /broadcast_traj_from_planner（发）→ Multibotnet → /broadcast_traj_to_planner（收）
  发送方按 traj_id 生成一条【确定性的、结构合法的】3 段 MINCO 轨迹：
      段数 P = 3，内点 P-1 = 2，order = 5
      start_p=(0,0,1) start_v=0 start_a=0
      end_p=(3,0,1)   end_v=0 end_a=0
      inner = (1,0,1), (2,0,1)
      duration = [0.5,0.5,0.5]
      x 方向整体平移 traj_id*0.01（让每帧内容都不同，便于发现「收到旧帧」）
  接收方用同样公式重算一遍，逐字段比对 → 能检出任何字段错位/截断。
  这样构造的好处：万一 planner 也在跑，这条轨迹对 MinJerkOpt 也是合法的，不会把它弄崩。

【用法】
  rosrun race6_comm race6_mintraj_test.py --self-id 0 --peers 1
  roslaunch race6_comm comm_ground_test.launch drone_id:=0

【退出码】0 = PASS，1 = FAIL
"""

from __future__ import print_function

import argparse
import sys
import threading
import time

import rospy

try:
    from controller_msgs.msg import MinTraj
except ImportError as exc:            # noqa: PERF203
    print("=" * 88)
    print("❌ 无法 import controller_msgs.msg.MinTraj：%s" % exc)
    print("   这条通道是竞赛主通道，必须先编译并 source controller_msgs：")
    print("     cd ~/six_ws && catkin_make && source devel/setup.bash")
    print("   检查： rospack find controller_msgs   # 应输出 ~/six_ws/src/planner/controller_msgs")
    print("=" * 88)
    sys.exit(2)

SEND_TOPIC = "/broadcast_traj_from_planner"
RECV_TOPIC = "/broadcast_traj_to_planner"

PIECES = 3                # 段数
DURATIONS = [0.5, 0.5, 0.5]
X0 = 0.0
X1 = 3.0
Z = 1.0
DES_CLEARANCE = 0.15      # 与 advanced_param_swarm.xml:149 的 swarm_clearance 一致


def make_traj(traj_id, drone_id):
    """按 traj_id 生成一条确定性的、结构合法的 3 段 MINCO 轨迹"""
    msg = MinTraj()
    msg.drone_id = drone_id
    msg.traj_id = traj_id
    msg.start_time = rospy.Time.now()      # ★ 这一行就是 FSM 判据里的 msg->start_time
    msg.des_clearance = DES_CLEARANCE
    msg.order = 5

    shift = traj_id * 0.01                  # 每帧内容不同
    msg.start_p = [X0 + shift, 0.0, Z]
    msg.start_v = [0.0, 0.0, 0.0]
    msg.start_a = [0.0, 0.0, 0.0]
    msg.end_p = [X1 + shift, 0.0, Z]
    msg.end_v = [0.0, 0.0, 0.0]
    msg.end_a = [0.0, 0.0, 0.0]

    msg.inner_x = [1.0 + shift, 2.0 + shift]
    msg.inner_y = [0.0, 0.0]
    msg.inner_z = [Z, Z]
    msg.duration = list(DURATIONS)
    return msg


def check_traj(msg):
    """结构合法性（与 ego_replan_fsm.cpp:1204-1213 的检查一致）+ 内容比对"""
    problems = []

    if msg.order != 5:
        problems.append("order=%d（应为 5，FSM 只支持 5）" % msg.order)
    if len(msg.duration) != len(msg.inner_x) + 1:
        problems.append("duration(%d) != inner_x(%d)+1（FSM:1209 会直接拒绝）"
                        % (len(msg.duration), len(msg.inner_x)))

    expect = make_traj(msg.traj_id, msg.drone_id)

    def cmp(name, got, want, tol=1e-6):
        if len(got) != len(want):
            problems.append("%s 长度 %d != %d" % (name, len(got), len(want)))
            return
        for i, (a, b) in enumerate(zip(got, want)):
            if abs(a - b) > tol:
                problems.append("%s[%d]=%.6f != %.6f" % (name, i, a, b))
                return

    cmp("start_p", msg.start_p, expect.start_p)
    cmp("end_p", msg.end_p, expect.end_p)
    cmp("inner_x", msg.inner_x, expect.inner_x)
    cmp("inner_y", msg.inner_y, expect.inner_y)
    cmp("inner_z", msg.inner_z, expect.inner_z)
    cmp("duration", msg.duration, expect.duration)
    if abs(msg.des_clearance - DES_CLEARANCE) > 1e-6:
        problems.append("des_clearance=%.6f != %.6f" % (msg.des_clearance, DES_CLEARANCE))

    return problems


class PeerMinTraj(object):
    def __init__(self, pid):
        self.pid = pid
        self.recv = 0
        self.first_traj_id = None
        self.last_traj_id = None
        self.dt_sum = 0.0
        self.dt_abs_max = 0.0
        self.dt_min = None
        self.dt_max = None
        self.bad = 0                 # 结构/内容有问题的帧数
        self.problem_samples = []

    def on_message(self, traj_id, dt, problems):
        self.recv += 1
        if self.first_traj_id is None:
            self.first_traj_id = traj_id
        self.last_traj_id = traj_id
        self.dt_sum += dt
        self.dt_abs_max = max(self.dt_abs_max, abs(dt))
        self.dt_min = dt if self.dt_min is None else min(self.dt_min, dt)
        self.dt_max = dt if self.dt_max is None else max(self.dt_max, dt)
        if problems:
            self.bad += 1
            if len(self.problem_samples) < 3:
                self.problem_samples.append((traj_id, problems))

    def expected(self):
        if self.first_traj_id is None:
            return 0
        return self.last_traj_id - self.first_traj_id + 1

    def loss_rate(self):
        e = self.expected()
        return (max(0, e - self.recv) / float(e)) if e > 0 else 0.0

    def mean_dt(self):
        return self.dt_sum / self.recv if self.recv else float("nan")


class MinTrajTestNode(object):
    def __init__(self):
        self.self_id = rospy.get_param("~self_id", None)
        if self.self_id is None:
            rospy.logfatal("必须指定本机编号：参数 ~self_id 或命令行 --self-id")
            sys.exit(2)
        self.self_id = int(self.self_id)

        self.rate_hz = float(rospy.get_param("~rate", 2.0))
        self.report_period = float(rospy.get_param("~report_period", 2.0))
        self.duration = float(rospy.get_param("~duration", 30.0))  # 秒；<=0 一直跑
        self.loss_warn = float(rospy.get_param("~loss_warn", 0.05))
        self.fsm_warn_dt = float(rospy.get_param("~fsm_warn_dt", 0.25))
        self.fsm_fatal_dt = float(rospy.get_param("~fsm_fatal_dt", 10.0))
        self.expect_peers = self._parse_peers(rospy.get_param("~peers", ""))

        self.lock = threading.Lock()
        self.peers = {}
        self.traj_id = 0
        self.sent = 0
        self.start_time = time.time()
        self.last_report = time.time()
        self.stopped = False

        self.pub = rospy.Publisher(SEND_TOPIC, MinTraj, queue_size=20)
        self.sub = rospy.Subscriber(RECV_TOPIC, MinTraj, self.on_msg, queue_size=100)

        rospy.loginfo("=" * 74)
        rospy.loginfo("race6_mintraj_test 启动（竞赛主通道 controller_msgs/MinTraj）")
        rospy.loginfo("  本机编号 self_id : %d", self.self_id)
        rospy.loginfo("  发送话题         : %s   ←→ Multibotnet 端口 4001", SEND_TOPIC)
        rospy.loginfo("  接收话题         : %s   ← 所有对端都发到这里", RECV_TOPIC)
        rospy.loginfo("  发送频率         : %.2f Hz，每条 %d 段 / %d 个内点",
                      self.rate_hz, PIECES, PIECES - 1)
        rospy.loginfo("  运行时长         : %.1f s%s", self.duration,
                      "（一直跑）" if self.duration <= 0 else "")
        rospy.loginfo("  期望对端         : %s",
                      ("drone_" + ", drone_".join(str(p) for p in self.expect_peers))
                      if self.expect_peers else "未指定（收到谁算谁）")
        rospy.loginfo("=" * 74)

    @staticmethod
    def _parse_peers(raw):
        if raw is None:
            return []
        if isinstance(raw, (list, tuple)):
            return [int(x) for x in raw]
        out = []
        for tok in str(raw).replace(";", ",").split(","):
            tok = tok.strip()
            if tok:
                out.append(int(tok))
        return out

    # ------------------------------------------------------------------
    def on_timer(self, _event):
        if self.stopped:
            return
        with self.lock:
            self.traj_id += 1
            tid = self.traj_id
            self.sent += 1
        self.pub.publish(make_traj(tid, self.self_id))

    def on_msg(self, msg):
        if self.stopped:
            return
        if msg.drone_id == self.self_id:
            return                      # 本机回环，忽略
        if msg.drone_id < 0:
            return

        dt = (rospy.Time.now() - msg.start_time).to_sec()   # ★ FSM 判据
        problems = check_traj(msg)

        with self.lock:
            p = self.peers.get(msg.drone_id)
            if p is None:
                p = PeerMinTraj(msg.drone_id)
                self.peers[msg.drone_id] = p
        p.on_message(msg.traj_id, dt, problems)

    # ------------------------------------------------------------------
    def maybe_report(self):
        now = time.time()
        if now - self.last_report < self.report_period:
            return
        self.last_report = now
        self.report()

    def report(self, final=False):
        with self.lock:
            peers = dict(self.peers)
            sent = self.sent

        tag = "最终结论" if final else "运行中"
        print("")
        print("=" * 108)
        print("[race6_mintraj_test] %s  本机 drone_%d  已运行 %.1f s  本机已发 %d 条 MinTraj"
              % (tag, self.self_id, time.time() - self.start_time, sent))
        if not peers:
            print("  ⚠️ 还没收到任何 MinTraj。这条通道有 4 个特有坑，逐项检查：")
            print("     1) 两端都编译并 source 了 controller_msgs？")
            print("        rospack find controller_msgs  → 应为 ~/six_ws/src/planner/controller_msgs")
            print("     2) config/*.yaml 里 MinTraj 那条的 message_type 是否写成")
            print("        controller_msgs/MinTraj（斜杠，不是点号）？")
            print("     3) 话题名是否就是 %s / %s ？" % (SEND_TOPIC, RECV_TOPIC))
            print("        （必须与 advanced_param_swarm.xml:50-51 的 remap 一致）")
            print("     4) multibotnet 端口 4001 两端是否都绑上了？")
            print("        netstat -tlnp | grep 4001")
            print("=" * 108)
            return False

        print("%-8s %8s %8s %8s %12s %12s %12s %10s  %s"
              % ("对端", "收到", "应到", "丢包率", "dt 均值", "dt 最小",
                 "dt 最大", "|dt|峰值", "判定"))
        print("-" * 108)

        all_pass = True
        for pid in sorted(peers.keys()):
            p = peers[pid]
            ok = (p.loss_rate() <= self.loss_warn
                  and p.bad == 0
                  and p.dt_abs_max <= self.fsm_warn_dt)
            all_pass = all_pass and ok
            print("%-8s %8d %8d %7.2f%% %9.2f ms %9.2f ms %9.2f ms %7.2f ms  %s"
                  % ("drone_%d" % pid, p.recv, p.expected(), p.loss_rate() * 100.0,
                     p.mean_dt() * 1e3,
                     (p.dt_min or 0.0) * 1e3,
                     (p.dt_max or 0.0) * 1e3,
                     p.dt_abs_max * 1e3,
                     "PASS" if ok else "FAIL"))

        print("-" * 108)
        print("  dt = 本机 ros::Time::now() - msg.start_time  ← 正是 ego_replan_fsm.cpp:1223 的判据")
        print("  dt 很小(几 ms)说明两机时钟对齐；dt 出现恒定大偏差说明系统时钟没同步；")
        print("  |dt| > %.0f ms → FSM 刷告警；|dt| > %.0f s → FSM 直接丢弃该轨迹"
              % (self.fsm_warn_dt * 1e3, self.fsm_fatal_dt))

        for pid in sorted(peers.keys()):
            p = peers[pid]
            if p.bad:
                print("  ❌ drone_%d 有 %d 帧结构/内容异常，样例：" % (pid, p.bad))
                for tid, probs in p.problem_samples:
                    print("       traj_id=%d: %s" % (tid, "; ".join(probs)))
            if p.dt_abs_max > self.fsm_fatal_dt:
                print("  ❌ drone_%d 的 |dt| 峰值 %.1f s > %.0f s —— 这些轨迹会被 FSM 丢弃！"
                      % (pid, p.dt_abs_max, self.fsm_fatal_dt))

        for pid in self.expect_peers:
            if pid not in peers:
                print("  ❌ 期望的对端 drone_%d 完全没有收到任何 MinTraj" % pid)
                all_pass = False

        print("  本轮判定：%s" % ("PASS ✅（竞赛主通道已通）" if all_pass else "FAIL ❌"))
        print("=" * 108)
        return all_pass

    def shutdown(self):
        self.stopped = True
        self.timer.shutdown()
        return self.report(final=True)


def main():
    parser = argparse.ArgumentParser(description="race6 MinTraj 主通道验证")
    parser.add_argument("--self-id", type=int, default=None)
    parser.add_argument("--peers", type=str, default=None)
    parser.add_argument("--rate", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None)
    args, _unknown = parser.parse_known_args()

    rospy.init_node("race6_mintraj_test", anonymous=False)

    if args.self_id is not None:
        rospy.set_param("~self_id", args.self_id)
    if args.peers is not None:
        rospy.set_param("~peers", args.peers)
    if args.rate is not None:
        rospy.set_param("~rate", args.rate)
    if args.duration is not None:
        rospy.set_param("~duration", args.duration)

    node = MinTrajTestNode()
    node.timer = rospy.Timer(rospy.Duration(1.0 / node.rate_hz), node.on_timer)

    rate = rospy.Rate(10)
    if node.duration > 0:
        deadline = time.time() + node.duration
        while not rospy.is_shutdown() and time.time() < deadline:
            node.maybe_report()
            rate.sleep()
        ok = node.shutdown()
    else:
        while not rospy.is_shutdown():
            node.maybe_report()
            rate.sleep()
        ok = node.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
