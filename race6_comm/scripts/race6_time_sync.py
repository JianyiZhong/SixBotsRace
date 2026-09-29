#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race6_comm / race6_time_sync.py —— 六机竞赛 · 多机时间同步脚本
=====================================================================

【为什么必须做时间同步】（这是本脚本存在的唯一理由，先看懂这段）
  实验室最新代码 SixBotsRace 的 ego_planner 在收到别机轨迹时，
  会用【本机墙上时钟】和【发送方写进消息里的墙上时钟】直接相减来判定：

      ego_replan_fsm.cpp:1222-1237
        t_now = ros::Time::now();                       // 本机时钟
        if (abs(t_now - msg->start_time) > 0.25) {      // 0.25 s
            if (< 10.0)  ROS_WARN("Time stamp diff ...")     // 只告警，轨迹照收
            else         ROS_ERROR("... not synchronized, abandon!"); return;  // 丢弃！
        }

  而 msg->start_time 在发送方是 `ros::Time::now()` 直接赋的
  （ego_replan_fsm.cpp:1359  polyTraj2ROSMsg）。也就是说：
      两机系统时钟之差 = 该判据里的差值。
  ⇒ 系统时钟差 < 10 s   : 不丢帧，但会刷 WARN，且轨迹相位对不准
  ⇒ 系统时钟差 < 0.25 s  : 完全不告警
  ⇒ 目标               : |offset| < 10 ms（协同避障才真正安全）

  注意：collision check 本身用的是【相对时差】
  （ego_replan_fsm.cpp:546  t_X = t + (info->start_time - swarm_traj[id].start_time)），
  所以恒定的时钟偏差不会立刻撞机；但 0.25 s 以上的偏差会让
  "预测别人位置" 这件事整体错位 0.25 s，1.5 m/s 下就是 0.375 m 的位置误差。

【本脚本做什么】
  用 NTP 四时戳法，在 Multibotnet 链路【真实的报文路径上】测量两机时钟差：
      REQ : [1, seq, t1, origin_id]
      RESP: [2, seq, t1, t2, t3, origin_id, responder_id]
      t1 = 请求方发出时刻（请求方钟）
      t2 = 应答方收到时刻（应答方钟）
      t3 = 应答方发出应答时刻（应答方钟）
      t4 = 请求方收到应答时刻（请求方钟）
      对端钟速 - 本机钟速  θ = ((t2 - t1) + (t3 - t4)) / 2
      往返延迟           rtt = (t4 - t1) - (t3 - t2)
  θ 就是 ego_replan_fsm 判据里的那个差值。往返延迟对称时 θ 无偏。

  测出来之后：
    * 默认【只报不动】，每 report_period 秒打一张表（offset / rtt / 样本数 / 判定）
    * 加 --apply 才去校时（地面测试用；飞行中绝对不要用，见文件末警告）

【话题】发送 /race6/timesync_tx   接收 /race6/timesync_rx   std_msgs/Float64MultiArray
  这条话题由 Multibotnet 跨机搬运（见 config/ground2/*.yaml 的 4201 端口）。
  REQ 和 RESP 都从本机 _tx 发出，都从 _rx 收进来，靠 data[0] 区分，靠 id 过滤自己。

  ⚠️ 为什么【发】和【收】必须是两个不同的话题名（这条非常关键）：
     Multibotnet 的「发送话题」是本地订阅、「接收话题」是本地发布
     （topic_manager.cpp:411-420 / 717-733）。
     如果发和收用同一个名字，那么「收进来的报文」会被「发送话题」再订阅一次、
     又广播出去 —— 报文在网络里自我复制，形成广播风暴，几秒内打满网络。
     实验室代码正是这样设计的：发 /broadcast_traj_from_planner，
     收 /broadcast_traj_to_planner，两个名字不一样。
     （见 advanced_param_swarm.xml:50-51。）
     所以本脚本也严格区分 _tx / _rx。

【用法】
  # 只测量（推荐先这样跑，看清楚偏差到底多大）
  rosrun race6_comm race6_time_sync.py --self-id 0 --peers 1

  # 测量 + 自动校时（地面、桨叶卸下、且 planner 还没起来的时候）
  rosrun race6_comm race6_time_sync.py --self-id 1 --peers 0 --apply

  # 通过 roslaunch 起（参数走私有命名空间）
  roslaunch race6_comm comm_ground_test.launch drone_id:=0

【判定标准（脚本退出时打印）】
  PASS : 全部对端 |offset| <= warn_offset(默认 50 ms) 且 rtt <= 100 ms
  WARN : |offset| 在 (50 ms, 250 ms] —— 不丢帧但会刷告警
  FAIL : |offset| > 250 ms —— 已越过 FSM 告警线；> 10 s 时轨迹会被直接丢弃
"""

from __future__ import print_function

import argparse
import os
import shutil
import statistics
import subprocess
import sys
import threading
import time

import rospy
from std_msgs.msg import Float64MultiArray

# ---------------------------------------------------------------------------
# 报文类型标记（data[0]）
# ---------------------------------------------------------------------------
KIND_REQ = 1.0
KIND_RESP = 2.0

# ★ 发和收必须是两个不同的名字，否则会形成广播风暴（见文件头注释）
SEND_TOPIC = "/race6/timesync_tx"
RECV_TOPIC = "/race6/timesync_rx"

# 单个请求等多久算丢（秒）
PENDING_TIMEOUT = 2.0


class PeerStats(object):
    """单个对端的时间同步统计"""

    def __init__(self, peer_id):
        self.peer_id = peer_id
        self.offsets = []          # 每次测到的 theta
        self.rtts = []             # 每次测到的 rtt
        self.resp_count = 0
        self.req_sent = 0
        self.lost = 0              # 超时未应答

    def add(self, theta, rtt):
        self.offsets.append(theta)
        self.rtts.append(rtt)
        self.resp_count += 1

    def mean_offset(self):
        return statistics.mean(self.offsets) if self.offsets else float("nan")

    def std_offset(self):
        return statistics.pstdev(self.offsets) if len(self.offsets) > 1 else 0.0

    def max_abs_offset(self):
        return max(abs(v) for v in self.offsets) if self.offsets else float("nan")

    def mean_rtt(self):
        return statistics.mean(self.rtts) if self.rtts else float("nan")

    def max_rtt(self):
        return max(self.rtts) if self.rtts else float("nan")

    def median_offset(self):
        return statistics.median(self.offsets) if self.offsets else float("nan")

    def reset(self):
        self.offsets = []
        self.rtts = []
        self.resp_count = 0
        self.req_sent = 0
        self.lost = 0


class TimeSyncNode(object):
    def __init__(self):
        # ---------------- 参数 ----------------
        self.self_id = rospy.get_param("~self_id", None)
        if self.self_id is None:
            rospy.logfatal("必须指定本机编号：参数 ~self_id 或命令行 --self-id")
            sys.exit(2)
        self.self_id = int(self.self_id)

        self.rate_hz = float(rospy.get_param("~rate", 1.0))
        self.warn_offset = float(rospy.get_param("~warn_offset", 0.05))     # 50 ms
        self.fatal_offset = float(rospy.get_param("~fatal_offset", 0.25))  # FSM 告警线
        self.report_period = float(rospy.get_param("~report_period", 5.0))
        self.duration = float(rospy.get_param("~duration", 0.0))  # 0 = 一直跑
        self.apply_mode = bool(rospy.get_param("~apply", False))
        self.apply_gate = float(rospy.get_param("~apply_gate", 0.002))  # 小于 2 ms 不动钟
        self.master_id = int(rospy.get_param("~master_id", -1))  # >=0 时以该机为基准
        self.csv_path = str(rospy.get_param("~csv", ""))
        self.expect_peers = self._parse_peers(rospy.get_param("~peers", ""))

        # ---------------- 状态 ----------------
        self.lock = threading.Lock()
        self.peers = {}                  # peer_id -> PeerStats
        self.pending = {}                # seq -> (t1, peer_id_hint)
        self.seq = 0
        self.req_sent = 0                # 本机发出的 REQ 总数（用来算应答率）
        self.start_time = time.time()
        self.last_report = time.time()
        self.stopped = False
        self.apply_count = 0
        self._csv_file = None

        # 把「期望对端」先建出来：这样一直不应答的对端也会出现在表里显示"无应答"，
        # 而不是干脆不显示（那样最容易被误判成"一切正常"）。
        for pid in self.expect_peers:
            if pid != self.self_id:
                self.peers[pid] = PeerStats(pid)

        # ---------------- ROS ----------------
        self.pub = rospy.Publisher(SEND_TOPIC, Float64MultiArray, queue_size=20)
        self.sub = rospy.Subscriber(RECV_TOPIC, Float64MultiArray, self.on_msg, queue_size=50)

        if self.csv_path:
            try:
                self._csv_file = open(self.csv_path, "w")
                self._csv_file.write("wall_time,self_id,peer_id,theta,rtt\n")
            except Exception as exc:      # noqa: BLE001
                rospy.logwarn("打不开 CSV %s: %s（忽略）", self.csv_path, exc)
                self._csv_file = None

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.on_timer)

        rospy.loginfo("=" * 68)
        rospy.loginfo("race6_time_sync 启动")
        rospy.loginfo("  本机编号 self_id     : %d", self.self_id)
        rospy.loginfo("  探测频率             : %.2f Hz", self.rate_hz)
        rospy.loginfo("  发/收话题            : %s  →  %s", SEND_TOPIC, RECV_TOPIC)
        rospy.loginfo("  告警阈值 warn_offset : %.3f s", self.warn_offset)
        rospy.loginfo("  丢弃阈值(同 FSM)     : %.3f s", self.fatal_offset)
        rospy.loginfo("  校时模式 apply       : %s", "开（会动系统时钟！）" if self.apply_mode else "关（只测量）")
        if self.master_id >= 0:
            rospy.loginfo("  基准机 master_id     : %d", self.master_id)
        rospy.loginfo("=" * 68)

    @staticmethod
    def _parse_peers(raw):
        """把 ~peers 参数（'1,2,3' 或列表）解析成 int 列表"""
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
    # 发探测包
    # ------------------------------------------------------------------
    def on_timer(self, _event):
        if self.stopped:
            return
        now = time.time()
        with self.lock:
            self.seq += 1
            seq = self.seq
            self.req_sent += 1
            self.pending[seq] = (now, None)
            # 清理超时
            expired = [s for s, (t1, _p) in self.pending.items()
                       if now - t1 > PENDING_TIMEOUT]
            for s in expired:
                del self.pending[s]

        msg = Float64MultiArray()
        msg.data = [KIND_REQ, float(seq), now, float(self.self_id)]
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # 收报文：既可能是别人发来的 REQ（要应答），也可能是别人给我的 RESP
    # ------------------------------------------------------------------
    def on_msg(self, msg):
        if self.stopped:
            return
        data = list(msg.data)
        if len(data) < 4:
            return
        kind = data[0]

        if kind == KIND_REQ and len(data) >= 4:
            self.handle_req(data)
        elif kind == KIND_RESP and len(data) >= 7:
            self.handle_resp(data)

    def handle_req(self, data):
        seq = int(data[1])
        t1 = data[2]              # 请求方的发出时刻（请求方钟，本机用不到）
        origin_id = int(data[3])

        # 自己的 REQ 会被本机 ROS 回环收到 → 必须过滤，否则等于跟自己同步
        if origin_id == self.self_id:
            return

        t2 = time.time()          # 本机收到时刻（本机钟）
        resp = Float64MultiArray()
        t3 = time.time()          # 本机发出应答时刻（本机钟）
        resp.data = [KIND_RESP, float(seq), t1, t2, t3,
                     float(origin_id), float(self.self_id)]
        self.pub.publish(resp)

    def handle_resp(self, data):
        seq = int(data[1])
        t1 = data[2]
        t2 = data[3]
        t3 = data[4]
        origin_id = int(data[5])
        responder_id = int(data[6])

        # 只认「我发起、别人应答」的报文
        if origin_id != self.self_id:
            return
        if responder_id == self.self_id:
            return                      # 自己应答自己（回环），无意义

        t4 = time.time()

        theta = ((t2 - t1) + (t3 - t4)) / 2.0        # 对端钟 - 本机钟
        rtt = (t4 - t1) - (t3 - t2)                  # 往返延迟

        with self.lock:
            self.pending.pop(seq, None)
            peer = self.peers.get(responder_id)
            if peer is None:
                peer = PeerStats(responder_id)
                self.peers[responder_id] = peer
            peer.add(theta, rtt)

        if self._csv_file:
            self._csv_file.write("%.6f,%d,%d,%.9f,%.9f\n"
                                 % (t4, self.self_id, responder_id, theta, rtt))
            self._csv_file.flush()

    # ------------------------------------------------------------------
    # 周期性打印
    # ------------------------------------------------------------------
    def maybe_report(self):
        now = time.time()
        if now - self.last_report < self.report_period:
            return
        self.last_report = now
        self.report()

    def report(self):
        with self.lock:
            peers = dict(self.peers)

        print("")
        print("=" * 96)
        print("[race6_time_sync] 本机 drone_%d  已运行 %.1f s" % (self.self_id, time.time() - self.start_time))

        no_data = (not peers) or all(p.resp_count == 0 for p in peers.values())
        if no_data:
            print("  ⚠️ 还没有收到任何对端的应答报文。逐项检查：")
            print("     1) 对端是否也在跑 race6_time_sync.py，且 --self-id 与它的 drone_id 一致？")
            print("     2) 两端 IP 是否已按真实地址填进 config/*.yaml 的 IP 段？")
            print("     3) multibotnet 节点是否两端都起来了？日志里应出现")
            print("        \"Topic '/race6/timesync_rx' [std_msgs/Float64MultiArray] receiving data from network\"")
            print("     4) rostopic hz %s  本机应 ≈ %.1f Hz（本机自己发的 REQ）" % (SEND_TOPIC, self.rate_hz))
            if not peers:
                print("=" * 96)
                return
            print("     （下表里的「无应答」就是这个意思）")

        print("%-8s %8s %8s %12s %11s %12s %11s %11s  %s"
              % ("对端", "应答", "本机发", "offset均值", "offset抖动", "|offset|峰值",
                 "rtt均值", "rtt峰值", "判定"))
        print("-" * 96)

        worst_verdict = "PASS"
        apply_candidates = []

        for pid in sorted(peers.keys()):
            p = peers[pid]
            if p.resp_count == 0:
                # 一个应答都没收到：本机发了 req_sent 个 REQ，它一个都没回
                print("%-8s %8d %8d %12s %11s %12s %11s %11s  %s"
                      % ("drone_%d" % pid, 0, self.req_sent, "-", "-", "-", "-", "-",
                         "无应答"))
                worst_verdict = _worse(worst_verdict, "FAIL")
                continue

            mo = p.mean_offset()
            so = p.std_offset()
            xo = p.max_abs_offset()
            mr = p.mean_rtt()
            xr = p.max_rtt()

            if xo <= self.warn_offset and xr <= 0.1:
                verdict = "PASS"
            elif xo <= self.fatal_offset:
                verdict = "WARN"
            else:
                verdict = "FAIL"
            worst_verdict = _worse(worst_verdict, verdict)

            print("%-8s %8d %8d %10.2f ms %9.2f ms %10.2f ms %8.2f ms %8.2f ms  %s"
                  % ("drone_%d" % pid, p.resp_count, self.req_sent,
                     mo * 1e3, so * 1e3, xo * 1e3, mr * 1e3, xr * 1e3, verdict))

            apply_candidates.append((pid, p))

        print("-" * 96)
        print("  说明：offset = 对端钟 - 本机钟（正是 ego_replan_fsm.cpp:1223 判据里的差值）")
        print("        「应答/本机发」两个数相等说明对端每个探测包都回了；")
        print("        对端少回几个不影响 offset，但说明链路有丢包，值得追。")
        print("  判定线：PASS ≤ %.0f ms ｜ WARN ≤ %.0f ms（会刷告警）｜ FAIL > %.0f ms"
              % (self.warn_offset * 1e3, self.fatal_offset * 1e3, self.fatal_offset * 1e3))
        print("  注意：|offset| > 10 s 时，轨迹会被 FSM 直接丢弃"
              "（ego_replan_fsm.cpp:1231-1236）")
        print("  本轮总判定：%s" % worst_verdict)
        print("=" * 96)

        if self.apply_mode and apply_candidates:
            self.try_apply(apply_candidates)

    # ------------------------------------------------------------------
    # 校时（只有 --apply 才会走到这里）
    # ------------------------------------------------------------------
    def try_apply(self, candidates):
        if self.master_id >= 0:
            target = None
            for pid, p in candidates:
                if pid == self.master_id:
                    target = (pid, p)
                    break
            if target is None:
                print("  [apply] 还没测到基准机 drone_%d，本轮不校时" % self.master_id)
                return
            pid, p = target
        else:
            # 没指定基准机 → 取所有对端 offset 的中位数，抗单点异常
            pid, p = max(candidates, key=lambda kv: kv[1].resp_count)

        theta = p.mean_offset()
        rtt = p.mean_rtt()
        if abs(theta) < self.apply_gate:
            print("  [apply] drone_%d 偏差 %.2f ms < %.2f ms，不动钟"
                  % (pid, theta * 1e3, self.apply_gate * 1e3))
            return
        if rtt > 0.05:
            print("  [apply] drone_%d 的 rtt %.1f ms 偏大，本轮不校时（避免把网络抖动当成时钟差）"
                  % (pid, rtt * 1e3))
            return

        print("  [apply] 依据 drone_%d 校时：offset = %+.2f ms（本机需要 %s %.2f ms）"
              % (pid, theta * 1e3, "前拨" if theta > 0 else "回拨", abs(theta) * 1e3))

        # 方式一：装了 chrony → 让 chronyd 立刻步进（不会和 NTP 打架）
        if shutil.which("chronyc"):
            rc = subprocess.call(["chronyc", "tracking"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if rc == 0:
                out = subprocess.call(["sudo", "-n", "chronyc", "makestep"])
                if out == 0:
                    self.apply_count += 1
                    print("  [apply] 已执行 sudo chronyc makestep（第 %d 次）" % self.apply_count)
                    print("  [apply] ⚠️ 时钟被步进后，请重启本脚本与其他 ROS 节点，再重新测一轮")
                    return
                print("  [apply] chronyc makestep 失败（可能需要免密 sudo），改用 date -s")

        # 方式二：直接改系统时间
        #   theta > 0 表示对端比我快 → 把本机钟往前拨 theta
        target_epoch = time.time() + theta
        cmd = ["sudo", "-n", "date", "-s", "@%.6f" % target_epoch]
        rc = subprocess.call(cmd)
        if rc == 0:
            self.apply_count += 1
            print("  [apply] 已执行 %s（第 %d 次）" % (" ".join(cmd), self.apply_count))
            print("  [apply] ⚠️ 系统时钟被跳变，请重启本脚本与其他 ROS 节点，再重新测一轮")
        else:
            print("  [apply] 校时失败（exit=%d）。两种办法：" % rc)
            print("          1) 给 sudo 免密： echo \"$USER ALL=(ALL) NOPASSWD:ALL\" | sudo tee /etc/sudoers.d/$USER")
            print("          2) 更推荐：用 chrony 做基准（见 README「时间同步」一节的 3 条命令）")

    def shutdown(self):
        self.stopped = True
        self.timer.shutdown()
        if self._csv_file:
            self._csv_file.close()

        with self.lock:
            peers = dict(self.peers)

        print("")
        print("=" * 96)
        print("[race6_time_sync] 收尾统计（本机 drone_%d，运行 %.1f s）"
              % (self.self_id, time.time() - self.start_time))
        no_data = (not peers) or all(p.resp_count == 0 for p in peers.values())
        if no_data:
            print("  ❌ 全程没有收到任何对端应答 —— 通信链路或 IP 配置有问题")
            print("     先看 §5.4 排查表第 1、2 行（ping / 防火墙 / receiving data from network）")
        else:
            all_pass = True
            for pid in sorted(peers.keys()):
                p = peers[pid]
                if p.resp_count == 0:
                    print("  ❌ drone_%d: 无应答" % pid)
                    all_pass = False
                    continue
                xo = p.max_abs_offset()
                good = xo <= self.warn_offset
                all_pass = all_pass and good
                print("  %s drone_%d: %d 个样本, offset 均值 %+.2f ms, 抖动 %.2f ms, "
                      "峰值 %.2f ms, rtt 均值 %.2f ms"
                      % ("✅" if good else "⚠️", pid, p.resp_count,
                         p.mean_offset() * 1e3, p.std_offset() * 1e3, xo * 1e3,
                         p.mean_rtt() * 1e3))
            print("")
            if all_pass:
                print("  ✅ 时间同步达标：全部对端 |offset| ≤ %.0f ms。" % (self.warn_offset * 1e3))
                print("     下一步：rosrun race6_comm race6_comm_test.py --self-id %d --peers %s"
                      % (self.self_id, ",".join(str(x) for x in sorted(peers.keys()))))
            else:
                print("  ⚠️ 未达标。若 offset 恒定但偏大 → 两机系统时钟没对齐，先做时钟同步；")
                print("     若 offset 抖动大而 rtt 也大 → 是网络问题，换信道/拉近距离/关压缩。")
        print("=" * 96)


def _worse(a, b):
    order = {"PASS": 0, "WARN": 1, "FAIL": 2}
    return a if order[a] >= order[b] else b


def main():
    parser = argparse.ArgumentParser(description="race6 多机时间同步")
    parser.add_argument("--self-id", type=int, default=None, help="本机编号 0..5")
    parser.add_argument("--peers", type=str, default=None,
                        help="期望对端编号，逗号分隔；用于把不应答的对端也列出来")
    parser.add_argument("--rate", type=float, default=None, help="探测频率 Hz")
    parser.add_argument("--duration", type=float, default=None,
                        help="跑多少秒后自动结束；0 或不填 = 一直跑")
    parser.add_argument("--apply", action="store_true",
                        help="测量后校时（地面测试用；飞行中禁止）")
    parser.add_argument("--master-id", type=int, default=None,
                        help="以哪台为基准校时；不填 = 自动取应答最多的对端")
    parser.add_argument("--csv", type=str, default=None, help="把每次测量写进 CSV")
    args, _unknown = parser.parse_known_args()

    rospy.init_node("race6_time_sync", anonymous=False)

    if args.self_id is not None:
        rospy.set_param("~self_id", args.self_id)
    if args.peers is not None:
        rospy.set_param("~peers", args.peers)
    if args.rate is not None:
        rospy.set_param("~rate", args.rate)
    if args.duration is not None:
        rospy.set_param("~duration", args.duration)
    if args.csv is not None:
        rospy.set_param("~csv", args.csv)
    if args.apply:
        rospy.set_param("~apply", True)
    if args.master_id is not None:
        rospy.set_param("~master_id", args.master_id)
        rospy.set_param("~apply", True)

    node = TimeSyncNode()

    if node.duration > 0:
        deadline = time.time() + node.duration
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            node.maybe_report()
            rate.sleep()
        node.shutdown()
    else:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            node.maybe_report()
            rate.sleep()
        node.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
