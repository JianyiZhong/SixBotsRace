#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race6_comm / race6_comm_test.py —— 多机链路收发验证（方向、丢包、延迟、内容完整性）
=====================================================================

【它验证什么】
  在 Multibotnet 真实链路上，每个方向都跑一路带序号的报文，检查四件事：
    1. 方向通不通     —— 对端能不能收到我发的
    2. 丢不丢包       —— 用序号缺口算丢包率（比按时间估算准）
    3. 延迟多少       —— 单向延迟 = 接收时刻 - 发送时刻（需要两机时钟已同步）
    4. 内容对不对     —— 净荷按固定模式填充，收到后逐一比对，算内容损坏数

【为什么延迟要注明「需要时钟同步」】
  data[1] 是发送方的墙上时钟。若两机时钟差 100 ms，算出来的单向延迟就整体差 100 ms，
  甚至出现负值。所以：
    * 看到「负延迟」或「延迟 = 某个恒定大数」→ 不是网络问题，是时钟没同步
    * 先跑 race6_time_sync.py 再跑本脚本，两个结果互相印证

【话题】发送 /race6/comm_test_tx   接收 /race6/comm_test_rx   std_msgs/Float64MultiArray
    data[0] = 序号 seq
    data[1] = 发送时的墙上时钟 time.time()
    data[2] = 发送方 self_id
    data[3..] = 净荷，第 k 个 = seq * 1000 + k  （用于校验内容完整性）

  ⚠️ 发和收必须是两个不同的名字：Multibotnet 的发送话题是本地订阅、
     接收话题是本地发布。同名会导致「收进来的又发出去」的自我复制风暴。
     （实验室代码 /broadcast_traj_from_planner 与 /broadcast_traj_to_planner
       就是刻意分开的两个名字，见 advanced_param_swarm.xml:50-51。）

【用法】
  # 跑 30 秒后自动出结论
  rosrun race6_comm race6_comm_test.py --self-id 0 --peers 1
  rosrun race6_comm race6_comm_test.py --self-id 1 --peers 0 --duration 30
  # 由 launch 统一拉起（推荐，和 multibotnet 一起）
  roslaunch race6_comm comm_ground_test.launch drone_id:=0

【退出码】0 = 全部对端 PASS；1 = 有对端 FAIL（方便脚本化）
"""

from __future__ import print_function

import argparse
import sys
import threading
import time

import rospy
from std_msgs.msg import Float64MultiArray

TOPIC_SEND = "/race6/comm_test_tx"
TOPIC_RECV = "/race6/comm_test_rx"
PAYLOAD_DEFAULT = 8          # 净荷 float 个数


class PeerLink(object):
    def __init__(self, pid):
        self.pid = pid
        self.recv = 0
        self.first_seq = None
        self.last_seq = None
        self.lat_sum = 0.0
        self.lat_min = None
        self.lat_max = None
        self.negative_lat = 0
        self.corrupt = 0

    def on_message(self, seq, lat, payload_ok):
        self.recv += 1
        if self.first_seq is None:
            self.first_seq = seq
        self.last_seq = seq
        if lat is not None:
            self.lat_sum += lat
            self.lat_min = lat if self.lat_min is None else min(self.lat_min, lat)
            self.lat_max = lat if self.lat_max is None else max(self.lat_max, lat)
            if lat < -0.001:
                self.negative_lat += 1
        if not payload_ok:
            self.corrupt += 1

    def expected(self):
        if self.first_seq is None:
            return 0
        return self.last_seq - self.first_seq + 1

    def lost(self):
        return max(0, self.expected() - self.recv)

    def loss_rate(self):
        e = self.expected()
        return (self.lost() / float(e)) if e > 0 else 0.0

    def mean_lat(self):
        return self.lat_sum / self.recv if self.recv else float("nan")


class CommTestNode(object):
    def __init__(self):
        self.self_id = rospy.get_param("~self_id", None)
        if self.self_id is None:
            rospy.logfatal("必须指定本机编号：参数 ~self_id 或命令行 --self-id")
            sys.exit(2)
        self.self_id = int(self.self_id)

        self.rate_hz = float(rospy.get_param("~rate", 5.0))
        self.payload_len = int(rospy.get_param("~payload", PAYLOAD_DEFAULT))
        self.report_period = float(rospy.get_param("~report_period", 1.0))
        self.duration = float(rospy.get_param("~duration", 30.0))
        self.loss_warn = float(rospy.get_param("~loss_warn", 0.05))     # 5%
        self.lat_warn = float(rospy.get_param("~lat_warn", 0.05))       # 50 ms
        self.expect_peers = self._parse_peers(rospy.get_param("~peers", ""))

        self.lock = threading.Lock()
        self.peers = {}
        self.seq = 0
        self.sent = 0
        self.start_time = time.time()
        self.last_report = time.time()
        self.stopped = False

        self.pub = rospy.Publisher(TOPIC_SEND, Float64MultiArray, queue_size=50)
        self.sub = rospy.Subscriber(TOPIC_RECV, Float64MultiArray, self.on_msg, queue_size=200)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.on_timer)

        rospy.loginfo("=" * 68)
        rospy.loginfo("race6_comm_test 启动")
        rospy.loginfo("  本机编号 self_id : %d", self.self_id)
        rospy.loginfo("  发送频率         : %.2f Hz", self.rate_hz)
        rospy.loginfo("  净荷长度         : %d 个 float", self.payload_len)
        rospy.loginfo("  运行时长         : %.1f s%s", self.duration,
                      "（一直跑）" if self.duration <= 0 else "")
        rospy.loginfo("  期望对端         : %s",
                      ("drone_" + ", drone_".join(str(p) for p in self.expect_peers))
                      if self.expect_peers else "未指定（收到谁算谁）")
        rospy.loginfo("  发/收话题        : %s  →  %s", TOPIC_SEND, TOPIC_RECV)
        rospy.loginfo("=" * 68)

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
            self.seq += 1
            seq = self.seq
            self.sent += 1
        t_send = time.time()
        msg = Float64MultiArray()
        data = [float(seq), t_send, float(self.self_id)]
        for k in range(self.payload_len):
            data.append(float(seq * 1000 + k))
        msg.data = data
        self.pub.publish(msg)

    def on_msg(self, msg):
        if self.stopped:
            return
        data = list(msg.data)
        if len(data) < 3:
            return
        seq = int(data[0])
        t_send = data[1]
        sender = int(data[2])
        if sender == self.self_id:
            return                       # 本机自己发的（ROS 回环），忽略

        # 内容完整性：净荷第 k 个应当等于 seq*1000+k
        ok = True
        for k, v in enumerate(data[3:]):
            if abs(v - (seq * 1000 + k)) > 1e-6:
                ok = False
                break

        lat = time.time() - t_send
        with self.lock:
            p = self.peers.get(sender)
            if p is None:
                p = PeerLink(sender)
                self.peers[sender] = p
        p.on_message(seq, lat, ok)

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

        elapsed = max(1e-6, time.time() - self.start_time)
        tag = "最终结论" if final else "运行中"
        print("")
        print("=" * 104)
        print("[race6_comm_test] %s  本机 drone_%d  已运行 %.1f s  本机已发 %d 帧 (%.2f Hz)"
              % (tag, self.self_id, elapsed, sent, sent / elapsed))
        if not peers:
            print("  ⚠️ 还没收到任何对端的 comm_test 报文。按这个顺序查（前两条是【静默失败】，最容易漏）：")
            print("     1) ★ Multibotnet 配置表里 Receive Topics 那几行，'<-' 后面【必须是 IP】。")
            print("        显示成一个名字（例如  <- drone0:4001）= 该别名没在 IP 段里定义，")
            print("        Multibotnet 会把它当主机名去解析，解析不了就永远连不上，而且【一个错都不报】。")
            print("     2) 跑配置自检： rosrun race6_comm check_config.py --all")
            print("     3) 对端是否也在跑本脚本？两端 multibotnet 是否都起来了？")
            print("        对端日志应出现: Topic '/race6/comm_test_rx' [...] receiving data from network")
            print("     4) 本机 rostopic hz %s 应 ≈ %.1f Hz（本机自己发的）" % (TOPIC_SEND, self.rate_hz))
            print("     5) 网络：ping 对端；防火墙 sudo ufw allow 4401；路由器是否开了客户端隔离")
            print("=" * 104)
            return False

        print("%-8s %8s %8s %8s %9s %11s %11s %11s %8s %8s"
              % ("对端", "收到", "应到", "丢包率", "内容损坏", "延迟均值", "延迟最小", "延迟最大", "负延迟", "判定"))
        print("-" * 104)

        all_pass = True
        for pid in sorted(peers.keys()):
            p = peers[pid]
            ok = (p.loss_rate() <= self.loss_warn
                  and p.corrupt == 0
                  and (p.mean_lat() <= self.lat_warn or p.mean_lat() != p.mean_lat()))
            all_pass = all_pass and ok
            print("%-8s %8d %8d %7.2f%% %9d %9.2f ms %9.2f ms %9.2f ms %8d %8s"
                  % ("drone_%d" % pid, p.recv, p.expected(), p.loss_rate() * 100.0,
                     p.corrupt,
                     p.mean_lat() * 1e3,
                     (p.lat_min or 0.0) * 1e3,
                     (p.lat_max or 0.0) * 1e3,
                     p.negative_lat,
                     "PASS" if ok else "FAIL"))

        print("-" * 104)
        for pid in self.expect_peers:
            if pid not in peers:
                print("  ❌ 期望的对端 drone_%d 完全没有收到任何报文" % pid)
                all_pass = False
        for pid in sorted(peers.keys()):
            if peers[pid].negative_lat > 0:
                print("  ⚠️ drone_%d 出现 %d 次负延迟 → 两机系统时钟没对齐，"
                      "先跑 race6_time_sync.py" % (pid, peers[pid].negative_lat))
        print("  判定线：丢包 ≤ %.1f%% 且 内容损坏 = 0 且 延迟 ≤ %.0f ms"
              % (self.loss_warn * 100.0, self.lat_warn * 1e3))
        print("  本轮判定：%s" % ("PASS ✅" if all_pass else "FAIL ❌"))
        print("=" * 104)
        return all_pass

    def shutdown(self):
        self.stopped = True
        self.timer.shutdown()
        return self.report(final=True)


def main():
    parser = argparse.ArgumentParser(description="race6 多机链路收发验证")
    parser.add_argument("--self-id", type=int, default=None)
    parser.add_argument("--peers", type=str, default=None, help="期望对端，逗号分隔")
    parser.add_argument("--rate", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--payload", type=int, default=None)
    args, _unknown = parser.parse_known_args()

    rospy.init_node("race6_comm_test", anonymous=False)

    if args.self_id is not None:
        rospy.set_param("~self_id", args.self_id)
    if args.peers is not None:
        rospy.set_param("~peers", args.peers)
    if args.rate is not None:
        rospy.set_param("~rate", args.rate)
    if args.duration is not None:
        rospy.set_param("~duration", args.duration)
    if args.payload is not None:
        rospy.set_param("~payload", args.payload)

    node = CommTestNode()

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
