#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race6_comm / scripts/gen_race6_configs.py —— 一次生成六份竞赛 Multibotnet 配置
=====================================================================

【为什么需要它】
  6 机竞赛的 6 份配置，内容只差「接收列表里连谁」，但每份都要写全 6 台飞机的
  IP。手改 6 个文件很容易漏掉一个（漏掉的后果是那两台互相收不到，
  而且要排查很久）。这个脚本让「改 IP」变成一条命令。

【用法】
  # 用真实 IP 重新生成 6 份（会覆盖 config/race6/drone_0..5.yaml）
  python3 ~/six_ws/src/race6_comm/scripts/gen_race6_configs.py \\
          --ips 192.168.1.101,192.168.1.102,192.168.1.103,\\
192.168.1.104,192.168.1.105,192.168.1.106

  # 先看会写成什么样，不落盘
  python3 .../gen_race6_configs.py --ips 10.0.0.11,10.0.0.12 --dry-run

  # 只做地面双机配置（drone_0 / drone_1）
  python3 .../gen_race6_configs.py --ips <ip0>,<ip1> --out config/ground2

【生成的 YAML 内容与仓库里现成的 6 份等价】（注释措辞可能略有差别）

【关键设计，改之前必须看懂】
  1) 发和收必须是两个不同的话题名，否则广播风暴：
       发 /race6/timesync_tx   收 /race6/timesync_rx
       发 /race6/comm_test_tx  收 /race6/comm_test_rx
     轨迹那条由实验室 launch 定死：发 /broadcast_traj_from_planner、
     收 /broadcast_traj_to_planner（advanced_param_swarm.xml:50-51）。
  2) 端口用「同端口、不同主机」方案：六台飞机各是独立主机，
     绑相同端口号不冲突，所以配置里没有 4001+id 这种算术。
     4001 traj / 4201 timesync / 4401 comm_test。
     （单机多机仿真必须改成错开端口，见 README「第二阶段」。）
"""

from __future__ import print_function

import argparse
import os
import sys

HEADER = """\
# =====================================================================
# race6_comm / config/race6/drone_{did}.yaml —— 竞赛配置：{did} 号机
#
# 本文件由 scripts/gen_race6_configs.py 生成（也可手改）。
# 端口方案、发/收话题必须分开的原因，详见 drone_0.yaml 的注释。
#
#   ★ 关键约束：发和收必须是两个不同的话题名，否则广播风暴
#     traj  发 /broadcast_traj_from_planner  收 /broadcast_traj_to_planner
#     odom  发 /mavros/local_position/odom   收 /mbn/droneN_odom（仅监控用）
#     sync  发 /race6/timesync_tx            收 /race6/timesync_rx
#     link  发 /race6/comm_test_tx           收 /race6/comm_test_rx
#   端口（所有飞机相同，主机不同）：4001 / 4101 / 4201 / 4401
#
# 【★ 上真机前务必确认 IP 段是本组六台飞机的真实 IP】★
# =====================================================================

IP:
  self: '*'
  localhost: '127.0.0.1'
"""

SEND_BLOCK = """\

send_topics:
  # 轨迹规划（竞赛主通道，名字由实验室 launch 定死）
  - topic: /broadcast_traj_from_planner
    message_type: controller_msgs/MinTraj
    max_frequency: 20
    bind_address: self
    port: 4001
    compression: true
  # 里程计：给对端/地面站看各自位置。
  # ⚠️ planner 本身【不消费】别机里程计（它只订阅自己的 odom_world，
  #    预测别机位置用的是 MinTraj），所以这条纯粹是监控/可视化用途。
  - topic: /mavros/local_position/odom
    message_type: nav_msgs/Odometry
    max_frequency: 20
    bind_address: self
    port: 4101
    compression: false
  # 时间同步（REQ 与 RESP 都从这条 _tx 发出）
  - topic: /race6/timesync_tx
    message_type: std_msgs/Float64MultiArray
    max_frequency: 20
    bind_address: self
    port: 4201
    compression: false
  # 通用链路收发验证
  - topic: /race6/comm_test_tx
    message_type: std_msgs/Float64MultiArray
    max_frequency: 20
    bind_address: self
    port: 4401
    compression: false

recv_topics:
"""

PEER_BLOCK = """\
  # ---- {pid} 号机 ----
  - topic: /broadcast_traj_to_planner
    message_type: controller_msgs/MinTraj
    connect_address: drone{pid}
    port: 4001
  - topic: /mbn/drone{pid}_odom
    message_type: nav_msgs/Odometry
    connect_address: drone{pid}
    port: 4101
  - topic: /race6/timesync_rx
    message_type: std_msgs/Float64MultiArray
    connect_address: drone{pid}
    port: 4201
  - topic: /race6/comm_test_rx
    message_type: std_msgs/Float64MultiArray
    connect_address: drone{pid}
    port: 4401
"""

FOOTER = """\

advanced:
  compression:
    enable: true
    type: lz4
    level: 6
  thread_pool:
    size: 2
  performance:
    enable_statistics: true
    statistics_interval_ms: 5000
  retry:
    max_retries: -1
    interval_ms: 1000
"""


def build(ip_list, did):
    """生成 drone_<did> 的完整配置文本"""
    out = [HEADER.format(did=did)]
    for i, ip in enumerate(ip_list):
        mark = "          # ← 本机" if i == did else ""
        out.append("  drone%d: '%s'%s\n" % (i, ip, mark))
    out.append(SEND_BLOCK)
    for pid in range(len(ip_list)):
        if pid == did:
            continue
        out.append(PEER_BLOCK.format(pid=pid))
    out.append(FOOTER)
    return "".join(out)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)                      # .../race6_comm

    ap = argparse.ArgumentParser(description="生成 race6 Multibotnet 配置")
    ap.add_argument("--ips", required=True,
                    help="逗号分隔的 IP 列表，顺序 = drone0,drone1,...")
    ap.add_argument("--out", default=os.path.join(pkg, "config", "race6"),
                    help="输出目录，默认 <race6_comm>/config/race6")
    ap.add_argument("--dry-run", action="store_true", help="只打印不写文件")
    args = ap.parse_args()

    ips = [s.strip() for s in args.ips.replace(";", ",").split(",") if s.strip()]
    if len(ips) < 2:
        print("至少要有 2 个 IP")
        return 2
    for ip in ips:
        parts = ip.split(".")
        if len(parts) != 4 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
            print("IP 格式不对：%s" % ip)
            return 2

    print("将要生成 %d 份配置 → %s" % (len(ips), args.out))
    for i, ip in enumerate(ips):
        print("  drone_%d.yaml : 本机 %s，收 %s"
              % (i, ip, ",".join(str(j) for j in range(len(ips)) if j != i)))

    if not args.dry_run:
        if not os.path.isdir(args.out):
            os.makedirs(args.out)
    for did in range(len(ips)):
        text = build(ips, did)
        if args.dry_run:
            print("\n" + "=" * 70 + "\n# drone_%d.yaml\n" % did + "=" * 70)
            print(text)
            continue
        path = os.path.join(args.out, "drone_%d.yaml" % did)
        with open(path, "w") as fh:
            fh.write(text)
        print("  已写 %s (%d 字节)" % (path, len(text)))

    if not args.dry_run:
        print("")
        print("提示：把这 %d 份分发到各台飞机上，每台只需要自己那一份：" % len(ips))
        print("  scp %s/drone_1.yaml orangepi@<1号机IP>:~/six_ws/src/race6_comm/config/race6/"
              % args.out)
        print("  或者整个 race6_comm 包一起拷过去。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
