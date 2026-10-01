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
# drone_{did}.yaml —— 6 机通用 Multibotnet 配置（软件编号 {did} = 物理 {phys} 号机）
#   ★ 本文件用于【物理 {phys} 号机】，启动时用 drone_id:={did}
{note}
# 本文件由 scripts/gen_race6_configs.py 生成（也可手改）。
#
# ★★ 编号：软件 drone_id 必须从 0 开始（物理 1 号机 = drone_id 0）★★
#   否则没有任何一台的 id 是 0，六台【谁都不会起飞】——原因见 drone_0.yaml 头部
#   （ego_replan_fsm.cpp:173 的起跑条件 + :1306-1320 要求 swarm_traj[0].drone_id == 0）。
#   映射：物理 1→0 ｜ 2→1 ｜ 3→2 ｜ 4→3 ｜ 5→4 ｜ 6→5
#   用脚本起最省事： bash race6_launch.sh {phys}
#
# ★ 关键约束：发和收必须是两个不同的话题名，否则广播风暴
#     traj  发 /broadcast_traj_from_planner  收 /broadcast_traj_to_planner
#     odom  发 /mavros/local_position/odom   收 /mbn/droneN_odom（仅监控用）
#     sync  发 /race6/timesync_tx            收 /race6/timesync_rx
#     link  发 /race6/comm_test_tx           收 /race6/comm_test_rx
#   端口（所有飞机相同，主机不同）：4001 / 4101 / 4201 / 4401
#
# 【★ 上真机前务必确认 IP 段是本组六台飞机的真实 IP，且所有配置完全一致】★
#   自检：python3 scripts/check_config.py --all
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


def build(ip_list, did, note=""):
    """生成 drone_<did> 的完整配置文本（did 是【软件编号】0..N-1，物理编号 = did+1）"""
    out = [HEADER.format(did=did, phys=did + 1, note=note)]
    for i, ip in enumerate(ip_list):
        if i == did:
            mark = "          # 物理 %d 号机（本机，drone_id:=%d）" % (i + 1, did)
        else:
            mark = "          # 物理 %d 号机" % (i + 1)
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
    ap.add_argument("--ips",
                    default=",".join("192.168.66.%d" % (101 + i) for i in range(6)),
                    help="逗号分隔的 IP，顺序 = 物理 1 号机..物理 6 号机"
                         "（= 软件 drone0..drone5）。默认 192.168.66.101..106")
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
    print("")
    print("  物理编号   软件 drone_id   配置文件             IP                 接收")
    print("  --------   ------------   ------------------   ----------------   ----")
    for i, ip in enumerate(ips):
        print("   %d 号机       %d          drone_%d.yaml      %-16s   %s"
              % (i + 1, i, i, ip,
                 ",".join(str(j) for j in range(len(ips)) if j != i)))
    print("")
    print("  ★ 软件 drone_id 必须从 0 开始，不能直接用 1~6（否则六台全都不起飞）")

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

    # ------------------------------------------------------------------
    # 顺手把 config/ground2/drone_0.yaml 和 drone_1.yaml 同步成同样内容。
    # ground2/ 是历史遗留路径（早期只服务物理 1↔2），但现在内容与 race6/ 等价；
    # 由生成器一起写，就不会出现"改了一处忘了另一处"。
    # ------------------------------------------------------------------
    if not args.dry_run and os.path.basename(os.path.abspath(args.out)) == "race6":
        g2 = os.path.join(os.path.dirname(os.path.abspath(args.out)), "ground2")
        if os.path.isdir(g2):
            print("")
            for did in (0, 1):
                note = ("#   （本文件位于 config/ground2/，功能内容与 config/race6/drone_%d.yaml 相同；\n"
                        "#     由生成器一起同步，改 IP 时两处不会不一致）\n"
                        "#   新流程请优先用 race6/，或直接跑 scripts/race6_launch.sh <物理编号>。\n"
                        % did)
                path = os.path.join(g2, "drone_%d.yaml" % did)
                with open(path, "w") as fh:
                    fh.write(build(ips, did, note=note))
                print("  同步 %s" % path)

    if not args.dry_run:
        print("")
        print("下一步：")
        print("  1) 自检： python3 %s/../scripts/check_config.py --all" % args.out)
        print("  2) 提交： git -C <仓库根> add race6_comm && git commit -m 'config: 填入六机真实 IP' && git push")
        print("  3) 每台飞机 pull 之后，用物理编号一条命令起（不用自己算 drone_id）：")
        for i in range(len(ips)):
            print("       物理 %d 号机:  bash race6_launch.sh %d" % (i + 1, i + 1))
        print("  ★ 六份配置里的六行 IP 必须完全一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
