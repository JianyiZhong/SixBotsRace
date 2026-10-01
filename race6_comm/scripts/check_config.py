#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race6_comm / scripts/check_config.py —— Multibotnet 配置文件自检（起飞前跑一遍）
=====================================================================

【为什么要这个脚本】
  Multibotnet 有一类【静默失败】：配置写得不对，但节点照常启动、日志一切正常，
  只是网络上永远收不到数据。最典型的一种是 ——
  connect_address 用了 IP 段里【没有定义】的别名：

      IP:
        drone3: '192.168.66.85'
        drone5: '192.168.66.161'
      recv_topics:
        - topic: /broadcast_traj_to_planner
          connect_address: drone0        # ← IP 段里根本没有 drone0
          port: 4001

  因为 topic_manager.cpp:341-352 的 resolveAddress() 找不到键时【原样返回字符串】：

      return key;        // 于是连接地址变成 tcp://drone0:4001

  ZMQ 会把 drone0 当【主机名】去做 DNS 解析，解析不出来就永远连不上。
  表现：send 侧正常在涨，recv 侧全是 0，日志里既没有报错、也没有
  "receiving data from network"。极难查。

  ★ 判别口诀：Multibotnet 配置表里 Receive Topics 的 "<-" 后面应该是 IP；
    显示成"名字"（如 drone0）就说明这个别名没定义。

【用法】
  python3 check_config.py --all                          # 检查 <race6_comm>/config 下全部
  python3 check_config.py --dir config/race6             # 只查某个目录（递归）
  python3 check_config.py config/ground2/drone_0.yaml    # 只查指定文件
  python3 check_config.py --quiet --all                  # 只输出问题

【退出码】0 = 全部可通过；1 = 有问题（不要上飞机）
"""

from __future__ import print_function

import argparse
import os
import re
import sys

try:
    import yaml
except ImportError:
    print("需要 PyYAML：sudo apt install -y python3-yaml")
    sys.exit(2)

# Multibotnet 的 IP 段里这两个是保留字，不算别名
RESERVED_ALIASES = ("self", "*", "localhost")

IP_RE = re.compile(r"^\d{1,3}(\.\d{1,3}){3}$")


def is_ip_literal(s):
    if not IP_RE.match(s):
        return False
    return all(0 <= int(p) <= 255 for p in s.split("."))


class Report(object):
    def __init__(self, path):
        self.path = path
        self.errors = []
        self.warns = []
        self.oks = []
        self.ipmap = {}          # 本文件解析出来的 IP 段，供跨文件一致性检查用

    def err(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warns.append(msg)

    def ok(self, msg):
        self.oks.append(msg)

    @property
    def failed(self):
        return len(self.errors) > 0


def check_file(path):
    rep = Report(path)
    try:
        with open(path, "r") as fh:
            cfg = yaml.safe_load(fh)
    except Exception as exc:                      # noqa: BLE001
        rep.err("YAML 解析失败：%s" % exc)
        return rep

    if not isinstance(cfg, dict):
        rep.err("顶层不是一个映射（YAML 结构不对）")
        return rep

    # ---------------- IP 段 ----------------
    ipmap = cfg.get("IP")
    if not isinstance(ipmap, dict) or not ipmap:
        rep.err("缺少 IP 段，或 IP 段不是映射")
        ipmap = {}
    else:
        rep.ipmap = dict(ipmap)
        if "self" not in ipmap and "*" not in ipmap:
            rep.err("IP 段里缺少 self（send_topics 的 bind_address: self 会用到）")
        for k, v in ipmap.items():
            if k in ("self", "*"):
                continue
            if not isinstance(v, str):
                rep.err("IP 段 %s 的值不是字符串：%r" % (k, v))
                continue
            if is_ip_literal(v):
                continue
            if k == "localhost" or v in ("127.0.0.1", "localhost"):
                continue
            rep.warn("IP 段 %s -> '%s' 不是 IP 字面量，会走主机名解析（现场建议写 IP）" % (k, v))

    def resolve(key):
        """复刻 Multibotnet 的 resolveAddress 语义"""
        if key in ("self", "*"):
            return "*"
        if isinstance(ipmap, dict) and key in ipmap:
            return str(ipmap[key])
        return key                              # ★ 找不到就原样返回

    # ---------------- send_topics ----------------
    sends = cfg.get("send_topics") or []
    if not isinstance(sends, list) or not sends:
        rep.err("缺少 send_topics")
        sends = []
    send_ports = {}
    send_topics = set()
    for i, t in enumerate(sends):
        tag = "send_topics[%d]" % i
        if not isinstance(t, dict):
            rep.err("%s 不是一个映射" % tag)
            continue
        topic = t.get("topic")
        mtype = t.get("message_type")
        port = t.get("port")
        freq = t.get("max_frequency")
        bind = t.get("bind_address")
        # config_parser.cpp:191-193 要求 max_frequency 和 bind_address 必须存在
        for key, val in (("topic", topic), ("message_type", mtype), ("port", port),
                         ("max_frequency", freq), ("bind_address", bind)):
            if val is None:
                rep.err("%s(%s) 缺少必填键 %s（缺了 Multibotnet 会启动失败）"
                        % (tag, topic or "?", key))
        if not isinstance(port, int):
            rep.err("%s(%s) port 不是整数：%r" % (tag, topic, port))
        else:
            if port in send_ports:
                rep.err("端口 %d 被两个 send_topics 同时绑定（%s 和 %s）→ bind 会失败"
                        % (port, send_ports[port], topic))
            send_ports[port] = topic
        if isinstance(mtype, str) and "/" not in mtype:
            rep.err("%s(%s) message_type 应该是 '包名/消息名'（斜杠），现在是 %r"
                    % (tag, topic, mtype))
        if isinstance(bind, str):
            if bind not in RESERVED_ALIASES and bind not in ipmap:
                rep.err("%s(%s) bind_address: %s 不在 IP 段里（会绑到错误地址）"
                        % (tag, topic, bind))
        if topic:
            send_topics.add(str(topic))
        rep.ok("send %s [%s] -> %s:%s" % (topic, mtype, resolve(bind) if bind else "?",
                                          port if port is not None else "?"))

    # ---------------- recv_topics ----------------
    recvs = cfg.get("recv_topics") or []
    if not isinstance(recvs, list) or not recvs:
        rep.err("缺少 recv_topics")
        recvs = []
    for i, t in enumerate(recvs):
        tag = "recv_topics[%d]" % i
        if not isinstance(t, dict):
            rep.err("%s 不是一个映射" % tag)
            continue
        topic = t.get("topic")
        mtype = t.get("message_type")
        port = t.get("port")
        addr = t.get("connect_address")
        for key, val in (("topic", topic), ("message_type", mtype), ("port", port),
                         ("connect_address", addr)):
            if val is None:
                rep.err("%s(%s) 缺少必填键 %s（缺了 Multibotnet 会启动失败）"
                        % (tag, topic or "?", key))
        if isinstance(mtype, str) and "/" not in mtype:
            rep.err("%s(%s) message_type 应该是 '包名/消息名'（斜杠），现在是 %r"
                    % (tag, topic, mtype))

        if isinstance(addr, str):
            resolved = resolve(addr)
            if resolved == addr and addr not in RESERVED_ALIASES and not is_ip_literal(addr):
                # ★ 这就是本次踩到的坑
                rep.err("%s(%s) connect_address: %s —— IP 段里【没有】这个别名！"
                        % (tag, topic, addr))
                rep.err("      Multibotnet 会把它当主机名，连 tcp://%s:%s，DNS 解析失败后"
                        "【永远收不到数据且不报错】" % (addr, port))
                rep.err("      修法：在 IP 段里加一行 `%s: '<对端真实IP>'`，"
                        "或把 connect_address 改成 IP 段里已有的名字" % addr)
            else:
                if resolved != addr:
                    rep.ok("recv %s <- %s (%s:%s)" % (topic, addr, resolved, port))
                else:
                    rep.ok("recv %s <- %s:%s" % (topic, resolved, port))
        # 发/收同名 → 广播风暴
        if topic and str(topic) in send_topics:
            rep.err("%s(%s) 这个话题名同时也出现在 send_topics 里！"
                    % (tag, topic))
            rep.err("      收发同名 → 收到的报文会被再次广播 → 报文自我复制（广播风暴）")
            rep.err("      发和收必须是两个不同的名字（见 advanced_param_swarm.xml:50-51）")

    return rep


def iter_yaml(dirs, files):
    out = list(files)
    for d in dirs:
        if not os.path.isdir(d):
            print("目录不存在：%s" % d)
            continue
        for root, _sub, names in os.walk(d):
            for n in sorted(names):
                if n.endswith((".yaml", ".yml")):
                    out.append(os.path.join(root, n))
    return out


def default_config_dir():
    """找出 race6_comm 的 config 目录。
    三种运行位置都要能用：源码树里 / devel/lib（rosrun）/ share 安装目录。"""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.normpath(os.path.join(here, os.pardir, "config")),
        os.path.normpath(os.path.join(here, os.pardir, os.pardir,
                                      "share", "race6_comm", "config")),
    ]
    try:
        import rospkg
        cands.insert(0, os.path.join(rospkg.RosPack().get_path("race6_comm"), "config"))
    except Exception:                             # noqa: BLE001  没装 rospkg / 没 source 也能用
        pass
    for c in cands:
        if os.path.isdir(c):
            return c
    return cands[0]


def main():
    default_dir = default_config_dir()

    ap = argparse.ArgumentParser(description="Multibotnet 配置自检")
    ap.add_argument("files", nargs="*", help="要检查的 yaml 文件")
    ap.add_argument("--dir", action="append", default=[],
                    help="要递归检查的目录（可多次指定）")
    ap.add_argument("--all", action="store_true",
                    help="检查 <race6_comm>/config 下全部 yaml")
    ap.add_argument("--quiet", action="store_true", help="只输出问题")
    args = ap.parse_args()

    dirs = list(args.dir)
    if args.all or (not dirs and not args.files):
        dirs.append(default_dir)

    targets = iter_yaml(dirs, args.files)
    if not targets:
        print("没有找到要检查的 yaml")
        return 1

    bad = 0
    reps = []
    for path in targets:
        rep = check_file(path)
        reps.append(rep)
        if not args.quiet or rep.failed or rep.warns:
            print("")
            print("=" * 78)
            print("检查 %s" % path)
            print("=" * 78)
            for m in rep.oks:
                if not args.quiet:
                    print("  [ OK ] %s" % m)
            for m in rep.warns:
                print("  [WARN] %s" % m)
            for m in rep.errors:
                print("  [FAIL] %s" % m)
        if rep.failed:
            bad += 1
            print("  → %s 有 %d 个错误" % (os.path.basename(path), len(rep.errors)))

    # ------------------------------------------------------------------
    # 跨文件一致性：同一个别名在所有配置里必须指向同一个 IP。
    # 这一条专治"改了一份忘了另一份"（例如只改了 race6/ 没改 ground2/，
    # 或者六份里有五份填了真实 IP、有一份还是占位值）——这种情况不会报错，
    # 只会让某一台静默收不到数据。
    # ------------------------------------------------------------------
    alias_vals = {}
    for rep in reps:
        for k, v in rep.ipmap.items():
            if k in ("self", "*"):
                continue
            alias_vals.setdefault(str(k), {}).setdefault(str(v), []).append(
                os.path.basename(os.path.dirname(rep.path)) + "/" + os.path.basename(rep.path))

    inconsistent = {k: v for k, v in alias_vals.items() if len(v) > 1}
    if inconsistent:
        print("")
        print("-" * 78)
        print("[FAIL] 跨文件不一致：同一个别名在不同配置里指向了不同的 IP")
        for k in sorted(inconsistent):
            print("  别名 %s：" % k)
            for val in sorted(inconsistent[k]):
                print("      %-16s  <-  %s" % (val, ", ".join(inconsistent[k][val])))
        print("  处理：一律用生成器重写所有配置（它会同时写 race6/ 和 ground2/），")
        print("        python3 scripts/gen_race6_configs.py --ips <6个IP>")
        print("        手改的话必须把每一份都改一致。")
        bad += 1

    print("")
    print("-" * 78)
    if bad:
        print("❌ %d/%d 个配置文件有问题，先修掉再上飞机。" % (bad, len(targets)))
        return 1
    print("✅ 全部 %d 个配置文件通过（含跨文件 IP 一致性）。" % len(targets))
    return 0


if __name__ == "__main__":
    sys.exit(main())
