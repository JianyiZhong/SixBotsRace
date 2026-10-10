# race6_comm —— 六机协作竞赛 · 多机通讯 + 时间同步模块

本包只做一件事：**把实验室最新代码 `SixBotsRace` 的多机通讯打通，并补上它缺的时间同步**。
为了不改动实验室代码，所有东西都放在这一个新包里。

- 竞赛任务：6 台无人机分 3 组，每组 2 台，分别前往 3 个任务点投弹后返回起点。
- 我的任务：多机通讯（Multibotnet）+ 多机时间同步（脚本）。
- 本阶段目标：**地面（桨叶卸下）双机跑通「通信 + 时间同步 + 收发验证」**。

---

## 0. 结论速览（先看这段）

| 问题 | 结论 |
|---|---|
| 实验室代码要改哪里？ | **一处都不用改。** 通讯层已经预留好了跨机话题，见 §2 |
| 需要新写什么？ | 只有**时间同步**。实验室没有，但 `ego_planner` 有 250 ms / 10 s 的硬判据，见 §3 |
| 本次新增了什么？ | 1 个新包 `race6_comm`（配置 + 3 个脚本 + 2 个 launch + 1 个建工作区脚本），见 §4 |
| 6 机 vs 实验室原来的 3 机差在哪？ | **只差 YAML 配置**（接收列表从 2 台变 5 台），C++ 一行不动 |
| 现在就能做的一步？ | PC 单机自测 → 地面双机联调，见 §5 |

---

## 1. 实验室最新代码（SixBotsRace）通讯路径梳理

`SixBotsRace` 是**源码库不是工作区**（里面是 `FAST_LIO/`、`MLMapping-Embedded_version/`、
`ego-planner2/src/planner/*`，没有 `src/` 入口），要先用 `scripts/setup_six_ws.sh` 组装成 catkin 工作区。

多机通讯全部集中在 `plan_manage` 这一个包里：

| 环节 | 文件:行 | 事实 |
|---|---|---|
| 发（话题名） | `plan_manage/launch/advanced_param_swarm.xml:50` | `~planning/broadcast_traj_send` → **`/broadcast_traj_from_planner`** |
| 发（建发布者） | `plan_manage/src/ego_replan_fsm.cpp:73` | `advertise<controller_msgs::MinTraj>("planning/broadcast_traj_send", 10)` |
| 发（打包） | `ego_replan_fsm.cpp:1328-1388` | `polyTraj2ROSMsg()` 把 MINCO 轨迹压成 `MinTraj`；注释写着 *"Use MINCO trajectory to minimize the message size in wireless communication"* (`:72`) |
| 发（触发点） | `ego_replan_fsm.cpp:601 / 644 / 723` | 每次重规划成功后 `publish(MINCO_msg)`（共 3 处） |
| 收（话题名） | `advanced_param_swarm.xml:51` | `~planning/broadcast_traj_recv` → **`/broadcast_traj_to_planner`** |
| 收（建订阅者） | `ego_replan_fsm.cpp:74-77` | `subscribe<MinTraj>(..., tcpNoDelay())`，队列 100 |
| 收（处理） | `ego_replan_fsm.cpp:1193-1326` | `RecvBroadcastMINCOTrajCallback`：过滤自己 → 校验 order/duration → 过滤乱序 → **查时钟** → 过滤远处轨迹 → 存 `swarm_traj[]` |
| 消息定义 | `controller_msgs/msg/MinTraj.msg` | 16 字段：`drone_id, traj_id, start_time, des_clearance, order, start/end_p/v/a, inner_x/y/z, duration` |
| 时基来源 | `plan_manage/src/planner_manager.cpp:399` | `setLocalTraj(..., ros::Time::now().toSec())` → 写进 `local_traj.start_time` |
| 时基打包 | `ego_replan_fsm.cpp:1359` | `MINCO_msg.start_time = ros::Time(data->start_time)` ← **发送方的墙上时钟** |
| **时间判据** | `ego_replan_fsm.cpp:1222-1237` | 见 §3，`|本机now − msg.start_time|` 决定告警还是丢弃 |
| 乱序判据 | `ego_replan_fsm.cpp:1251` | `start_time <= 已存的` → `"Old traj received, ignored."` |
| 避碰用时间 | `ego_replan_fsm.cpp:546` | 用**相对时差** `t_X = t + (info->start_time − swarm_traj[id].start_time)` |
| 避碰用距离 | `planner_manager.cpp:420-440` | `checkCollision()`：`dist < getSwarmClearance() + des_clearance` 就重规划 |
| 编队起跑 | `ego_replan_fsm.cpp:171-198` | `SEQUENTIAL_START`：`drone_id<=0` 或者 (`drone_id>=1` 且 `have_recv_pre_agent_`) 才起飞 |
| 前置机判定 | `ego_replan_fsm.cpp:1306-1320` | 靠**收到别机轨迹**来置 `have_recv_pre_agent_` |
| 缓冲区扩容 | `ego_replan_fsm.cpp:1240-1249` | `swarm_traj` 按 `recv_id` 自动 push_back → **6 机不需要改代码** |

**一句话**：多机通讯在实验室代码里已经是「发一条、收一条」的干净结构，
缺的只是**中间那段搬运**——原来用 `swarm_bridge`（UDP）或 `swarm_playground` 的
Multibotnet，现在明确用 Multibotnet。

---

## 2. 为什么通讯层一处都不用改

四条理由，逐条对应到代码：

1. **话题名已经预留**：`advanced_param_swarm.xml:50-51` 把内部相对话题 remap 成了
   `/broadcast_traj_from_planner` 和 `/broadcast_traj_to_planner`。
   **发和收是两个不同名字**——这不是笔误，是必须的（见 §7 坑 1）。
2. **真机上不会撞名**：每台飞机跑自己的 ROS master，`/broadcast_traj_from_planner`
   在每台上都是「本机 planner 发出的轨迹」，语义正好对。
   所以 Multibotnet 的配置只要写「订阅本机 A 话题 → 送到对端 B 话题」即可，planner 完全无感。
3. **消息已经按无线场景优化**：用 MINCO 参数化而不是采样点（`ego_replan_fsm.cpp:72`），
   一条 `MinTraj` 只有几十~几百字节，压缩与否都不构成带宽问题。
4. **接收端已做容错**：自己发的会忽略（`:1196`）、乱序会丢（`:1251`）、
   远处的会丢（`:1277-1288`）、`swarm_traj` 按 id 自动扩容（`:1240-1249`）。
   6 台和 3 台对这段代码没有区别。

> **实验室已有先例可以直接对照**：
> `~/EGO-Planner-v2/swarm-playground/main_ws/src/Multibotnet/config/race3/drone_{0,1,2}.yaml`
> 是他们上次三机竞赛用的 Multibotnet 配置，`launch/swarm.launch:74-82` 是启动方式。
> 本包的配置就是在这套先例上按 6 机 + 真机（多主机）调整的，并**修正了两个坑**：
> ① 他们用的是 `traj_utils/MINCOTraj`，`SixBotsRace` 里已经改成 `controller_msgs/MinTraj`；
> ② 单机仿真必须错开端口，真机多主机可以同端口（本包用同端口，省掉端口算术）。

### 2.1 「方案A / 方案B」这个问法在这份代码里不成立（**别去改源码**）

看上游 EGO-Planner-v2 / EGO-Swarm 的文档时，通常会看到"接入 Multibotnet 有两个方案"：

- **方案A**：在发/收轨迹的地方直接把 ROS publish/subscribe 换成 Multibotnet 的 API；
- **方案B**：另起一个节点，订阅本机轨迹 → 通过 Multibotnet 发出去；收到他机轨迹 → 再以 ROS 话题发出来。

**在这份实验室代码里，方案B 已经存在了，而且实现者就是 Multibotnet 本身。**
`multibotnet_topic_node` 这个独立节点做的正是方案B 描述的事，而它是由 YAML 驱动的：

```yaml
send_topics:                    # 「订阅本机轨迹 → 发到网络」
  - topic: /broadcast_traj_from_planner
    message_type: controller_msgs/MinTraj
    port: 4001
recv_topics:                    # 「收到他机轨迹 → 以 ROS 话题发布，供原逻辑订阅」
  - topic: /broadcast_traj_to_planner
    message_type: controller_msgs/MinTraj
    connect_address: drone3
    port: 4001
```

原因是这段代码的收发**已经被「话题名」这个接缝彻底解耦**：`ego_replan_fsm.cpp:73-74`
只认那两个话题名，它不知道也不关心另一头是谁（原本预留的是 `swarm_bridge`，
现在是 Multibotnet）。⇒ **接入工作量 = 0 行源码改动，只需要 YAML 配置。**

反过来，去改 `ego_replan_fsm.cpp` 调 Multibotnet API（方案A）是把已经解耦好的设计重新耦合，
得不偿失。

#### 两个容易搞混的点（上游资料会误导你）

| 上游常见说法 | 这份代码的实际情况 |
|---|---|
| 通讯模块叫 `swarm_bridge` | **仓库里没有这个包。** 只有 7 处 `<include file="$(find swarm_bridge)/launch/bridge_udp.launch">`，全在 `swarm.launch` / `circle_exchange.launch` / `multi_drone_interactive.launch` 等**上游遗留 demo** 里，现在一跑就报"找不到包"。实验室已经把它删了，把搬砖工的位置留给了 Multibotnet |
| 广播话题叫 `/broadcast_bspline` | **没有这个话题**（全仓库 0 处）。真实话题是 `/broadcast_traj_from_planner` / `/broadcast_traj_to_planner`，类型 `controller_msgs/MinTraj`（**MINCO 参数化**，不是 B 样条；`ego_replan_fsm.cpp:72` 注明了是为省无线带宽）。另外 `planning/trajectory`（`traj_utils/PolyTraj`）是**本机内部**给 `traj_server` 的，不参与多机通讯 |
| 广播话题带 `drone_id` 后缀 | **不带**。`drone_id` 在消息体内的 `MinTraj.drone_id` 字段，接收端按它过滤（`:1195-1197`）。带后缀是单机仿真才需要的做法，见 §6 |

#### 那什么情况下才真的要改代码？

| 需求 | 要改 C++ 吗 | 正确做法 |
|---|---|---|
| 6 机真机通讯 | **不要** | 只用 YAML（本包已给） |
| 6 机**单机仿真** | **不要** | 改 launch 的 remap，给每机加话题前缀 —— 见 §6；实验室自己就是这么做的（`launch/include/advanced_param.xml:44`） |
| 额外传点东西（状态、任务分配指令） | **不要** | 在 Multibotnet 配置里**新增**一组 send/recv 话题，planner 无感（本包的 `/race6/comm_test`、`/mbn/*_odom` 就是这么加的） |
| 收到别机轨迹后要做**额外决策**（例如投弹同步、任务分配） | **要**（但这是**功能开发**，不是"接入 Multibotnet"） | 在 FSM 里加逻辑；那属于下一个任务，本包不含 |
| 时间同步 | **不要** | 操作系统层用 chrony（§3）+ 本包的 `race6_time_sync.py` 验证 |

#### 怎么证明实验室代码真的一行没动

```bash
git -C ~/SixBotsRace status --porcelain -- ego-planner2 MLMapping-Embedded_version FAST_LIO
#   必须输出为空
```

`setup_six_ws.sh` 的「步骤 3.5 守卫」每次搭工作区都会替你跑这一条。

---

## 3. 时间同步（本次唯一需要"新写逻辑"的地方）

### 3.1 为什么必须做——代码里的硬判据

```cpp
// ego_replan_fsm.cpp:1222-1237
ros::Time t_now = ros::Time::now();                       // 本机墙上时钟
if (abs((t_now - msg->start_time).toSec()) > 0.25) {      // 0.25 s
  if (abs((t_now - msg->start_time).toSec()) < 10.0) {
    ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs", ...);   // 只告警，轨迹照收
  } else {
    ROS_ERROR("... swarm time seems not synchronized, abandon!");      // ★ 直接丢弃
    return;
  }
}
```

而 `msg->start_time` 就是发送方规划完成那一刻的墙上时钟（`planner_manager.cpp:399` → `:1359`）。
所以这个差值 ≈ **两机系统时钟差 + 网络/处理延迟**。

| 两机时钟差 | 后果 |
|---|---|
| < 10 ms | 理想。协同避碰的相位预测才真正准 |
| 50 ms | 本脚本判 WARN；1.5 m/s 下别人位置预测错 7.5 cm |
| > 250 ms | `ego_planner` 开始刷 `Time stamp diff` 告警 |
| > 10 s | **轨迹被直接丢弃**，等于通讯断了 |

### 3.2 两层方案

> **完整的分步操作、`chronyc tracking` 怎么读、三重验证、装不上 chrony 的兜底、以及全部坑，
> 见 [`真机双机部署步骤.md`](真机双机部署步骤.md) 阶段 3。** 这里只给最小可用版。

**第一层：操作系统时钟对齐（必须做，这是根本）**

用 chrony，指定**物理 1 号机（drone_id 0）**当时间源，其余当客户端。每台飞机各执行：

```bash
# ---- 源机：物理 1 号机（192.168.66.101）----
sudo apt install -y chrony
# 关掉可能抢 NTP 的 systemd-timesyncd（两个 NTP 客户端会互相打架）
sudo systemctl disable --now systemd-timesyncd 2>/dev/null || true
sudo cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.bak
sudo tee /etc/chrony/chrony.conf >/dev/null <<'EOF'
driftfile /var/lib/chrony/chrony.drift
allow 192.168.66.0/24        # ← 你们飞机所在网段（六机都是 192.168.66.x）
local stratum 8              # 没有外网时，自己当权威时钟
makestep 0.1 3
rtcsync
EOF
sudo systemctl restart chrony
sudo systemctl enable  chrony   # ★ 开机自启（不设这行，断电重启后就不会自动对时）
chronyc tracking                # 看 "System time" 与 "RMS offset"

# ---- 其余五台（客户端）----
sudo apt install -y chrony
sudo systemctl disable --now systemd-timesyncd 2>/dev/null || true
sudo cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.bak
sudo tee /etc/chrony/chrony.conf >/dev/null <<'EOF'
driftfile /var/lib/chrony/chrony.drift
server 192.168.66.101 iburst    # ← 源机（物理 1 号机 / drone_id 0）的 IP
makestep 0.1 3
rtcsync
EOF
sudo systemctl restart chrony
sudo systemctl enable  chrony   # ★ 同上
sleep 20
chronyc sources -v              # 期望源机那一行前面是 ^*，Reach 攒到 377
chronyc tracking                # System time / RMS offset 应在毫秒级（这就是两机时钟差）
```

> ⚠️ 如果现场没有路由器/NTP 服务器，就靠源机的 `local stratum 8` 自建。
> 关键是**六台必须互相同步到同一个源**，而不是各自跟互联网对表。
>
> ⚠️ **想让源机的绝对时间也准**（bag 时间戳、日志），在它的 `chrony.conf` 里加一行
> `pool ntp.aliyun.com iburst`：有外网时就用互联网时间，没外网时自动退回 `local stratum 8`。
> 不加也能用 —— 六台一致就够了，FSM 只比较两机之差。
>
> ⚠️ 每次上电（换电池/重启）后，用这三条确认一次：
> `systemctl is-active chrony` → `chronyc tracking`（看 `System time`）→ 跑一遍
> `race6_time_sync.py`（实测 offset，这才是最终证据）。
>
> ★ **上面是"单源"的最小配置，正式比赛前建议先看**
> [`比赛日启动清单.md` §1.1-1.3](比赛日启动清单.md)：**源机（物理 1 号机）挂了会怎样、三层预案**。
> 一句话结论：chrony 失去源后**不会崩**，只是停止校时，各台按晶振缓慢漂移
> （约 2.4 ms/分钟）—— 几分钟内完全无感，但 1~2 小时后会逼近 FSM 的 250 ms 判据。
> 所以比赛时的**首选是把时间源放在地面站 PC**（不会摔、能接外网），而不是飞机上。

**第二层：Multibotnet 链路上的实测与校时（本包提供的脚本）**

`scripts/race6_time_sync.py`——用 NTP 四时戳法，在**真实报文路径上**测两机时钟差：

```
REQ : [1, seq, t1, origin_id]                  ← 本机 _tx 发出
RESP: [2, seq, t1, t2, t3, origin_id, responder_id]  ← 对端 _rx 收到后立刻回
t1 请求方发出（请求方钟）   t2 应答方收到（应答方钟）
t3 应答方发出（应答方钟）   t4 请求方收到（请求方钟）

offset θ = ((t2 − t1) + (t3 − t4)) / 2      ← 对端钟 − 本机钟，正是上面的判据差值
rtt      = (t4 − t1) − (t3 − t2)
```

默认**只测量不动钟**；加 `--apply` 才校时（`chronyc makestep` 优先，否则 `sudo date -s`）。
**飞行中绝对不要用 `--apply`**：步进系统时钟会让所有 ROS 节点的时间跳变。

为什么不用 `rostopic delay` / `topic_tools delay` 那类现成工具？
因为那些只测「消息在 ROS 里的延迟」，测不出**两机系统时钟差**——而 FSM 判据要的正是后者。

---

## 4. 本包新增的文件（以及"没动"的文件）

**代码放在仓库里**（`~/SixBotsRace/race6_comm/`）。`~/six_ws` 只是按需生成、随时可删可重建的
catkin 工作区 —— **仓库才是唯一真源**。完整的仓库/分支/版本管理约定见
`~/SixBotsRace/PROJECT.md`。

> 如果你还没做那次搬家（`race6_comm` 仍在 `~/six_ws/src/race6_comm/`），
> 把本文档里的 `~/SixBotsRace/race6_comm/...` 统统换成 `~/six_ws/src/race6_comm/...`
> 即可，两条路径下文件内容完全一样。建议尽快搬（`PROJECT.md` §2「一次搬家」）。

```
~/SixBotsRace/race6_comm/                     ← 本次全部新增内容都在这里（仓库真源）
├── package.xml
├── CMakeLists.txt
├── README.md                                 ← 本文件（模块原理 / 排查表 / 验收清单）
├── 真机双机部署步骤.md                        ← ★ 上真机的分步操作手册（手把手，从拷代码开始）
├── config/
│   ├── ground2/drone_0.yaml                  ← 地面双机：0 号机
│   ├── ground2/drone_1.yaml                  ← 地面双机：1 号机
│   ├── race6/drone_0.yaml ~ drone_5.yaml     ← 6 机竞赛（第 1 阶段只改 IP）
│   └── selftest/loopback.yaml                ← PC 单机回环自测
├── launch/
│   ├── multibotnet_node.launch               ← 只起通信节点（配 planner launch 用）
│   ├── comm_ground_test.launch               ← 地面双机：通信+时间同步+收发验证 一键启动
│   └── comm_selftest.launch                  ← PC 单机自测（不碰飞机）
└── scripts/
    ├── setup_six_ws.sh                       ← 组装 ~/six_ws 并编译（含"实验室代码零改动"守卫）
    ├── gen_race6_configs.py                  ← 改 IP 后一条命令重生成 6 份配置
    ├── race6_time_sync.py                    ← ★ 时间同步脚本（核心）
    ├── race6_comm_test.py                    ← 通用链路验证（方向/丢包/延迟/内容）
    └── race6_mintraj_test.py                 ← 竞赛主通道验证（MinTraj + FSM 判据）
```

**没有改动任何实验室文件**（`ego_planner` / `MLMapping` / `FAST_LIO` 一行未动，
本包对它们只是只读引用）。工作区搭建脚本默认用**软链**把这些包放进 `~/six_ws/src/`
（软链指向仓库真源，改完立刻生效；`--copy` 才是真复制）。

---

## 5. 地面双机联调（本阶段全部工作）

> **★ 桨叶全部卸下，飞机不上电到电机。★** 本阶段只验证通讯与时间同步，不涉及飞行。
>
> **要上真机，直接看 [`真机双机部署步骤.md`](真机双机部署步骤.md)** ——
> 那里是从「把 `race6_comm` 搬进仓库 + 内置 Multibotnet」→「两台飞机验收全打勾」的
> 分步操作（含 chrony 时钟同步、离线 git bundle 部署、里程计通道验证）。
> 本节保留原理、预期输出样例和排查表。

### 5.1 在 PC 上先做单机自测（强烈建议，10 分钟，能挡掉大部分低级问题）

这一步不需要飞机：一个 Multibotnet 节点把报文绕回自己，再起两个
`self_id=0/1` 的脚本实例，等价于两台飞机互相收发。

```bash
# 1) 组装工作区（minimal 模式：只要 multibotnet + controller_msgs + race6_comm）
bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal

# 2) 生效
source /opt/ros/noetic/setup.bash
source ~/six_ws/devel/setup.bash
rospack profile
```

**预期输出（节选）**：

```
[ OK ] ROS 环境已加载（noetic）
[ OK ] multibotnet 已就绪          ← 依赖检查（libzmq3-dev / yaml-cpp / lz4 ...）
[ OK ] libzmq3-dev 已就绪
...
[ OK ] 软链 multibotnet  ->  /home/zjy/chen_ws2/src/multibotnet
[ OK ] 软链 race6_comm  ->  /home/zjy/SixBotsRace/race6_comm   ← 软链到仓库真源
[ OK ] 软链 controller_msgs（MinTraj）
[ OK ] 实验室代码干净（这三个子目录没有未提交改动）    ← 步骤 3.5 守卫
[ OK ] 当前仓库 commit：205f1ad (feature/zhongjianyi/multi-comm)
[ OK ] 编译成功
[ OK ] multibotnet  ->  /home/zjy/six_ws/src/multibotnet
[ OK ] controller_msgs  ->  /home/zjy/six_ws/src/planner/controller_msgs
[ OK ] race6_comm  ->  /home/zjy/SixBotsRace/race6_comm
[ OK ] 工作区就绪：/home/zjy/six_ws
```

```bash
# 3) 单机自测（约 25 秒后测试节点自动出结论）
source ~/six_ws/devel/setup.bash
roslaunch race6_comm comm_selftest.launch
```

**预期输出**：

```
 __  __       _ _   _ _           _   _   _      _
|  \/  |_   _| | |_(_) |__   ___ | |_| \ | | ___| |_
...
            Topic Communication Node v4.1.2

============ Topic Node Configuration ============
  Compression:
    Enabled: false
...
  Send Topics (3):
  /broadcast_traj_from_planner [20Hz] -> <本机IP>:4001
  /race6/timesync_tx [20Hz] -> <本机IP>:4201
  /race6/comm_test_tx [20Hz] -> <本机IP>:4401
  Receive Topics (3):
  /broadcast_traj_to_planner <- localhost:4001
  /race6/timesync_rx <- localhost:4201
  /race6/comm_test_rx <- localhost:4401
=================================================
2026-02-14 10:00:00.123 [INFO] Waiting for ZMQ subscriptions to take effect...
2026-02-14 10:00:00.723 [INFO] TopicManager started successfully
2026-02-14 10:00:00.824 [INFO] Topic '/race6/timesync_rx' [std_msgs/Float64MultiArray] receiving data from network
2026-02-14 10:00:00.830 [INFO] Topic '/race6/comm_test_rx' [std_msgs/Float64MultiArray] receiving data from network
2026-02-14 10:00:00.840 [INFO] Topic '/broadcast_traj_to_planner' [controller_msgs/MinTraj] receiving data from network
```

脚本侧（**这一段就是判据**）：

```
[race6_time_sync] 本机 drone_0  已运行 5.0 s
对端        应答   本机发   offset均值   offset抖动  |offset|峰值    rtt均值   rtt峰值  判定
------------------------------------------------------------------------------------------------
drone_1        5        5      0.02 ms      0.01 ms      0.03 ms    0.31 ms   0.55 ms  PASS
------------------------------------------------------------------------------------------------
  说明：offset = 对端钟 - 本机钟（正是 ego_replan_fsm.cpp:1223 判据里的差值）
  本轮总判定：PASS

[race6_comm_test] 最终结论  本机 drone_0  已运行 25.0 s  本机已发 125 帧 (5.00 Hz)
对端        收到     应到   丢包率  内容损坏   延迟均值   延迟最小   延迟最大   负延迟    判定
drone_1     125      125   0.00%        0     0.28 ms    0.15 ms    1.20 ms       0   PASS
  本轮判定：PASS ✅

[race6_mintraj_test] 最终结论  本机 drone_0  已运行 25.0 s  本机已发 50 条 MinTraj
对端        收到     应到   丢包率     dt 均值     dt 最小     dt 最大   |dt|峰值  判定
drone_1      50       50   0.00%     0.31 ms    0.12 ms    1.05 ms   1.05 ms  PASS
  dt = 本机 ros::Time::now() - msg.start_time  ← 正是 ego_replan_fsm.cpp:1223 的判据
  本轮判定：PASS ✅（竞赛主通道已通）
```

> 单机上 offset 天然 ≈ 0（同一个钟），**所以这一步只证明"脚本和链路对"，
> 不能证明"时钟对齐"**。时钟是否真对齐，必须到两台真机上测。

### 5.2 地面双机（两台真机，桨叶卸下）

**(a) 组网与查 IP**

```bash
# 两台飞机接同一个路由器/热点，然后在【每一台】上：
ip -4 addr show | grep -w inet
#   期望：能看到同一网段的地址，例如 192.168.152.101 / 192.168.152.102

ping -c 3 192.168.152.102          # 在 0 号机上 ping 1 号机，应 0% packet loss
```

**(b) 把真实 IP 填进配置**（**这一步不做，后面一定收不到**）

```bash
# 用生成器一次重写全部配置 —— 它会同时写 config/race6/ 和 config/ground2/，
# 并且带着"物理编号 → 软件 drone_id"的对照表打印出来
cd ~/SixBotsRace/race6_comm
python3 scripts/gen_race6_configs.py \
    --ips <物理1IP>,<物理2IP>,<物理3IP>,<物理4IP>,<物理5IP>,<物理6IP>
python3 scripts/check_config.py --all       # 必须全绿（含跨文件 IP 一致性）
```

> ⚠️ **不要手改单个文件**：同一个别名在 8 份配置里都要一致，手改必漏。
> 生成器会保证 race6/（6 份）与 ground2/（2 份）永远一致。

**(c) 两台都装好工作区。** 推荐用 git（能保证 PC 和两台飞机是**同一个 commit**，见
`PROJECT.md` §6）：

```bash
# 在【每台飞机】上执行一次（假设能访问 GitHub）：
cd ~
git clone -b feature/zhongjianyi/multi-comm git@github.com:JianyiZhong/SixBotsRace.git
bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal
source ~/six_ws/devel/setup.bash

# 现场没网 / 没配 SSH key 时，用 git bundle 离线传（PC 上先生成）：
#   git -C ~/SixBotsRace bundle create /tmp/sixbots.bundle feature/zhongjianyi/multi-comm
#   scp /tmp/sixbots.bundle orangepi@192.168.152.102:~/
#   飞机上： cd ~ && git clone -b feature/zhongjianyi/multi-comm sixbots.bundle SixBotsRace
#            bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal

# 临时应急也可以直接拷包（不推荐：以后改代码不会同步过去，容易忘）：
#   scp -r ~/SixBotsRace/race6_comm orangepi@192.168.152.102:~/six_ws/src/
#   cd ~/six_ws && catkin_make && source devel/setup.bash
```

**部署后务必核对两边版本一致**（这是"PC 上跑得通、飞机上跑不通"的头号原因）：

```bash
git -C ~/SixBotsRace rev-parse HEAD            # 两台飞机输出必须完全相同
git -C ~/SixBotsRace status --porcelain -- ego-planner2 MLMapping-Embedded_version FAST_LIO
#   必须为空（实验室代码零改动）
```

**(d) 时钟同步**：按 §3.2 第一层配好 chrony，两边都执行 `chronyc tracking` 确认
`Last offset` 在毫秒级。

**(e) 两台各起一份（两个终端，各在自己的飞机上）**

```bash
# 0 号机（SSH 到 192.168.152.101）
cd ~/six_ws && source devel/setup.bash
roslaunch race6_comm comm_ground_test.launch drone_id:=0 peers:=1

# 1 号机（SSH 到 192.168.152.102）
cd ~/six_ws && source devel/setup.bash
roslaunch race6_comm comm_ground_test.launch drone_id:=1 peers:=0
```

> 也可以用脚本（自动算 `drone_id` / `peers`，并校验本机 IP 与配置一致）：
> ```bash
> bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh 1 --pair=2   # 物理 1 号机
> bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh 2 --pair=1   # 物理 2 号机
> ```

**(f) ★ 正式比赛前的双机 / 三机测试：起通信要和比赛一样**

上面 (e) 是**地面联调**，它带的 `race6_mintraj_test` 会往 `/broadcast_traj_from_planner`
发**假轨迹**，对端 FSM 会当成真实轨迹参与避碰 —— 联调可以，**带桨/带飞绝对不行**。

赛前测试的正确起法是加 `--race`（`--race` 的通信节点与比赛日**完全相同**，
只是多留一个只读的 `race6_time_sync` 当"对端有没有来"的体温计）：

```bash
# 双机（物理 1 ↔ 物理 3），两台各一个终端：
bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh 1 --race --pair=3
bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh 3 --race --pair=1

# 三机（物理 1、2、3）：
bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh 1 --race --peers=1,2
bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh 2 --race --peers=0,2
bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh 3 --race --peers=0,1

# 正式比赛（纯通信，一个测试节点都不起）：
bash ~/SixBotsRace/race6_comm/scripts/race6_launch.sh <物理编号> --race
```

| 起了什么 | 地面联调（不加 `--race`） | 赛前双机/三机（`--race --pair/--peers`） | 比赛（`--race`） |
|---|---|---|---|
| Multibotnet | 起 | 起（**与比赛同一份**） | 起 |
| `race6_time_sync` | 起 | 起（**只测量**） | 不起 |
| `race6_comm_test` | 起 | 不起 | 不起 |
| `race6_mintraj_test` | ⚠️ **起，会发假轨迹** | 不起 ★ | 不起 |

> 判断"对端到底通没通"只看 `rostopic hz /race6/timesync_rx`：**一直在动**才是通的。
> **别用 `rostopic echo`**：Multibotnet 的接收发布器是 latched 的
> （`message_factory.cpp:88` 的 `advertise(..., true)`），对端早就停了它也照样能 echo 出内容（坑 1.6）。

### 5.3 期望结果（逐项对照，这就是"通过"的定义）

| # | 检查项 | 在哪儿看 | 期望 |
|---|---|---|---|
| 1 | Multibotnet 起来了，配置读对了 | 两边终端开头 | 打印配置表，`Send Topics (4)` / `Receive Topics (4)`，IP 是你填的真实 IP |
| 2 | **链路真的收到对端数据** | Multibotnet 日志 | 最多四条 `Topic '...' [...] receiving data from network`（`_odom` 那条只有 mavros 在发里程计时才出现） |
| 3 | 双向统计在涨 | 每 5 秒的 `Topic Statistics` | `send:/race6/comm_test_tx: Messages: sent=N`；`recv:/race6/comm_test_rx: Messages: recv=M`，M 也持续增长 |
| 4 | **时钟差达标** | `race6_time_sync` 表 | `|offset|` 峰值 **< 50 ms**（chrony 配好时通常 < 5 ms），`rtt` < 10 ms，判定 **PASS** |
| 5 | 通用链路 | `race6_comm_test` 表 | 丢包 **0%**、内容损坏 **0**、延迟均值 < 50 ms、负延迟 0，判定 **PASS** |
| 6 | **竞赛主通道（最关键）** | `race6_mintraj_test` 表 | 丢包 **0%**、`|dt|` 峰值 **< 250 ms**，判定 **PASS ✅（竞赛主通道已通）** |

**第 6 项 PASS 的实际含义**：把实验室的 `ego_planner` 接上来之后，
`/broadcast_traj_from_planner` → Multibotnet → `/broadcast_traj_to_planner`
这一整条链路（**包括消息类型、md5、话题名、时间戳**）已经验证通了，
也就是"多机通讯模块"这件事本身完成了。

**里程计通道（真机独有，第 7 项）**：转发的是真实的 `nav_msgs/Odometry`（约 300 字节），
比合成报文更有说服力：

```bash
# 0 号机上：本机位置
rostopic echo -n1 /mavros/local_position/odom/pose/pose/position
# 1 号机上：从网络收到的 0 号机位置 —— 两边的 x/y/z 应基本一致（差几厘米以内）
rostopic echo -n1 /mbn/drone0_odom/pose/pose/position
rostopic hz   /mbn/drone0_odom          # 期望 ≈ 20 Hz
rostopic info /mbn/drone0_odom          # Publishers 里应有 /multibotnet_drone1
```

> ⚠️ 里程计**不参与协同避碰**：`ego_planner` 只订阅自己的 `odom_world`，
> 预测别机位置用的是别机的 `MinTraj`（`ego_replan_fsm.cpp:546-550`）。
> 转发 odom 是给地面站/RViz 监控用的（`/mbn/droneN_odom`）。别把避碰的安全性寄托在它上面。

另外可以用两条命令直接肉眼确认：

```bash
# 在 0 号机上：本机自己发的频率
rostopic hz /race6/comm_test_tx        # 期望 ≈ 5 Hz

# 在 1 号机上：应该能看到 0 号机发来的报文（drone_id=0 的那条轨迹）
rostopic echo -n 1 /broadcast_traj_to_planner
#   期望：drone_id: 0，order: 5，duration 长度 = inner_x 长度 + 1

# 在 1 号机上：确认 Multibotnet 真的把话题发布出来了
rostopic info /broadcast_traj_to_planner
#   期望 Publishers 里有 /multibotnet_drone1
```

### 5.4 出问题时的排查顺序（照这个顺序查，别跳）

| 现象 | 最可能的原因 | 怎么确认/修 |
|---|---|---|
| 完全收不到，Multibotnet 日志里**没有** `receiving data from network` | ①IP 没换 ②防火墙 ③对端没起 | `ping` 对端；`sudo ufw allow 4001 && sudo ufw allow 4201 && sudo ufw allow 4401`；`netstat -tlnp \| grep -E '4001\|4201\|4401'` |
| Multibotnet 起来就退出，`Failed to load config` | YAML 少了必填键 | `send_topics` 必须有 `max_frequency` **和** `bind_address`；`recv_topics` 必须有 `connect_address`（`config_parser.cpp:191-197`，缺一个就抛异常） |
| 有 `receiving data from network`，但脚本报"还没收到" | 话题名对不上 | `rostopic list \| grep race6` 看是否同时有 `_tx` 和 `_rx`；别把脚本里的话题名改成配置里的 |
| 收到 MinTraj 但 `dt` 是个恒定大数 | **系统时钟没对齐** | `chronyc tracking`；先解决时钟，别去调网络 |
| `race6_comm_test` 报大量负延迟 | 同上（时钟差 > 延迟本身） | 先跑 `race6_time_sync.py`，再看是否变正 |
| 丢包率高但 rtt 小 | 无线链路质量 | 换信道/靠近；把 `compression` 关掉；把 `max_frequency` 降下来 |
| 一开就网络打满 / CPU 100% | **发和收用了同一个话题名** → 广播风暴 | 严格保持 `_tx` / `_rx` 分开，见 §7 坑 1 |
| 脚本报 `无法 import controller_msgs` | `controller_msgs` 没编或没 source | `rospack find controller_msgs`；`cd ~/six_ws && catkin_make && source devel/setup.bash` |
| **`rostopic echo` 每次都收到东西，以为"一直在收"** | **Multibotnet 的接收发布者是 latched（锁存）**：`message_factory.cpp:88` 的 `advertise(..., true)`。新订阅者一连上就会拿到"最后一条"，哪怕几小时没有新消息 | 判断"当前在不在收"**只能用 `rostopic hz`**；`echo` 只适合看内容。若两次 `echo -n1` 拿到的 `traj_id`/`nsecs` 完全一样，就是锁存重放 |
| **统计里 `recv=0` 但数据其实在收** | （已修）旧版 `getStatistics()` 用**话题名**做 key，而多个对端会发布到同一个本地话题名 → 5 个 transport 互相覆盖只留最后一个 | 见 §7 坑 1.6；已改为按 `话题名 <- 对端地址:端口` 分行 |

---

### 5.5 地面通过之后的一小步：悬停状态下复测（**本阶段不做，留作下一步**）

原本的计划里还有"两台飞机原地起飞悬停 + 通信 + 时间同步"。按当前决定，
**这一步先不做**，等 §5.3 六项全部 PASS 之后再单独推进。届时要点：

1. **起飞悬停不是本包的事**，由实验室既有的 `px4Controller` 负责
   （`chen_ws2/src/px4Controller/src/px4ctrl/launch/run_ctrl.launch`
   会起 `px4ctrl_node` 并自动执行 `takeoff.sh`，后者往
   `/px4ctrl/takeoff_land` 发 `takeoff_land_cmd: 1`）。
   本包只在飞机稳定悬停之后，把 §5.2(e) 那条 `comm_ground_test.launch` 再跑一遍。
   > ⚠️ 但如果是**带桨/带飞**状态下复测，不要用 (e)，用 §5.2(f) 的
   > `race6_launch.sh <编号> --race --peers=<对端>` —— 它和比赛日同一套通信节点，
   > 不会发假轨迹。真要让 `race6_comm_test` / `race6_mintraj_test` 上飞机测，
   > 必须先把桨卸掉。
2. **顺序**：先让两台的 Multibotnet + 时间同步在地面跑起来并 PASS，
   再起飞；起飞后重跑 `race6_comm_test` / `race6_mintraj_test` 看指标是否劣化
   （**只能在卸桨状态下**；带飞时用 §5.2(f) 的 `rostopic hz /race6/timesync_rx` 代替）。
3. **要重点观察的两件事**：
   - 悬停时 CPU 被建图/控制占满，Multibotnet 的 `Topic Statistics` 里
     `msg/s` 是否还稳（掉到预期的一半以下说明线程被饿死，
     可以把 `advanced.thread_pool.size` 调回 2 并降低 `max_frequency`）；
   - `race6_time_sync` 的 `rtt` 峰值是否明显变大（WiFi 抖动/天线遮挡）。
4. **红线**：飞行中**绝不**使用 `race6_time_sync.py --apply`。
   步进系统时钟会让所有 ROS 节点的时间跳变，直接毁掉正在执行的轨迹。
   时钟必须在起飞前就对齐好。

---

## 6. 第二阶段（**本次不做**，列出来是为了划清边界）

以下都不在"多机通讯模块"范围内，等通讯验收通过再单独推进：

1. **6 机竞赛联调**：把 `config/race6/*.yaml` 的 IP 换成真实 IP
   （`python3 scripts/gen_race6_configs.py --ips <6个IP>`），
   每台用 `roslaunch race6_comm multibotnet_node.launch drone_id:=N`
   与实验室的 planner launch 一起起。
2. **6 机单机仿真**：必须把轨迹话题改成**每机独立**（同一台主机不能用同一个话题名），
   照抄实验室自己的做法——把 `advanced_param_swarm.xml` 的两行 remap 改成：
   ```xml
   <remap from="~planning/broadcast_traj_send" to = "/drone_$(arg drone_id)_broadcast_traj_from_planner"/>
   <remap from="~planning/broadcast_traj_recv" to = "/broadcast_traj_to_planner"/>
   ```
   （对照 `~/EGO-Planner-v2/swarm-playground/main_ws/src/planner/plan_manage/launch/include/advanced_param.xml:43-45`）
   同时 Multibotnet 配置要改成**端口错开**（4001+id / 4201+id / 4401+id），
   因为同一台主机上同端口会 `Failed to bind`。
3. **投弹（任务点投弹）**：需要新增一个舵机/投放控制节点 + 到达目标点后的触发逻辑，
   与通讯无关，本包不含。
4. **任务分配**：3 组分别去 3 个任务点、往返航点，属于 FSM waypoint 配置和上层调度。
5. **上机前必须检查的一个雷**：`advanced_param_swarm.xml:42` 有
   ```xml
   <rosparam file="/opt/datafs/config/swarm_param.yaml" command="load" />
   ```
   这是**绝对路径**。机上没有这个文件时 roslaunch 会直接报错退出。
   上机前先确认它存在，或把它改成包内相对路径。

---

## 7. 已知坑清单（都是踩过的）

**坑 1（最严重）：Multibotnet 的发送话题和接收话题绝不能同名。**
发送话题是「本地订阅」、接收话题是「本地发布」（`topic_manager.cpp:411-420` / `717-733`）。
同名 ⇒ 收进来的报文被发送话题再订阅一次、又广播出去 ⇒ **报文自我复制，广播风暴**。
实验室代码把 `/broadcast_traj_from_planner` 与 `/broadcast_traj_to_planner`
分成两个名字，就是为了这个。本包四条通道全部遵守：

| 通道 | 发（本机订阅） | 收（本机发布） |
|---|---|---|
| 轨迹规划 | `/broadcast_traj_from_planner` | `/broadcast_traj_to_planner` |
| 里程计 | `/mavros/local_position/odom` | `/mbn/droneN_odom` ← 前缀 `mbn` 就是干这个用的 |
| 时间同步 | `/race6/timesync_tx` | `/race6/timesync_rx` |
| 链路测试 | `/race6/comm_test_tx` | `/race6/comm_test_rx` |

> 想把收到的别机里程计「再转发给第三台」时最容易犯这个错：**永远不要**把 recv 的
> topic 名写成 `/mavros/local_position/odom`，那样它会被本机的发送话题再广播出去。

**坑 1.5（静默失败，比坑 1 更难查）：`connect_address` 用了 `IP:` 段里没定义的别名。**

```yaml
IP:
  drone3: '192.168.66.85'
  drone5: '192.168.66.161'      # ← 没有 drone0
recv_topics:
  - topic: /broadcast_traj_to_planner
    connect_address: drone0      # ← 这里写 drone0
    port: 4001
```

`resolveAddress()` 找不到键时**原样返回字符串**（`topic_manager.cpp:341-352`），
于是连接地址变成 `tcp://drone0:4001` —— ZMQ 把它当**主机名**做 DNS 解析，
解析失败就永远连不上，**而 Multibotnet 一个错都不报**。
症状：`send` 侧计数正常增长、`recv` 侧全是 0、日志里既没有报错也没有
`receiving data from network`。

**判别口诀：配置表里 `Receive Topics` 的 `<-` 后面必须是 IP。
显示成一个名字（如 `<- drone0:4001`）就是别名没定义。**
（发送侧不受影响，因为 `bind_address: self` 会被特殊处理成本机 IP。）

上飞机前跑一次自检，这个坑和其他几类配置错误都能一次查出来：

```bash
rosrun race6_comm check_config.py --all
# 或在工作区外： python3 ~/SixBotsRace/race6_comm/scripts/check_config.py --all
```

它会检查：别名是否都定义了、必填键是否齐全、`message_type` 是否用了斜杠、
端口是否重复绑定、**发/收是否同名**（坑 1），并把每个 `recv` 真正会用的
`tcp://地址:端口` 打印出来。

**坑 1.6（会骗人的诊断）：Multibotnet 的接收发布者是 latched，`rostopic echo` 不能用来判断"在不在收"。**

```cpp
// message_factory.cpp:88
ros::Publisher pub = msg->advertise(nh_, topic, queue_size, true);   // 第 4 参 = latch
```

后果：**任何新订阅者一连上，立刻收到"最后一条"消息**（哪怕已经几小时没有新数据）。
所以 `rostopic echo -n1 /broadcast_traj_to_planner` **永远能返回东西**，
很容易误判成"一直在收"。识别方法：连着 echo 两次，如果 `traj_id` / `nsecs`
**完全一样**，就是锁存重放，不是新数据。

| 想看什么 | 用什么 |
|---|---|
| 消息**内容**对不对（字段、drone_id） | `rostopic echo`（会被 latch 重放，但不影响看内容） |
| **当前有没有在收**（活性/频率） | **`rostopic hz`**（唯一可靠） |
| 各对端分别收了多少（累计） | Multibotnet 的 `Topic Statistics`（**已修**：按 `话题名 <- 对端:端口` 分行） |

> 旧版 `getStatistics()` 用话题名做 key，而**多个对端会发布到同一个本地话题名**
> （5 个对端 → 5 个 transport → 一个 key）→ 互相覆盖只留最后一个，
> 表现为 `recv=0` 的假象。已在 `topic_manager.cpp` 修成按对端分行。

**坑 2：`MinTraj` 的消息类型要写 `controller_msgs/MinTraj`（斜杠）。**
两端都必须编译并 source 了 `controller_msgs`，因为 md5sum 随消息过网络
（`topic_manager.cpp:455`）。一端没有这个包，必然收不到，而且报错不明显。

**坑 3：YAML 必填键。** `send_topics` 缺 `max_frequency` 或 `bind_address`、
`recv_topics` 缺 `connect_address`，都会让节点启动即失败（`config_parser.cpp:191-197`）。

**坑 4：时钟差是"看不见的杀手"。** 它不会让链路报错，
只会让 `|dt|` 变成几百毫秒或几秒，然后在 FSM 里静默丢帧。
地面测试一定要看 `dt` 列，而不是只看"收到了"。

**坑 5：起跑顺序是 id 序的。** `ego_replan_fsm.cpp:171` 要求
`drone_id<=0` 或 `have_recv_pre_agent_` 才能出 `SEQUENTIAL_START`。
而 `have_recv_pre_agent_` 是在 `:1306-1320` 里由**收到别机轨迹**触发的
（注意那段代码把赋值写在循环体内，所以实际效果是"只要收到 0 号机的轨迹就会被置真"）。
实务结论：**通讯没通，后面的飞机永远不会起飞**；起跑时先让 0 号机广播起来。

**坑 6：单机多机仿真必须错开端口。** 同一台主机上两个节点绑同一个端口会
`Failed to bind`。真机多主机才可以同端口（本包方案）。

**坑 7：`zsh`/`bash` 里的 `:Zone.Identifier`。** 从 Windows 复制过来的文件会带
`xxx:Zone.Identifier` 垃圾文件，可能干扰 catkin。`setup_six_ws.sh` 会自动清理。

---

## 8. 验收清单（本阶段"做完"的定义）

- [ ] `bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal` 成功，
      `rospack find multibotnet/controller_msgs/race6_comm` 都能找到
- [ ] `roslaunch race6_comm comm_selftest.launch` 三个脚本都 PASS（PC 单机）
- [ ] 两台飞机 `ping` 通，配置里的真实 IP 已填、两份文件一致
- [ ] 两台 `chronyc tracking` 的 `Last offset` 在毫秒级
- [ ] 两台 `comm_ground_test.launch` 都起来，Multibotnet 打印 3 条
      `receiving data from network`
- [ ] `race6_time_sync` 判定 PASS（`|offset|` < 50 ms）
- [ ] `race6_comm_test` 判定 PASS（丢包 0%）
- [ ] **`race6_mintraj_test` 判定 PASS（`|dt|` < 250 ms）← 本次任务的验收标志**
- [ ] 全程没有改动 `ego_planner` / `MLMapping` / `FAST_LIO` 任何文件
