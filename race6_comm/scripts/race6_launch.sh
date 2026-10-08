#!/bin/bash
# =====================================================================
# race6_comm / scripts/race6_launch.sh
#
# 用【飞机物理编号 1~6】一条命令起联调，不用自己算 drone_id / peers / 配置文件名。
#
# 为什么需要它 —— 两个反复踩过的坑：
#   1) 软件 drone_id 必须从 0 开始（物理 1 号机 = drone_id 0），不能直接用 1~6。
#      否则没有任何一台的 id 是 0，六台【谁都不会起飞】
#      （原因见 config/race6/drone_0.yaml 头部，对应 ego_replan_fsm.cpp:173 / :1306-1320）。
#      于是"物理编号 N ↔ 软件编号 N-1"这个减法每次都要小心。
#   2) 把同一份配置文件拷到六台飞机上（或改了 IP 没改对）→ 静默收不到数据。
#      本脚本会拿【本机真实 IP】和配置里 drone<id> 那一行对一下，不一致直接拒绝启动。
#
# 用法：
#   bash race6_launch.sh 3 --race          # ★ 比赛日：只起通信，不带任何测试脚本
#   bash race6_launch.sh 1                 # 物理 1 号机：drone_id=0, 期望对端 1,2,3,4,5
#   bash race6_launch.sh 6                 # 物理 6 号机：drone_id=5, 期望对端 0,1,2,3,4
#   bash race6_launch.sh 3 --pair=5        # ★ 只和物理 5 号机对测（期望对端只有 drone4）
#   bash race6_launch.sh 3 --peers=0,4     # 手动指定期望对端（软件编号）
#   bash race6_launch.sh 3 --dry-run       # 只打印将要执行的命令，不启动
#   bash race6_launch.sh 1 --force         # 跳过"本机 IP 与配置不符"的检查
#   bash race6_launch.sh 1 --config-only   # 等价 --dry-run
#
# ★ --race 与不加 --race 的区别：
#     不加（联调模式）→ 起 comm_ground_test.launch：Multibotnet + 三个测试脚本，
#                        其中 race6_mintraj_test 会发【假轨迹】，只适合地面联调
#     --race（比赛模式）→ 起 multibotnet_node.launch：只起 Multibotnet，零测试节点
#
# ★ 两台对测一定要用 --pair。不带它的话脚本会期望另外 5 台都在，
#   没开机的那 4 台会被判成"完全没有收到" → 明明通了却报 FAIL。
#
# 依赖：已经 source 过工作区（rosrun / rospack 能找到 race6_comm）。
# =====================================================================
set -u

R='\033[31m'; G='\033[32m'; Y='\033[33m'; B='\033[34m'; N='\033[0m'
ok()   { echo -e "${G}[ OK ]${N} $*"; }
warn() { echo -e "${Y}[WARN]${N} $*"; }
fail() { echo -e "${R}[FAIL]${N} $*"; }

N_DRONES=6
DRY_RUN=0
FORCE=0
PAIR_PHYS=""          # --pair=N：本次只和【物理 N 号机】对测
PEERS_OVERRIDE=""     # --peers=0,1,2：直接指定期望对端（软件编号）
RACE=0                # --race：比赛模式，只起通信（不带任何测试脚本）

PHYS=""
for a in "$@"; do
  case "$a" in
    --dry-run|--config-only) DRY_RUN=1 ;;
    --force)                 FORCE=1 ;;
    --race)                  RACE=1 ;;
    --pair=*)                PAIR_PHYS="${a#--pair=}" ;;
    --peers=*)               PEERS_OVERRIDE="${a#--peers=}" ;;
    -h|--help)               sed -n '2,34p' "$0"; exit 0 ;;
    *)                       PHYS="$a" ;;
  esac
done

if [ -z "$PHYS" ]; then
  fail "用法： bash $0 <飞机物理编号 1~6> [--race] [--pair=<对测的物理编号>] [--dry-run] [--force]"
  echo "  ★ 比赛日： bash $0 3 --race          # 只起通信，不带任何测试脚本（推荐）"
  echo "  联调时  ： bash $0 3 --pair=5        # 物理 3 号机，只和物理 5 号机对测"
  echo "            bash $0 3                 # 物理 3 号机，期望其余 5 台都在"
  exit 2
fi
case "$PHYS" in
  ''|*[!0-9]*) fail "物理编号必须是数字：'$PHYS'"; exit 2 ;;
esac
if [ "$PHYS" -lt 1 ] || [ "$PHYS" -gt "$N_DRONES" ]; then
  fail "物理编号必须在 1~$N_DRONES 之间，给的是 $PHYS"
  exit 2
fi

ID=$((PHYS - 1))                      # ★ 物理编号 → 软件 drone_id

# peers（期望对端）：
#   默认 = 其余所有软件编号；
#   --pair=N  → 只有那一台（★ 两台对测必须用它，否则脚本会期望 5 个对端、
#               把没开机的 4 台判成"完全没收到"从而误报 FAIL）；
#   --peers=a,b → 直接指定。
PEERS=""
if [ -n "$PAIR_PHYS" ]; then
  case "$PAIR_PHYS" in
    ''|*[!0-9]*) fail "--pair 必须是数字：'$PAIR_PHYS'"; exit 2 ;;
  esac
  if [ "$PAIR_PHYS" -lt 1 ] || [ "$PAIR_PHYS" -gt "$N_DRONES" ] || [ "$PAIR_PHYS" = "$PHYS" ]; then
    fail "--pair=$PAIR_PHYS 不合法（必须是与本机不同的 1~$N_DRONES）"; exit 2
  fi
  PEERS=$((PAIR_PHYS - 1))
elif [ -n "$PEERS_OVERRIDE" ]; then
  PEERS="$PEERS_OVERRIDE"
else
  for i in $(seq 0 $((N_DRONES - 1))); do
    [ "$i" = "$ID" ] && continue
    if [ -z "$PEERS" ]; then PEERS="$i"; else PEERS="$PEERS,$i"; fi
  done
fi

# 找包目录（优先 rospack，其次仓库默认位置）
PKG=""
if command -v rospack >/dev/null 2>&1; then
  PKG="$(rospack find race6_comm 2>/dev/null || true)"
fi
if [ -z "$PKG" ]; then
  if [ -d "$HOME/SixBotsRace/race6_comm" ]; then
    PKG="$HOME/SixBotsRace/race6_comm"
    warn "rospack 找不到 race6_comm，退回 $PKG（建议先 source 工作区）"
  else
    fail "找不到 race6_comm：先 source 工作区（source ~/chen_ws/devel/setup.bash）"
    exit 1
  fi
fi

CONFIG="$PKG/config/race6/drone_${ID}.yaml"

echo "=============================================================="
echo " race6 六机联调启动"
echo "   飞机物理编号 : $PHYS"
echo "   软件 drone_id: $ID      ← 物理编号 - 1（必须从 0 开始）"
echo "   期望对端     : $PEERS"
echo "   配置文件     : $CONFIG"
echo "=============================================================="

if [ ! -f "$CONFIG" ]; then
  fail "配置文件不存在：$CONFIG"
  echo "     （6 份配置应该在 config/race6/drone_0.yaml ~ drone_5.yaml）"
  exit 1
fi

# ---- 1) 配置自检：别名有没有定义、必填键齐不齐、发/收有没有同名 ----
if [ -f "$PKG/scripts/check_config.py" ]; then
  if python3 "$PKG/scripts/check_config.py" "$CONFIG" --quiet; then
    ok "配置自检通过"
  else
    fail "配置自检不通过 —— 先修上面的问题（常见：connect_address 用了未定义的别名）"
    exit 1
  fi
else
  warn "没有 check_config.py，跳过配置自检"
fi

# ---- 2) 本机 IP 与配置里 drone<ID> 那一行是否一致 ----
EXPECT="$(grep -E "^[[:space:]]*drone${ID}:" "$CONFIG" | head -1 | sed -E "s/.*'([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)'.*/\1/")"
LOCAL_IPS="$( (hostname -I 2>/dev/null || true; ip -4 -o addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1) | tr ' ' '\n' | grep -v '^$' | sort -u | tr '\n' ' ')"
echo "   本机 IP      : ${LOCAL_IPS:-（取不到）}"
echo "   配置期望 IP  : ${EXPECT:-（解析失败）}"

IP_OK=0
if [ -n "$EXPECT" ]; then
  for ip in $LOCAL_IPS; do
    [ "$ip" = "$EXPECT" ] && IP_OK=1
  done
fi

if [ "$IP_OK" = "1" ]; then
  ok "本机 IP 与该配置一致"
else
  fail "本机 IP 与配置里 drone${ID} 的 IP 不一致！"
  echo "       这说明你在这台机器上用了【别的飞机】的配置（或 IP 还没填对）。"
  echo "       六份配置里的六行 IP 必须完全一致，且 drone${ID} 那行要等于本机 IP。"
  echo "       查本机所有网卡： ip -4 addr show | grep -w inet"
  echo "       确认无误可以用 --force 跳过。"
  if [ "$FORCE" != "1" ] && [ "$DRY_RUN" != "1" ]; then
    exit 1
  fi
  [ "$DRY_RUN" = "1" ] && warn "--dry-run：只提示，不阻断"
fi

# ---- 3) 启动 ----
if [ "$RACE" = "1" ]; then
  # ★ 比赛模式：只起 Multibotnet，不起任何测试脚本。
  #   为什么必须这样：comm_ground_test.launch 里的 race6_mintraj_test 会往
  #   /broadcast_traj_from_planner 发【假轨迹】，对端 FSM 会把它当成"本机的真实轨迹"
  #   参与避碰计算 —— 比赛日绝不能带着它飞。
  CMD="roslaunch race6_comm multibotnet_node.launch drone_id:=$ID config:=$CONFIG"
else
  CMD="roslaunch race6_comm comm_ground_test.launch drone_id:=$ID peers:=$PEERS config:=$CONFIG"
fi

echo
echo "将要执行："
echo "  $CMD"
echo
echo "提醒："
if [ "$RACE" = "1" ]; then
  echo "  * 【比赛模式】只起了 Multibotnet，没有任何测试脚本（peers 在此模式下无意义）"
  echo "  * 想看时间同步监控，另开一个终端跑："
  echo "        rosrun race6_comm race6_time_sync.py --self-id $ID --peers $PEERS"
else
  echo "  ⚠️ 【联调模式】会同时起三个测试脚本，其中 race6_mintraj_test 会发【假轨迹】！"
  echo "     比赛日请改用： bash $0 $PHYS --race"
fi
echo "  * 里程计那一路要有数据，得另开一个终端起 mavros（不要加 &，否则 Ctrl+C 停不掉）："
echo "        roslaunch mavros px4.launch"
echo

if [ "$DRY_RUN" = "1" ]; then
  ok "--dry-run：不实际启动"
  exit 0
fi

exec $CMD
