#!/bin/bash
# =====================================================================
# race6_comm / scripts/setup_six_ws.sh
#
# 六机竞赛工作区搭建脚本：把实验室最新代码 + Multibotnet + race6_comm
# 组装成一个可编译的 catkin 工作区（默认 ~/six_ws）。
#
# 为什么需要它：
#   /home/zjy/SixBotsRace 是【源码库】而不是 catkin 工作区
#   （它里面是 FAST_LIO/、MLMapping-Embedded_version/、ego-planner2/src/planner/*，
#     没有 src/ 入口），catkin_make 无法直接在它上面跑。
#
# 两种模式：
#   --minimal （默认）只装「地面通信+时间同步联调」需要的三个包：
#               multibotnet + controller_msgs + race6_comm
#               几十秒就能编完，不需要 Sophus / PCL / livox / mavros。
#               ★ 你现在这一步只需要这个。
#   --full     再装规划/建图/控制全套（planner 七个包 + MLMapping + FAST_LIO
#               + px4Controller/livox 若存在），需要先装 Sophus，编译很久。
#               第二阶段（仿真/上机飞）再用。
#
# 两种放置方式：
#   --link （默认）软链接到工作区。仓库是唯一真源，改完立刻生效，不用重复复制；
#                 上飞机时直接 git clone 仓库再跑本脚本，也一样用软链接。
#   --copy        真正复制到工作区。想脱离仓库独立带走一份时用。
#
# 它从哪里找实验室代码（★ 推荐把 race6_comm 放进仓库，见 PROJECT.md）：
#   <仓库根>/
#     ├── race6_comm/scripts/setup_six_ws.sh   ← 本脚本
#     ├── ego-planner2/src/planner/*           ← 实验室代码，永远不改
#     ├── MLMapping-Embedded_version/  FAST_LIO/
#   默认把「race6_comm 的上一级目录」当仓库根；
#   若那里没有 ego-planner2/（说明你还放在旧位置 <ws>/src/race6_comm/），
#   脚本会回退到 $HOME/SixBotsRace 并提示你搬家。
#
# 幂等：重复执行不会出错，已存在的不会再动。
#
# 用法：
#   bash <仓库根>/race6_comm/scripts/setup_six_ws.sh --minimal
#   bash <仓库根>/race6_comm/scripts/setup_six_ws.sh --full
#   bash <仓库根>/race6_comm/scripts/setup_six_ws.sh --minimal --deps-only
#   WS=$HOME/other_ws bash setup_six_ws.sh --minimal
# =====================================================================
set -u

WS="${WS:-$HOME/six_ws}"
# MBN_SRC 在下面算出「仓库根」之后再解析（要优先用仓库里自带的副本，见步骤 2.1）
MBN_SRC="${MBN_SRC:-}"

MODE="minimal"
METHOD="link"
DEPS_ONLY=0
INSTALL_DEPS=0

for a in "$@"; do
  case "$a" in
    --minimal)     MODE="minimal" ;;
    --full)        MODE="full" ;;
    --link)        METHOD="link" ;;
    --copy)        METHOD="copy" ;;
    --deps-only)   DEPS_ONLY=1 ;;
    --install-deps) INSTALL_DEPS=1 ;;
    -h|--help)     sed -n '2,40p' "$0"; exit 0 ;;
    *) echo "未知参数: $a"; exit 2 ;;
  esac
done

R='\033[31m'; G='\033[32m'; Y='\033[33m'; B='\033[34m'; N='\033[0m'
ok()   { echo -e "${G}[ OK ]${N} $*"; }
warn() { echo -e "${Y}[WARN]${N} $*"; }
fail() { echo -e "${R}[FAIL]${N} $*"; }
step() { echo; echo -e "${B}===== $* =====${N}"; }

SRC="$WS/src"

# ---------------------------------------------------------------------
# 0. 定位仓库根 / 实验室代码
#    repo 布局（推荐）： <仓库>/race6_comm/scripts/setup_six_ws.sh
#    旧布局：           <ws>/src/race6_comm/scripts/setup_six_ws.sh
# ---------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RC_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"                 # race6_comm 包目录
REPO_GUESS="$(cd "$RC_DIR/.." && pwd)"                 # 仓库根（race6_comm 的上一级）

if [ -n "${LAB:-}" ]; then
  IN_REPO=1                                            # 用户显式指定，尊重
elif [ -d "$REPO_GUESS/ego-planner2/src/planner" ]; then
  LAB="$REPO_GUESS"; IN_REPO=1
else
  LAB="${HOME}/SixBotsRace"; IN_REPO=0
fi

# ---------------------------------------------------------------------
# Multibotnet 从哪里来（按优先级）：
#   1) 环境/命令行显式指定 MBN_SRC=...
#   2) 仓库里自带的 third_party/multibotnet  ← ★ 推荐：飞机上 git clone 一步到位，离线可部署
#   3) 开发机上现成的 ~/chen_ws2/src/multibotnet
#   4) 工作区里已经有 src/multibotnet（真机上的第二次及以后运行走这条，见 2.1）
# ---------------------------------------------------------------------
if [ -z "$MBN_SRC" ]; then
  for cand in "$REPO_GUESS/third_party/multibotnet" \
              "$REPO_GUESS/multibotnet" \
              "$HOME/chen_ws2/src/multibotnet"; do
    if [ -d "$cand" ]; then MBN_SRC="$cand"; break; fi
  done
fi

echo "=============================================================="
echo " six_ws 工作区搭建"
echo "   工作区   : $WS"
echo "   race6_comm: $RC_DIR"
echo "   实验室代码: $LAB"
echo "   模式     : $MODE"
echo "   放置方式 : $METHOD"
echo "=============================================================="

if [ "$IN_REPO" = "0" ]; then
  warn "race6_comm 目前不在仓库里（真源散落在工作区里，容易丢失/和仓库不一致）"
  echo "       建议搬进仓库，仓库才是项目的唯一真源（按顺序执行，别跳步）："
  echo "         mv $RC_DIR $HOME/SixBotsRace/race6_comm      # ★ 先做这个，确认成功"
  echo "         rm -rf $WS/src/race6_comm                    # ★ mv 成功后才执行（此时它已不存在）"
  echo "         bash $HOME/SixBotsRace/race6_comm/scripts/setup_six_ws.sh $MODE"
  echo "       详见 $HOME/SixBotsRace/PROJECT.md「一次搬家」"
fi

[ -d "$LAB" ] || { fail "找不到实验室代码目录 $LAB（用 LAB=... 覆盖）"; exit 1; }
# 注意：这里必须检查 package.xml，不能检查 $SRC/race6_comm ——
# 搬家之后 race6_comm 在仓库里，工作区里的软链是后面 place_race6_comm 才建的。
[ -f "$RC_DIR/package.xml" ] || { fail "找不到 $RC_DIR/package.xml —— 本脚本必须放在 race6_comm/scripts/ 里执行"; exit 1; }

# ---------------------------------------------------------------------
# 先 source ROS：后面的 rospack / catkin_init_workspace / catkin_make 都要用它。
# （catkin_init_workspace 在 /opt/ros/noetic/bin 里，不 source 就找不到命令）
# ---------------------------------------------------------------------
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash || { fail "source /opt/ros/noetic/setup.bash 失败，ROS 装了吗？"; exit 1; }
ok "ROS 环境已加载（$ROS_DISTRO）"

# ---------------------------------------------------------------------
# 0. 依赖检查
# ---------------------------------------------------------------------
step "0. 系统依赖检查"
MISSING_APT=""
need_pkg() {  # need_pkg <apt包名> <检查用的文件或命令>
  if eval "$2" >/dev/null 2>&1; then
    ok "$1 已就绪"
  else
    warn "缺少 $1"
    MISSING_APT="$MISSING_APT $1"
  fi
}

need_pkg "libzmq3-dev"            "[ -f /usr/include/zmq.h ]"
need_pkg "libyaml-cpp-dev"        "pkg-config --exists yaml-cpp"
need_pkg "liblz4-dev"             "[ -f /usr/include/lz4.h ]"
need_pkg "zlib1g-dev"             "[ -f /usr/include/zlib.h ]"
need_pkg "ros-noetic-topic-tools" "rospack find topic_tools"
need_pkg "ros-noetic-mavros-msgs" "rospack find mavros_msgs"
need_pkg "ros-noetic-rostest"     "rospack find rostest"

if [ -n "$MISSING_APT" ]; then
  echo
  echo "缺少以下 apt 包（multibotnet / controller_msgs 编不过）："
  echo "  sudo apt install -y$MISSING_APT"
  if [ "$INSTALL_DEPS" = "1" ]; then
    echo "（--install-deps 已指定，正在安装…）"
    # shellcheck disable=SC2086
    sudo apt install -y $MISSING_APT || { fail "apt 安装失败"; exit 1; }
  else
    warn "请先执行上面的 apt 命令，或加 --install-deps 让本脚本代劳"
    exit 1
  fi
fi

[ "$DEPS_ONLY" = "1" ] && { echo; ok "依赖检查完成（--deps-only）"; exit 0; }

# ---------------------------------------------------------------------
# 1. 初始化工作区
# ---------------------------------------------------------------------
step "1. 初始化 catkin 工作区"
mkdir -p "$SRC"
if [ -f "$WS/.catkin_workspace" ]; then
  ok "已是 catkin 工作区"
else
  ( cd "$SRC" && catkin_init_workspace ) && ok "已初始化（src/CMakeLists.txt 已生成）" \
    || { fail "catkin_init_workspace 失败"; exit 1; }
fi

# ---------------------------------------------------------------------
# 2. 放置源码包
# ---------------------------------------------------------------------
place_file() {  # place_file <src> <dst绝对路径> <说明>
  local src="$1" dst="$2" what="$3"
  if [ -e "$dst" ] || [ -L "$dst" ]; then
    if [ "$METHOD" = "link" ] && [ ! -L "$dst" ]; then
      warn "已存在但是【真实副本】、不是软链：$what"
      echo "       不影响使用，但它不会跟随真源更新（早期用 --copy 跑过一次会留下这种副本）"
    else
      ok "已存在，跳过：$what"
    fi
    return 0
  fi
  mkdir -p "$(dirname "$dst")"
  if [ "$METHOD" = "link" ]; then
    ln -s "$src" "$dst" && ok "软链 $what  -> $src"
  else
    if command -v rsync >/dev/null 2>&1; then
      # 排除 .git / 旧 build / 点云数据：既省时间，也避免把
      # MLMapping 里 CMakeCache 硬编码 /home/orangepi 的旧 build 目录带过来
      rsync -a --exclude='.git' --exclude='build' --exclude='PCD' \
               --exclude='Log' --exclude='*.pcd' "$src" "$dst" \
        && ok "复制 $what"
    else
      cp -a "$src" "$dst" && ok "复制 $what（未装 rsync，未排除 .git/build）"
    fi
  fi
}

step "2. 放置源码包（$METHOD）"

# ---- 2.0 race6_comm 本体（你自己的代码，真源在仓库里）----
place_race6_comm() {
  local dst="$SRC/race6_comm"

  # ★ 安全第一：旧布局下真源就在工作区里，此时 $dst 和 $RC_DIR 是【同一个目录】。
  #   必须最先判断，否则会被下面"真实副本"的分支误判，进而提示 rm -rf 把唯一副本删掉！
  if [ -d "$dst" ]; then
    local dst_real; dst_real="$(cd "$dst" 2>/dev/null && pwd)"
    if [ -n "$dst_real" ] && [ "$dst_real" = "$RC_DIR" ]; then
      warn "race6_comm 的真源目前就在工作区里（旧布局），无需放置"
      echo "       搬进仓库的步骤见上面提示，或 $HOME/SixBotsRace/PROJECT.md「一次搬家」"
      return 0
    fi
  fi

  # 已经是软链且指向同一个真源 → 什么都不用做
  if [ -L "$dst" ]; then
    local tgt; tgt="$(readlink -f "$dst" 2>/dev/null || true)"
    if [ "$tgt" = "$RC_DIR" ]; then
      ok "软链已就位：$dst -> $RC_DIR"
      return 0
    fi
    fail "$dst 是指向别处的软链（$tgt），请先删掉：rm $dst"
    return 1
  fi

  # 已存在但是个真目录 → 确实是另一份旧副本，这种最危险：改了不生效还不报错
  if [ -d "$dst" ]; then
    fail "$dst 是一个【真实副本】，不是软链 —— 它会和仓库里的真源各改各的！"
    echo "       确认过它只是副本（真源已在 $RC_DIR）之后，这样清掉再重跑："
    echo "         rm -rf $dst"
    echo "         bash $RC_DIR/scripts/setup_six_ws.sh $MODE"
    return 1
  fi

  mkdir -p "$(dirname "$dst")"
  if [ "$METHOD" = "link" ]; then
    ln -s "$RC_DIR" "$dst" && ok "软链 race6_comm  -> $RC_DIR"
  else
    cp -a "$RC_DIR" "$dst" && ok "复制 race6_comm（注意：这是一份快照，改仓库不会同步过来）"
  fi
}

# ---- 2.1 Multibotnet（跨机通讯本体，第三方 v4.1.2，非实验室代码）----
# ★ 真机部署的关键一步：飞机上不会有 ~/chen_ws2，也不一定有外网。
#   所以优先用【工作区里已有的副本】，其次用【仓库里自带的 third_party/multibotnet】，
#   最后才退回开发机上的 ~/chen_ws2。把 multibotnet 放进仓库后，
#   一台飞机只需要 `git clone` 一次就全部齐了，离线也能部署。
if [ -e "$SRC/multibotnet" ]; then
  ok "multibotnet 已在工作区里，跳过（$SRC/multibotnet）"
elif [ -n "$MBN_SRC" ] && [ -d "$MBN_SRC" ]; then
  place_file "$MBN_SRC" "$SRC/multibotnet" "multibotnet（来自 $MBN_SRC）"
else
  fail "找不到 multibotnet 源（MBN_SRC='$MBN_SRC'）"
  echo "     飞机上要一步到位，建议把它放进仓库（在 PC 上执行一次）："
  echo "       mkdir -p $REPO_GUESS/third_party"
  echo "       cp -a $HOME/chen_ws2/src/multibotnet $REPO_GUESS/third_party/"
  echo "       git -C $REPO_GUESS add third_party && git -C $REPO_GUESS commit -m 'vendor: 内置 Multibotnet v4.1.2（离线部署用）'"
  echo "       git -C $REPO_GUESS push"
  echo "     或者临时指定： MBN_SRC=<路径> bash $0 $MODE"
  exit 1
fi

# ---- 2.2 实验室最新代码（只读引用，我们不改它）----
place_race6_comm || { fail "race6_comm 放置失败，先处理上面的提示再重跑"; exit 1; }

PLANNER="$LAB/ego-planner2/src/planner"
if [ -d "$PLANNER" ]; then
  # controller_msgs：MinTraj 定义在这里，通信测试就要它
  place_file "$PLANNER/controller_msgs" "$SRC/planner/controller_msgs" "controller_msgs（MinTraj）"
else
  fail "找不到 $PLANNER"
  exit 1
fi

if [ "$MODE" = "full" ]; then
  for p in traj_utils path_searching plan_env traj_opt plan_manage drone_detect; do
    place_file "$PLANNER/$p" "$SRC/planner/$p" "$p"
  done
  place_file "$LAB/MLMapping-Embedded_version" "$SRC/MLMapping-Embedded_version" "MLMapping-Embedded_version"
  place_file "$LAB/FAST_LIO"                   "$SRC/FAST_LIO"                   "FAST_LIO"

  # 实机控制/驱动（在 chen_ws2 里有现成的就一起带上）
  for extra in \
      "$HOME/chen_ws2/src/px4Controller:px4Controller" \
      "$HOME/chen_ws2/src/rviz_draw:rviz_draw" \
      "$HOME/chen_ws2/src/Livox-SDK2:Livox-SDK2" \
      "$HOME/chen_ws2/src/livox_ros_driver2:livox_ros_driver2" \
      "$HOME/chen_ws2/src/super-lio:super-lio" \
      "$HOME/chen_ws2/src/cmd_to_mavros:cmd_to_mavros" ; do
    s="${extra%%:*}"; n="${extra##*:}"
    if [ -d "$s" ]; then
      place_file "$s" "$SRC/$n" "$n（来自 chen_ws2）"
    else
      warn "没有 $s，跳过 $n"
    fi
  done

  # ---- 2.3 Sophus（MLMapping 依赖，必须装到 /usr/local）----
  step "2.3 检查 Sophus"
  if [ -f /usr/local/lib/cmake/Sophus/SophusConfig.cmake ] \
     && [ -f /usr/local/include/sophus/se3.h ] \
     && [ -f /usr/local/lib/libSophus.so ]; then
    ok "Sophus 已完整安装（Config + 老版 se3.h + libSophus.so）"
  else
    SOPHUS_SH="$HOME/chen_ws2/src/race3_cfg/install_sophus.sh"
    if [ -f "$SOPHUS_SH" ]; then
      warn "Sophus 未装全，调用现成脚本：$SOPHUS_SH"
      bash "$SOPHUS_SH" || { fail "Sophus 安装失败"; exit 1; }
    else
      fail "Sophus 未安装，且找不到 $SOPHUS_SH"
      echo "     MLMapping 代码 include 的是老版 <sophus/se3.h>，"
      echo "     必须用 3rdPartLib 里自带那份，不能装新版（新版只有 .hpp）。"
      exit 1
    fi
  fi
fi

# ---------------------------------------------------------------------
# 3. 无法编译的包加 CATKIN_IGNORE（只影响 --full）
# ---------------------------------------------------------------------
if [ "$MODE" = "full" ]; then
  step "3. 给暂时编不了的包加 CATKIN_IGNORE"
  set_ignore() {
    local d="$1" why="$2"
    [ -d "$d" ] || return 0
    if [ -f "$d/CATKIN_IGNORE" ]; then ok "已有 CATKIN_IGNORE: ${d#$SRC/}"
    else printf '【跳过】%s\n' "$why" > "$d/CATKIN_IGNORE"; ok "新建: ${d#$SRC/} ($why)"; fi
  }
  set_ignore "$SRC/super-lio"                        "依赖 livox_ros_driver2"
  set_ignore "$SRC/cmd_to_mavros"                    "依赖 mavros"
  set_ignore "$SRC/px4Controller/src/px4ctrl"        "依赖 mavros（实机才要）"
  set_ignore "$SRC/px4Controller/src/control_test"   "依赖 px4ctrl"
  # ★ 绝不能忽略 px4Controller 本身：quadrotor_msgs 在里面，ego_planner 依赖它
  for bad in "$SRC/px4Controller/CATKIN_IGNORE" \
             "$SRC/px4Controller/src/CATKIN_IGNORE" \
             "$SRC/px4Controller/src/utils/CATKIN_IGNORE"; do
    [ -f "$bad" ] && { rm -f "$bad"; warn "删除危险标记（会连带跳过 quadrotor_msgs）：$bad"; }
  done
fi

# ---------------------------------------------------------------------
# 3.5 守卫：实验室代码必须保持"零改动"
#   这是本项目的硬规矩（见 PROJECT.md）：实验室子目录一行都不改，
#   所有新东西都放 race6_comm/。这样：
#     * 以后能干净地 git pull 实验室更新，不会冲突
#     * 给实验室交东西时，PR 里只有 race6_comm/ 一个新目录，零修改
#   有改动时这里会列出来，方便你确认是"手滑"还是"确实需要"。
# ---------------------------------------------------------------------
step "3.5 守卫：实验室代码是否被改动过"
LAB_DIRS="ego-planner2 MLMapping-Embedded_version FAST_LIO"
if [ -d "$LAB/.git" ]; then
  DIRTY="$(git -C "$LAB" status --porcelain -- $LAB_DIRS 2>/dev/null)"
  if [ -z "$DIRTY" ]; then
    ok "实验室代码干净（$LAB 下这三个子目录没有未提交改动）"
  else
    warn "实验室代码有改动！请确认是故意的："
    echo "$DIRTY" | sed 's/^/       /'
    echo "       如果只想看实验室改了什么："
    echo "         git -C $LAB diff -- $LAB_DIRS"
    echo "       如果不是故意的："
    echo "         git -C $LAB checkout -- $LAB_DIRS"
  fi
  # 顺手把当前 commit 报出来，写进 deps.lock 用得上
  ok "当前仓库 commit：$(git -C "$LAB" rev-parse --short HEAD 2>/dev/null) ($(git -C "$LAB" rev-parse --abbrev-ref HEAD 2>/dev/null))"
else
  warn "$LAB 不是 git 仓库，无法校验实验室代码是否被改动（建议用 git 管理，见 PROJECT.md）"
fi

# ---------------------------------------------------------------------
# 4. 清理 Windows 残留（从 Windows 复制过来的文件常带 :Zone.Identifier）
# ---------------------------------------------------------------------
step "4. 清理 :Zone.Identifier 残留"
ZI_LIST=()
while IFS= read -r -d '' f; do ZI_LIST+=("$f"); done \
  < <(find "$WS" -name "*:Zone.Identifier" -print0 2>/dev/null)
if [ "${#ZI_LIST[@]}" -gt 0 ]; then
  for f in "${ZI_LIST[@]}"; do rm -f "$f" 2>/dev/null || true; done
  ok "删除 ${#ZI_LIST[@]} 个残留文件"
else
  ok "无残留"
fi

# ---------------------------------------------------------------------
# 5. 编译
# ---------------------------------------------------------------------
step "5. catkin_make"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash || { fail "source ROS 失败"; exit 1; }
cd "$WS" || exit 1

# 上次 configure 失败会留下坏缓存（例如 CMAKE_HOME_DIRECTORY 不是本机路径）
if [ -f "$WS/build/CMakeCache.txt" ]; then
  CACHED_HOME=$(grep -m1 '^CMAKE_HOME_DIRECTORY' "$WS/build/CMakeCache.txt" 2>/dev/null | cut -d= -f2-)
  if [ -n "$CACHED_HOME" ] && [ "$CACHED_HOME" != "$WS/src" ]; then
    warn "build/CMakeCache.txt 记的是 '$CACHED_HOME'，与本机 '$WS/src' 不符 → 删掉 build/ 与 devel/ 重来"
    rm -rf "$WS/build" "$WS/devel"
  fi
fi

LOG=/tmp/six_ws_build.log
echo "完整日志：$LOG"
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release 2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}

echo
if [ "$RC" -ne 0 ]; then
  fail "编译失败（exit=$RC）"
  echo "----- 最后 40 行里的错误 -----"
  grep -nE "error|Error|fatal|错误" "$LOG" | tail -40
  echo
  echo "如果只是 planner/mlmapping 之类的包失败，先不管它："
  echo "  地面通信+时间同步只需要 multibotnet / controller_msgs / race6_comm，"
  echo "  用 --minimal 模式重建一个干净工作区即可。"
  exit "$RC"
fi
ok "编译成功"

# ---------------------------------------------------------------------
# 6. 验证
# ---------------------------------------------------------------------
step "6. 验证包可见性"
# shellcheck disable=SC1091
source "$WS/devel/setup.bash" || true
rospack profile >/dev/null 2>&1 || true

MISSING=0
CHECK="multibotnet controller_msgs race6_comm"
[ "$MODE" = "full" ] && CHECK="$CHECK ego_planner traj_utils plan_env quadrotor_msgs mlmapping"
for p in $CHECK; do
  if P=$(rospack find "$p" 2>/dev/null); then ok "$p  ->  $P"; else fail "$p  找不到"; MISSING=$((MISSING+1)); fi
done

echo
if [ "$MISSING" -eq 0 ]; then
  ok "工作区就绪：$WS"
  echo
  echo "下一步（地面双机通信+时间同步联调）："
  echo "  1) 改 IP（改的是【仓库里的真源】，不是工作区副本）："
  echo "       vi $RC_DIR/config/ground2/drone_0.yaml   （drone0/drone1 两行）"
  echo "       vi $RC_DIR/config/ground2/drone_1.yaml   （同样的两行，必须一致）"
  echo "  2) 每台飞机上各起一份："
  echo "       roslaunch race6_comm comm_ground_test.launch drone_id:=0 peers:=1"
  echo "       roslaunch race6_comm comm_ground_test.launch drone_id:=1 peers:=0"
  echo "  3) 验收通过后别忘提交（否则只有工作区里有，仓库里没有）："
  echo "       git -C $LAB add race6_comm && git -C $LAB commit -m 'config: 填入真实 IP'"
  echo "       git -C $LAB tag -a v0.1-ground-comms -m '地面双机通信+时间同步验收通过'"
  echo "  详见 $RC_DIR/README.md 与 $LAB/PROJECT.md"
else
  fail "$MISSING 个包缺失，看编译日志 $LOG"
  exit 1
fi

echo
ok "完成。"
