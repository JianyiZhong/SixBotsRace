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
#
# ★ 飞机上（复用原有工作区，不要再造一个）：
#   WS=$HOME/chen_ws bash <仓库根>/race6_comm/scripts/setup_six_ws.sh --minimal --no-lab
#   --no-lab 只把 multibotnet + race6_comm 放进工作区，【绝不碰】工作区里原有的
#   实验室代码（否则同名包重复，或给 px4ctrl 之类贴上 CATKIN_IGNORE 把飞行栈搞坏）。
# =====================================================================
set -u

WS="${WS:-$HOME/six_ws}"
# MBN_SRC 在下面算出「仓库根」之后再解析（要优先用仓库里自带的副本，见步骤 2.1）
MBN_SRC="${MBN_SRC:-}"

MODE="minimal"
METHOD="link"
DEPS_ONLY=0
INSTALL_DEPS=0
# NO_LAB=1：只把 multibotnet + race6_comm 放进工作区，【不碰实验室代码】。
#   用途：飞机上已经有现成的实验室工作区（~/chen_ws），里面已经有一份实验室代码，
#   此时再把 controller_msgs / plan_manage 等塞进去会造成【同名包重复】，
#   catkin_make 直接报 "Multiple packages found with the same name"。
#   用法： WS=$HOME/chen_ws bash setup_six_ws.sh --minimal --no-lab
NO_LAB=0

for a in "$@"; do
  case "$a" in
    --minimal)     MODE="minimal" ;;
    --full)        MODE="full" ;;
    --link)        METHOD="link" ;;
    --copy)        METHOD="copy" ;;
    --deps-only)   DEPS_ONLY=1 ;;
    --install-deps) INSTALL_DEPS=1 ;;
    --no-lab)      NO_LAB=1 ;;
    -h|--help)     sed -n '2,70p' "$0"; exit 0 ;;
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
elif [ -d "$REPO_GUESS/race6_comm/scripts" ] || [ -f "$REPO_GUESS/PROJECT.md" ]; then
  # ★ 不再用 ego-planner2 判断"是否在仓库里"：仓库现在只放通讯模块，
  #   实验室代码已移出（参考副本放在 ~/lab_code_new，不进这个仓库）。
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
echo " 工作区搭建（把 multibotnet + race6_comm 放进一个 catkin 工作区）"
echo "   工作区   : $WS"
echo "   race6_comm: $RC_DIR"
echo "   实验室代码: $LAB"
echo "   模式     : $MODE"
echo "   放置方式 : $METHOD"
[ "$NO_LAB" = "1" ] && echo "   --no-lab : 是（只放两个新包，不碰工作区里原有的实验室代码）"
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

if [ "$NO_LAB" = "1" ]; then
  # 飞机模式：工作区里已经有一份实验室代码（~/chen_ws/src），绝不能再放一份，
  # 否则同名包重复，catkin_make 会报 "Multiple packages found with the same name"。
  step "2.2 跳过实验室代码（--no-lab）"
  ok "只放了 multibotnet + race6_comm；实验室代码用工作区里原有的那份"
else
  step "2.2 放置实验室代码"
  PLANNER="$LAB/ego-planner2/src/planner"
  if [ -d "$PLANNER" ]; then
    # controller_msgs：MinTraj 定义在这里，通信测试就要它
    place_file "$PLANNER/controller_msgs" "$SRC/planner/controller_msgs" "controller_msgs（MinTraj）"
  else
    # 仓库里已经没有实验室代码了（新布局）→ 自动退化成 --no-lab，不要报错退出
    warn "仓库里没有 $PLANNER（实验室代码已移出仓库），自动按 --no-lab 处理"
    NO_LAB=1
    if [ "$MODE" = "full" ]; then
      warn "你用的是 --full（要在 PC 上搭仿真），但没有实验室代码可放。"
      echo "       要搭 PC 仿真请显式指路： LAB=$HOME/lab_code_new bash $0 --full"
    fi
  fi

  if [ "$MODE" = "full" ] && [ -d "$PLANNER" ]; then
    for p in traj_utils path_searching plan_env traj_opt plan_manage drone_detect; do
      place_file "$PLANNER/$p" "$SRC/planner/$p" "$p"
    done
    place_file "$LAB/MLMapping-Embedded_version" "$SRC/MLMapping-Embedded_version" "MLMapping-Embedded_version"
    place_file "$LAB/FAST_LIO"                   "$SRC/FAST_LIO"                   "FAST_LIO"
  fi
fi

# ---------------------------------------------------------------------
# 2.2b controller_msgs：race6_comm 的编译依赖，必须有人提供
#   MinTraj 的定义就在这个包里 —— 它是【通讯接口契约】，所以即使仓库不再带
#   实验室代码，也建议把它作为 third_party/controller_msgs 留在仓库里。
#   飞机上 chen_ws 本来就有 → 检测到就跳过（放第二份会导致"同名包重复"编译失败）。
# ---------------------------------------------------------------------
if [ "$NO_LAB" = "1" ] || [ ! -d "$LAB/ego-planner2/src/planner" ]; then
  EXISTING_CM="$(find "$SRC" -maxdepth 5 -name package.xml 2>/dev/null \
                 | xargs grep -l '<name>controller_msgs</name>' 2>/dev/null | head -1)"
  if [ -n "$EXISTING_CM" ]; then
    ok "工作区里已经有 controller_msgs，跳过（$EXISTING_CM）"
  elif [ -d "$REPO_GUESS/third_party/controller_msgs" ]; then
    place_file "$REPO_GUESS/third_party/controller_msgs" "$SRC/third_party/controller_msgs" \
               "controller_msgs（仓库自带的接口定义）"
  else
    fail "工作区里没有 controller_msgs，仓库里也没有 third_party/controller_msgs"
    echo "     race6_comm 编译需要它（controller_msgs/MinTraj.msg 就是通讯协议本身）。补上："
    echo "       mkdir -p $REPO_GUESS/third_party"
    echo "       rsync -a <真机实验室代码>/ego-planner2/src/planner/controller_msgs/ \\"
    echo "             $REPO_GUESS/third_party/controller_msgs/"
    echo "     然后再跑一次本脚本。"
    exit 1
  fi
fi

# ---------------------------------------------------------------------
# 以下两块【只有 PC 上的 --full 模式】才需要：
#   飞机上（--no-lab）工作区里本来就已经有 px4Controller / rviz_draw / MLMapping / FAST_LIO
#   和装好的 Sophus，再塞一份会造成同名包重复，也没有必要重新装 Sophus。
# ---------------------------------------------------------------------
if [ "$MODE" = "full" ] && [ "$NO_LAB" = "0" ]; then

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
# 3. 无法编译的包加 CATKIN_IGNORE（只影响 PC 上的 --full）
#    ★ --no-lab 时【绝不能跑】：那会给飞机上原有工作区的包贴 CATKIN_IGNORE，
#      等于把飞机本来的飞行栈（px4ctrl 等）从编译里摘掉！所以这里必须双重限定。
# ---------------------------------------------------------------------
if [ "$MODE" = "full" ] && [ "$NO_LAB" = "0" ]; then
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
if [ ! -d "$LAB/ego-planner2" ] && [ ! -d "$LAB/MLMapping-Embedded_version" ] && [ ! -d "$LAB/FAST_LIO" ]; then
  # 新布局：实验室代码已移出仓库，这条守卫没有对象了。
  # 改为检查"仓库里有没有意外残留的实验室目录"（有的话说明没清干净）。
  ok "仓库里已不含实验室代码（新布局：通讯模块独立仓库）"
  echo "       实验室代码参考副本：$HOME/lab_code_new（不进本仓库）"
  echo "       版本对齐靠 $LAB/deps.lock 记录；同步实验室代码时更新它。"
  if [ -d "$LAB/.git" ]; then
    ok "当前仓库 commit：$(git -C "$LAB" rev-parse --short HEAD 2>/dev/null) ($(git -C "$LAB" rev-parse --abbrev-ref HEAD 2>/dev/null))"
  fi
elif [ -d "$LAB/.git" ]; then
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
    warn "★ 这会让【整个工作区】重新编译一遍；如果这是飞机上的 chen_ws，可能要等很久，且期间无法起飞。"
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
  echo "如果只是 planner/mlmapping 之类的包失败："
  echo "  * PC 上：地面通信+时间同步只需要 multibotnet / controller_msgs / race6_comm，"
  echo "           换一个干净的 --minimal 工作区重建即可。"
  echo "  * 飞机上（--no-lab，复用原有 ~/chen_ws）：不要重建工作区！"
  echo "           那是飞机唯一能飞的栈。先看是不是 multibotnet/race6_comm 本身的问题："
  echo "             cd $WS && catkin_make -DCATKIN_WHITELIST_PACKAGES=\"multibotnet;race6_comm\""
  echo "           编过之后再 catkin_make -DCATKIN_WHITELIST_PACKAGES=\"\" 清掉白名单。"
  exit "$RC"
fi
ok "编译成功"

# ---------------------------------------------------------------------
# 6. 验证
# ---------------------------------------------------------------------
# ---------------------------------------------------------------------
# 5.5 校验 launch 文件的 XML 合法性
#   专门拦一类很隐蔽的错误：XML 注释里出现【连续两个减号】"--"。
#   例如 <!-- ... 请加 --apply 参数 ... --> 会让 roslaunch 直接报
#     RLException: Invalid roslaunch XML syntax: not well-formed (invalid token)
#   而且只有真正加载到那个文件时才报 —— 自测的 launch 能跑，不代表别的也能跑。
#   （XML 规范：注释内容里不允许出现 "--"，因为它是注释结束符的一部分。）
# ---------------------------------------------------------------------
step "5.5 校验 launch 文件的 XML 合法性"
XMLBAD=0
for f in "$RC_DIR"/launch/*.launch; do
  [ -f "$f" ] || continue
  if ERR=$(python3 -c "import sys,xml.dom.minidom as m; m.parse(sys.argv[1])" "$f" 2>&1); then
    ok "XML OK: $(basename "$f")"
  else
    fail "XML 不合法: $(basename "$f")"
    echo "$ERR" | tail -2 | sed 's/^/       /'
    XMLBAD=$((XMLBAD+1))
  fi
done
if [ "$XMLBAD" -ne 0 ]; then
  fail "$XMLBAD 个 launch 文件有 XML 语法错误"
  echo "       最常见原因：注释里写了连续两个减号。逐个查："
  echo "         grep -n -- '--' $RC_DIR/launch/*.launch"
  echo "       看报的【行号:列号】，那一列附近一定有连续两个减号，改成别的写法即可。"
  exit 1
fi

step "6. 校验 Multibotnet 配置（静默失败的重灾区）"
# 见 scripts/check_config.py 头部：connect_address 用了 IP 段里【没有定义】的别名时，
# resolveAddress() 会原样返回字符串（topic_manager.cpp:341-352），
# 连接地址变成 tcp://<别名>:port，ZMQ 当主机名去解析、解析不了就永远连不上，
# 而 Multibotnet 一个错都不报 —— 只在"接收计数一直是 0"上体现。
if [ -f "$RC_DIR/scripts/check_config.py" ]; then
  if python3 "$RC_DIR/scripts/check_config.py" --dir "$RC_DIR/config" --quiet; then
    ok "Multibotnet 配置全部通过"
  else
    fail "有配置文件不通过 —— 先修掉再上飞机（最常见：connect_address 用了未定义的别名）"
    exit 1
  fi
else
  warn "找不到 scripts/check_config.py，跳过配置检查"
fi

step "7. 验证包可见性"
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
  echo "=================================================================="
  echo -e "${Y}★ 上面这些 [ OK ] 是在【本脚本自己的子 shell】里验证的，"
  echo -e "  它【不会】改变你当前终端的 ROS_PACKAGE_PATH！"
  echo -e "  现在请在你的终端里手动执行这一行，roslaunch 才能找到 race6_comm：${N}"
  echo
  echo "      source $WS/devel/setup.bash"
  echo
  echo "  想一劳永逸，就把它写进 ~/.bashrc（以后新开终端自动生效）："
  echo "      cat >> ~/.bashrc <<'BASH_EOF'"
  echo "      source /opt/ros/noetic/setup.bash"
  echo "      [ -f $WS/devel/setup.bash ] && source $WS/devel/setup.bash"
  echo "      BASH_EOF"
  echo
  echo "  自检（必须打印出路径，不能是 not found）："
  echo "      rospack find race6_comm"
  echo "=================================================================="
  echo
  echo "下一步（真机通讯 + 时间同步联调）："
  echo "  1) 填 IP：用生成器一次重写全部配置（会同时写 race6/ 和 ground2/，不会漏）"
  echo "       cd $RC_DIR && python3 scripts/gen_race6_configs.py --ips <物理1IP>,<物理2IP>,<物理3IP>,<物理4IP>,<物理5IP>,<物理6IP>"
  echo "       python3 scripts/check_config.py --all      # 必须全绿"
  echo "  2) 每台飞机上按【物理编号】起（drone_id = 物理编号 - 1，自动算）："
  echo "       bash $RC_DIR/scripts/race6_launch.sh <物理编号>              # 六台全上"
  echo "       bash $RC_DIR/scripts/race6_launch.sh 3 --pair=5             # 只和物理 5 号机对测"
  echo "       详见 $RC_DIR/测试全链路.md"
  echo "  3) 验收通过后别忘提交（否则只有工作区里有，仓库里没有）："
  echo "       git -C $LAB add race6_comm && git -C $LAB commit -m 'config: 填入真实 IP'"
  echo "       git -C $LAB tag -a v0.1-ground-comms -m '地面双机通信+时间同步验收通过'"
  echo "  详见 $RC_DIR/README.md 与 $LAB/PROJECT.md"
else
  fail "$MISSING 个包缺失，看编译日志 $LOG"
  echo "  提示：rospack 有缓存，新增包后要 source 工作区并执行 rospack profile 才会被看到。"
  exit 1
fi

echo
ok "完成。"
