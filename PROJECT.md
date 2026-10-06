# PROJECT.md —— 六机竞赛项目的代码管理约定

> 这份文档回答一个问题：**你的工作放哪里、怎么提交、怎么保证 PC 和飞机上是同一份代码。**
> 与它配套的文件：`deps.lock`（版本锁定）、`.gitignore`、`race6_comm/README.md`（通讯模块本身的用法）。

---

## 0. 先回答：我的工作要在 SixBotsRace 里改动吗？

要分成**两件不同的事**，答案一个是"不要"，一个是"要"：

| 问题 | 答案 |
|---|---|
| 要不要修改**实验室的代码文件**（`ego-planner2/`、`MLMapping-Embedded_version/`、`FAST_LIO/` 里的任何文件）？ | **不要，一行都不改。** 已逐行核实：多机通讯 + 时间同步完全不需要改它们（`advanced_param_swarm.xml:50-51` 已经把话题 remap 成跨机话题，Multibotnet 在外面搬运即可，见 `race6_comm/README.md` §1、§2） |
| 你的**新代码（`race6_comm/`）提交到哪个仓库？** | **就提交到这个仓库**，分支 `feature/zhongjianyi/multi-comm`。不要另开仓库 |

**为什么新代码要放进来（而不是单独一个仓库）：**

1. 你已经在 `main` 上把实验室代码提交并推到 GitHub 了（commit `205f1ad "add lab code"`），这个仓库已经是项目的唯一真源。新代码再放别处，就会变成"要记住两个仓库、两台飞机各 clone 两次"。
2. 一次 `git clone` 就拿到"实验室代码 + 你的代码"，部署和复现都只有一步。
3. **将来交回实验室时最干净**：一个 PR 里只有 `race6_comm/` 这**一个新目录**，对实验室代码**零修改**，Review 起来一目了然。
4. 反过来，如果你去改实验室文件，以后实验室更新代码时你会遇到冲突，而且说不清"这行为什么被你改了"。

**一句话**：**代码进同一个仓库，但只新增目录，绝不改实验室子目录。**

---

## 1. 三条铁律

### 铁律 1：实验室三个子目录永远零改动（只读）

```bash
cd ~/SixBotsRace
git status --porcelain -- ego-planner2 MLMapping-Embedded_version FAST_LIO
#   必须输出为空。有输出就是你手滑改了，用下面这行撤销：
git checkout -- ego-planner2 MLMapping-Embedded_version FAST_LIO
```

`race6_comm/scripts/setup_six_ws.sh` 已内置这个守卫（步骤 3.5），每次搭工作区都会替你检查一遍。

### 铁律 2：`~/six_ws` 是生成物，不入 git，不手工改

- 它是 `setup_six_ws.sh` 按需生成的 catkin 工作区，随时可以删掉重建。
- 里面的 `src/planner`、`src/MLMapping-Embedded_version`、`src/FAST_LIO`、`src/multibotnet`、`src/race6_comm` **全是软链接**，指向真源。
- 想改代码 → 改仓库里的 `race6_comm/`，立刻生效（软链）；这里的脚本改动要 `catkin_make` 才生效。

### 铁律 3：每个"现场可用"的状态都打 tag

比赛现场出问题时，你需要一条命令回到已知可用的版本：

```bash
git tag -a v0.1-ground-comms -m "地面双机通信+时间同步验收通过"
git push origin v0.1-ground-comms
# 现场回滚：
git stash -u && git checkout v0.1-ground-comms
```

---

## 2. 一次搬家：把 `race6_comm` 放进仓库

现在 `race6_comm` 还在 `~/six_ws/src/race6_comm`（工作区里），**真源散落在工作区是危险的**：`rm -rf ~/six_ws` 就没了，而且它不在版本控制里。搬进仓库：

```bash
# 1) 确认在正确的分支上
cd ~/SixBotsRace
git rev-parse --abbrev-ref HEAD          # 期望：feature/zhongjianyi/multi-comm

# 2) 搬家（这是移动，不是复制）
mv ~/six_ws/src/race6_comm ~/SixBotsRace/race6_comm

# 3) 提交
git add race6_comm .gitignore PROJECT.md deps.lock
git status --short                        # 眼过一遍：应该只有这几个新增
git commit -m "feat(comm): 新增 race6_comm —— 六机多机通讯与时间同步模块

- Multibotnet 配置：6 机竞赛 + 地面双机 + 单机回环自测
- 时间同步脚本：NTP 四时戳法实测两机时钟差（复现 ego_replan_fsm 的 250ms 判据）
- 链路验证脚本：通用收发验证 + MinTraj 主通道端到端验证
- setup_six_ws.sh：组装工作区，并把实验室代码零改动作为守卫
- 实验室代码 (ego-planner2 / MLMapping / FAST_LIO) 零改动"
git push origin feature/zhongjianyi/multi-comm

# 4) 重建工作区（会把 race6_comm 以软链接方式放回工作区，真源仍在仓库）
bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal
```

**目标布局：**

| 路径 | 内容 | 是否入库 |
|---|---|---|
| `<仓库>/ego-planner2/` `MLMapping-Embedded_version/` `FAST_LIO/` | 实验室代码 | ✅ 已入库，**只读** |
| `<仓库>/race6_comm/` | **你的代码**（通讯+时间同步+脚本+launch+配置） | ✅ 入库，日常在这里改 |
| `<仓库>/deps.lock` | 上游版本锁定（实验室快照来源、Multibotnet 版本） | ✅ 入库 |
| `<仓库>/PROJECT.md` `.gitignore` | 项目管理约定 | ✅ 入库 |
| `~/six_ws/` | catkin 工作区（软链 + build/ + devel/） | ❌ 生成物 |

> 仓库根目录的 `README.md` 是实验室的（只有一行 `# SixBotsRace`），**不要去改它** —— 铁律 1。项目说明写在 `PROJECT.md` 和 `race6_comm/README.md` 里。

---

## 3. 分支约定

| 分支 | 用途 | 规则 |
|---|---|---|
| `main` | 实验室代码基线 + 已验收的成果 | 只放**跑通过**的东西；不在上面写未验证的代码 |
| `feature/zhongjianyi/multi-comm` | **你当前的任务**（多机通讯 + 时间同步） | 日常提交都在这条上 |
| 以后每个任务一条 `feature/<拼音>/<任务名>` | 例：`feature/zhongjianyi/race6-sim`（6 机仿真）、`feature/zhongjianyi/bomb-drop`（投弹） | 同样规则 |

**合并回 `main` 的条件**：`race6_comm/README.md` §8 那张验收清单全打勾（地面双机通信+时间同步 PASS）。
每次合并用 `--no-ff` 保留任务边界：`git checkout main && git merge --no-ff feature/zhongjianyi/multi-comm`。

---

## 4. ★ 最关键的一条：给实验室代码挂 `upstream`

现状（从 `.git` 读出来的）：

- 只有一个远端 `origin = git@github.com:JianyiZhong/SixBotsRace.git`（你自己的仓库）
- 从来 fetch 过别的远端
- `main` 和 `feature/zhongjianyi/multi-comm` 目前都指向 `205f1ad`

⇒ **你现在拿不到实验室的代码更新**：`git pull` 只会从你自己的仓库拉，实验室那边更新了你不会知道。

### 情况 A：实验室有自己的仓库（推荐这么做）

```bash
cd ~/SixBotsRace
git remote add upstream <实验室仓库URL>
git fetch upstream
git log --oneline upstream/main | head        # 看看差距有多大

# 以后同步实验室代码：
git checkout main
git pull upstream main && git push origin main
git checkout feature/zhongjianyi/multi-comm
git merge main                                # 或 git rebase main
```

同步后必须**更新 `deps.lock`** 里的 `upstream_commit`，然后重跑一遍通信自测（铁律 3 打 tag）。

### 情况 B：实验室没有仓库，只给快照（U 盘 / 内网 / scp）

那么 `main` 就承担"上游快照"的角色，**每次拿到新代码单独一个 commit，信息写清来源和日期**：

```bash
cd ~/SixBotsRace
git checkout main
# 用新快照覆盖这三个目录（只覆盖、不删除多出来的文件）
rsync -a --delete <新快照>/ego-planner2/              ego-planner2/
rsync -a --delete <新快照>/MLMapping-Embedded_version/ MLMapping-Embedded_version/
rsync -a --delete <新快照>/FAST_LIO/                  FAST_LIO/
git status --short
git commit -am "vendor(lab): 同步实验室代码快照 2026-03-01（来源：xxx）"
git push origin main
git checkout feature/zhongjianyi/multi-comm && git merge main
```

⚠️ 快照方式下**没有 commit hash 可对**，所以 `deps.lock` 里的 `obtained_at` / `obtained_from` **必须认真填**（哪天、谁给的、哪个压缩包）。否则半年后出问题，你无法判断"飞机上这份代码到底是哪个版本"。

> 现在 `deps.lock` 里这几个字段是 `TODO`，**请尽快补上**（你刚拿到这份代码，还记得来源；过两周就忘了）。

---

## 5. 版本一致性：`deps.lock` + tag

比赛现场最耗时间的一类问题是"PC 上跑得通、飞机上跑不通"，九成原因是**两边代码版本不一致**。所以：

1. 换实验室代码 / 换 Multibotnet 版本 → 更新 `deps.lock` 并提交。
2. **上飞机前、起飞前**，在两台飞机上各跑一次：

```bash
git -C ~/SixBotsRace rev-parse HEAD          # 两台必须输出同一个 hash
git -C ~/SixBotsRace status --porcelain -- ego-planner2 MLMapping-Embedded_version FAST_LIO   # 必须为空
bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal   # 顺带跑守卫
```

3. 现场回滚：`git stash -u && git checkout <tag>`。

---

## 6. 部署到飞机

```bash
# ---- 飞机上（一次性）----
cd ~
git clone -b feature/zhongjianyi/multi-comm git@github.com:JianyiZhong/SixBotsRace.git
bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal
source ~/six_ws/devel/setup.bash

# ---- 之后每次更新 ----
cd ~/SixBotsRace && git pull
bash race6_comm/scripts/setup_six_ws.sh --minimal
```

**飞机上没有网络 / 没配 SSH key 时**（现场常见），用 git bundle 离线保证版本一致：

```bash
# PC 上
git -C ~/SixBotsRace bundle create /tmp/sixbots.bundle feature/zhongjianyi/multi-comm
scp /tmp/sixbots.bundle orangepi@<飞机IP>:~/
# 飞机上
cd ~ && git clone -b feature/zhongjianyi/multi-comm sixbots.bundle SixBotsRace
bash ~/SixBotsRace/race6_comm/scripts/setup_six_ws.sh --minimal
```

> 为什么不用 `--copy` 直接拷 `race6_comm`？可以（`setup_six_ws.sh --copy`），但那是"快照"，
> 下次改代码不会同步过去，容易忘。用 git 是唯一能保证两台一致的办法。

---

## 7. 提交粒度与提交信息

- 一个**能独立验证**的改动 = 一个 commit。别把"改脚本 + 改配置 + 改文档"堆成一个巨型 commit。
- 提交信息带前缀，方便以后 `git log --oneline` 扫：

| 前缀 | 用于 |
|---|---|
| `feat(comm):` | 通讯/时间同步的新功能 |
| `fix(comm):` | 修 bug |
| `config:` | 只改 YAML/launch 参数 |
| `docs:` | 文档 |
| `vendor(lab):` | 同步实验室代码快照 |
| `chore:` | 杂项（清理、gitignore） |

- 提交前自查（三条）：

```bash
git status --porcelain -- ego-planner2 MLMapping-Embedded_version FAST_LIO   # 必须为空
git status --short | grep -E 'six_ws|build/|devel/'                          # 必须为空
git diff --stat HEAD                                                         # 眼过一遍改了哪些文件
```

---

## 8. 两件需要你确认的事

### 8.1 仓库可见性（合规）

这个仓库现在是 public 还是 private？**建议设成 private**，或先问过实验室再公开：

- `ego-planner2/` 是**实验室改过的分支**（不是上游原版），属于实验室的成果；
- `MLMapping-Embedded_version/`、`FAST_LIO/` 虽然是开源项目，但里面的 `PCD/`、`Log/` 是**实验室自己采的数据**；
- 比赛前公开自己的方案对你们也没好处。

命令：GitHub 仓库 → Settings → General → Danger Zone → Change visibility。

### 8.2 仓库体积

第一次 `add lab code` 时如果连点云/日志一起提交了，历史里可能已经有几百 MB，之后每台飞机 `git clone` 都会很慢。先量一下：

```bash
du -sh ~/SixBotsRace/.git
git -C ~/SixBotsRace count-objects -vH
git -C ~/SixBotsRace ls-files FAST_LIO/PCD | wc -l
git -C ~/SixBotsRace ls-files FAST_LIO/Log | wc -l
```

如果确实很大，**现在只有 1~2 个 commit，正是重写历史最省事的时候**：

```bash
cd ~/SixBotsRace
git rm -r --cached FAST_LIO/PCD FAST_LIO/Log MLMapping-Embedded_version/3rdPartLib
git commit -m "chore: 不再跟踪点云/日志/第三方构建产物（.gitignore 已覆盖）"

# 想彻底从历史里删掉（仓库只有你在用，安全）：
#   pip install git-filter-repo
#   git filter-repo --path FAST_LIO/PCD --path FAST_LIO/Log --invert-paths
#   git push --force origin main
#   git push --force origin feature/zhongjianyi/multi-comm
```

> 注意：FAST_LIO 的**重定位**需要地图 pcd 文件，真机部署时单独用 scp/U 盘传那一两个文件，
> 不要放进 git。`.gitignore` 里已经加了 `*.pcd`、`FAST_LIO/PCD/`、`FAST_LIO/Log/`、`*.bag`。

---

## 9. 事故处理速查

| 症状 | 原因 | 处理 |
|---|---|---|
| 改了 `race6_comm` 但运行行为没变 | 工作区里是**真副本**而不是软链，改的是另一份 | 删掉 `~/six_ws/src/race6_comm` 再跑 setup（脚本会红字提示，不会让你误删真源） |
| 改了 Python 脚本不生效 | 脚本由 `catkin_install_python` 装到 `devel/lib` | `cd ~/six_ws && catkin_make`；launch/config 是就地读的，不用编 |
| 两台飞机行为不一致 | 两边 commit 不同 | 两边 `git rev-parse HEAD` 对比；见 §5 |
| `git status` 冒出实验室文件的改动 | 手滑 | `git checkout -- ego-planner2 MLMapping-Embedded_version FAST_LIO` |
| `git pull` 拿不到实验室更新 | 只配了 `origin`（你自己的仓库） | 见 §4 加 `upstream` |
| 飞机 `git clone` 很慢/失败 | 仓库历史太大 / 现场无网 | 见 §8.2 瘦身；用 §6 的 git bundle 离线传 |
| 要给实验室交东西 | — | 从 `main` 或 feature 开 PR，只包含 `race6_comm/`；先跑铁律 1 的检查确认零修改 |
