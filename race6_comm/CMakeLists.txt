cmake_minimum_required(VERSION 3.0.2)
project(race6_comm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  controller_msgs
)

catkin_package()

# ---------------------------------------------------------------------------
# Python 脚本
#   catkin_install_python 会把脚本装到 devel/lib/race6_comm/ 并写好 shebang，
#   因此修改脚本后必须重新 catkin_make 才生效。
# ---------------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/race6_time_sync.py
  scripts/race6_comm_test.py
  scripts/race6_mintraj_test.py
  scripts/check_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
# 配置 / 启动文件 / 工作区搭建脚本
#   这些是「就地读取」的，改完不需要 catkin_make。
# ---------------------------------------------------------------------------
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/setup_six_ws.sh
  scripts/gen_race6_configs.py
  scripts/race6_launch.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
