#!/usr/bin/env bash
# Source đủ 4 layer để build/run stack fuzzy_controller + camera + hand-eye.
# Dùng: source ~/interbotix_ws/source_all.sh
#
# Thứ tự: ROS underlay -> apriltag_ws -> easy_handeye2_ws -> interbotix_ws
set -e
ROS_DISTRO="${ROS_DISTRO:-humble}"
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f "$HOME/apriltag_ws/install/setup.bash" ] && source "$HOME/apriltag_ws/install/setup.bash"
[ -f "$HOME/easy_handeye2_ws/install/setup.bash" ] && source "$HOME/easy_handeye2_ws/install/setup.bash"
[ -f "$HOME/interbotix_ws/install/setup.bash" ] && source "$HOME/interbotix_ws/install/setup.bash"
echo "[source_all] ros=${ROS_DISTRO} + apriltag_ws + easy_handeye2_ws + interbotix_ws"
