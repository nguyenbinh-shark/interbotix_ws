#!/usr/bin/env bash
# Record ros2 bag 5 topic fuzzy để so sánh A/B (step vs profile) và replay trong PlotJuggler.
# Dùng:  ./config/fuzzy_record.sh [tên_bag]      (mặc định: fuzzy_run)
# VD:    ./config/fuzzy_record.sh step           (chạy enable_profile:=false)
#        ./config/fuzzy_record.sh profile        (chạy enable_profile:=true)
# Dừng: Ctrl+C -> PJ: Data -> Load -> ROS2 bag mở cả 'step' và 'profile' để overlay.
set -euo pipefail
NAME="${1:-fuzzy_run}"
ros2 bag record -o "$NAME" \
  /rx150/joint_states \
  /rx150/fuzzy/reference \
  /rx150/fuzzy/effort \
  /rx150/fuzzy/error \
  /rx150/fuzzy/edot
