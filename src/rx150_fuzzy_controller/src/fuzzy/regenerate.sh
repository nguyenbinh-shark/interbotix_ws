#!/usr/bin/env bash
set -euo pipefail
GEN_DIR="/home/hust/interbotix_ws/gen_fit_and_3d_graph"
SRC_FIS="$(dirname "$0")/fuzzy_type1.fis"
cp "$SRC_FIS" "$GEN_DIR/fuzzy_type1.fis"
( cd "$GEN_DIR" && python3 fis2c.py fuzzy_type1.fis )
cp "$GEN_DIR/fuzzy_type1.c" "$(dirname "$0")/fuzzy_type1.c"
cp "$GEN_DIR/fuzzy_type1.h" "$(dirname "$0")/fuzzy_type1.h"
echo "Regenerated fuzzy_type1.{c,h}"
