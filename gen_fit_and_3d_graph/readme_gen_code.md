cd /home/hust/interbotix_ws/gen_fit_and_3d_graph

# 1. Sinh code C từ .fis   → fuzzy_type1.h / .c / _demo.c
python3 fis2c.py fuzzy_type1.fis

# 2. Vẽ lại mặt 3D          → surface.json → fuzzy_surface.html
python3 gen_surface.py && python3 build_artifact.py
xdg-open fuzzy_surface.html
