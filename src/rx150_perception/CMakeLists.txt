cmake_minimum_required(VERSION 3.8)
project(rx150_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Node nhận diện (Python, chạy như program — giống pattern fuzzy_controller).
# Pick-place đã chuyển sang package riêng rx150_pick_place (Layer 2).
install(
  PROGRAMS
    scripts/yolo_detector_node.py
    scripts/hand_gesture_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
