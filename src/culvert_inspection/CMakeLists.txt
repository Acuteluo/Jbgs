# culvert_inspection 包: 巡检通信协议节点(仅本机 ROS2 topic, 不做串口)。
#
# 协议(std_msgs/msg/UInt8):
#   输入 <input_topic>(默认 /inspection/command):
#     0x01 = 一次巡检触发
#   输出 <ack_topic>(默认 /inspection/ack):
#     0x02 = 左右两帧均成功写入 run_save 后发布一次
#
# 帧选择规则: 记录 0x01 到达的单调时刻 t(t 时刻已到达的帧为第 0 帧),
# 对左右相机分别独立地取"t 之后按到达顺序的第 2 帧"; 目标帧解码失败或
# 相机离线时顺延取其后第 3、4…帧; 超时(可配置)放弃本次巡检且不发 0x02。
#
# 组成:
#   include/culvert_inspection/inspection_fsm.hpp  状态机核心(无 ROS 依赖)
#   src/inspection_node.cpp                        rclcpp 薄封装
#   test/test_inspection_fsm.cpp                   状态机 gtest 单元测试

cmake_minimum_required(VERSION 3.8)
project(culvert_inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_executable(inspection_node src/inspection_node.cpp)
target_include_directories(inspection_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(inspection_node rclcpp std_msgs sensor_msgs)
target_link_libraries(inspection_node ${OpenCV_LIBS})

install(TARGETS inspection_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ---- 状态机单元测试(gtest, colcon test / ctest 均可跑) ----
if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_inspection_fsm test/test_inspection_fsm.cpp)
  target_include_directories(test_inspection_fsm PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
endif()

ament_package()
