# 文件: tests/CMakeLists.txt
# 作用: 系统级自动化测试(独立 cmake 工程, 非 ROS 包)。
#
# 测试项(ctest 编排, 顺序即编号):
#   01_clean_build      测试 1: ./run.sh --build-only 干净重建
#   02_full_sim         测试 2+8: 全模拟启动, 全话题可观测, 关停无残留
#   03_missing_left     测试 3: 初始缺左相机, 其余持续工作
#   03_missing_right    测试 3: 初始缺右相机
#   03_missing_ir       测试 3: 初始缺红外
#   03_missing_sensor   测试 3: 初始缺传感器
#   04_hotplug          测试 4: 四设备运行中断连/恢复 + 离线标注 + 节流日志
#   05_protocol         测试 5+6: 巡检协议(选帧/重复触发/顺延/超时/写盘失败)
#   07_off_mode         测试 7: in_trulyworking=false 不处理不保存
#
# 运行: 见 run_tests.sh(source 环境后 cmake+ctest)。

cmake_minimum_required(VERSION 3.16)
project(jbgs_system_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_executable(sys_full_sim system/sys_full_sim.cpp)
add_executable(sys_initial_missing system/sys_initial_missing.cpp)
add_executable(sys_hotplug system/sys_hotplug.cpp)
add_executable(sys_protocol system/sys_protocol.cpp)
add_executable(sys_off_mode system/sys_off_mode.cpp)

# 小工具: 等价 ament_target_dependencies(避免本工程引入 ament)
function(ament_target_dependencies_custom target)
  target_include_directories(${target} PRIVATE
    ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}
    ${sensor_interfaces_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
  target_link_libraries(${target}
    ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES}
    ${sensor_interfaces_LIBRARIES} ${std_msgs_LIBRARIES})
endfunction()

foreach(t sys_full_sim sys_initial_missing sys_hotplug sys_protocol
          sys_off_mode)
  target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  ament_target_dependencies_custom(${t})
  target_link_libraries(${t} ${OpenCV_LIBS})
endforeach()



enable_testing()

add_test(NAME 01_clean_build
  COMMAND bash -c "cd ${CMAKE_CURRENT_SOURCE_DIR}/.. && ./run.sh --build-only")
set_tests_properties(01_clean_build PROPERTIES TIMEOUT 900)

add_test(NAME 02_full_sim COMMAND sys_full_sim)
set_tests_properties(02_full_sim PROPERTIES TIMEOUT 180)

foreach(dev left right ir sensor)
  add_test(NAME 03_missing_${dev}
    COMMAND sys_initial_missing ${dev})
  set_tests_properties(03_missing_${dev} PROPERTIES TIMEOUT 180)
endforeach()

add_test(NAME 04_hotplug COMMAND sys_hotplug)
set_tests_properties(04_hotplug PROPERTIES TIMEOUT 240)

add_test(NAME 05_protocol COMMAND sys_protocol)
set_tests_properties(05_protocol PROPERTIES TIMEOUT 240)

add_test(NAME 07_off_mode COMMAND sys_off_mode)
set_tests_properties(07_off_mode PROPERTIES TIMEOUT 180)

# 统一测试环境(域隔离, 避免与机器上其他 ROS 会话互相干扰)
set_tests_properties(
  02_full_sim 03_missing_left 03_missing_right 03_missing_ir
  03_missing_sensor 04_hotplug 05_protocol 07_off_mode
  PROPERTIES ENVIRONMENT
  "JBGS_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/..;ROS_DOMAIN_ID=93")
