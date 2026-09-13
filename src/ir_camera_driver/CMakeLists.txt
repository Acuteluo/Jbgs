# ir_camera_driver 包: 红外热成像机芯(UVC/V4L2)驱动节点
#
# 节点职责:
#  - 打开 /dev/videoN 的 UVC 红外机芯(白热 8bit 灰度流, 不依赖厂家 SDK),
#    编码 JPEG 后发布唯一话题 /ir_camera/image_raw/compressed;
#  - 独立采集线程 + 热插拔自愈(打不开/读到坏帧 -> 关闭 -> 每秒重试,
#    设备重新接入后自动恢复), 与大恒相机驱动行为对齐;
#  - sim_mode: 无实体机芯时用模拟热像(移动冷斑/噪声)顶替,
#    插拔时间窗与真机语义一致, 供全链路热插拔测试。
cmake_minimum_required(VERSION 3.10)
project(ir_camera_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 驱动核心库(可被 composition 容器加载)
add_library(${PROJECT_NAME} SHARED
    src/ir_camera_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
)

# 注册组件并生成独立可执行文件 ir_camera_driver_node
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN ir_camera_driver::IrCameraNode
    EXECUTABLE ${PROJECT_NAME}_node
)

# 安装规则: 库/头文件/配置/launch
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
