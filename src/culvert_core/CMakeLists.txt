# culvert_core 包: 涵洞裂缝检测核心节点框架
#
# 说明:
#  - 本包模仿旧项目 img_processing 的 core_node 结构: 头文件声明类,
#    cpp 里 InitParams()/InitROS2() 分步初始化, main 里 rclcpp::spin;
#  - 目前框架只做"订阅图像/环境数据 + 节流打印", 裂缝检测等核心
#    逻辑留了明确的 TODO 位置(ProcessFrame 里的 CoreLogic);
#  - 后续需要 Eigen/message_filters/tf2 时, 在 find_package、
#    package.xml 与 ament_target_dependencies 里同步追加即可。
cmake_minimum_required(VERSION 3.10)
project(culvert_core)

# C++17(与旧项目保持一致)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# 传感器接口包(温湿度/CO2 消息)
find_package(sensor_interfaces REQUIRED)

# OpenCV: JPEG 图像解码(compressed 模式, 检测链路的推荐输入)
find_package(OpenCV REQUIRED)
# 无边框铺满用 X11 Motif hints(不用 GTK FULLSCREEN, 避免 Mutter 闪黑)
find_package(X11)

# 核心节点可执行文件
add_executable(core_node
    src/core_node.cpp
    src/yolo_detector.cpp
    src/yolo_crack.cpp
    src/ir_seepage_detector.cpp
)

target_include_directories(core_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(core_node
    rclcpp
    sensor_msgs
    std_msgs
    sensor_interfaces
)

# 链接第三方库
target_link_libraries(core_node
    ${OpenCV_LIBRARIES}
)
if(X11_FOUND)
    target_compile_definitions(core_node PRIVATE JBGS_HAS_X11=1)
    target_include_directories(core_node PRIVATE ${X11_INCLUDE_DIR})
    target_link_libraries(core_node ${X11_LIBRARIES})
endif()

# ---- 单图 YOLO 检测测试工具(与 core_node 共用 yolo_detector/yolo_crack) ----
# 用法: ros2 run culvert_core test_yolo <图片路径> [模型路径] [conf阈值]
add_executable(test_yolo
    src/test_yolo.cpp
    src/yolo_detector.cpp
    src/yolo_crack.cpp
)

target_include_directories(test_yolo PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(test_yolo
    rclcpp
)

target_link_libraries(test_yolo
    ${OpenCV_LIBRARIES}
)

# 安装规则: 可执行文件 + 参数 yaml(供 bringup 的 launch 引用)
install(TARGETS core_node test_yolo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # ---- 红外渗水检测器单元测试(时间一致性滑窗, gtest) ----
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_ir_seepage_detector
        test/test_ir_seepage_detector.cpp
        src/ir_seepage_detector.cpp
    )
    target_include_directories(test_ir_seepage_detector PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(test_ir_seepage_detector ${OpenCV_LIBRARIES})
endif()

ament_package()
