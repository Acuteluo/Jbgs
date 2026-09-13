# tas_sensor_driver 包: 塔石 TAS-WSCO2 传感器驱动
# 串口用 POSIX termios 直接读写(不依赖 serial_driver 库, 行为完全可控)
cmake_minimum_required(VERSION 3.8)
project(tas_sensor_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# 驱动核心库(可被 composition 容器加载)
add_library(${PROJECT_NAME} SHARED
    src/crc16.cpp
    src/modbus.cpp
    src/tas_sensor_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    sensor_interfaces
    std_msgs
)

# 注册组件并生成独立可执行文件 tas_sensor_driver_node
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN tas_sensor_driver::TasSensorDriver
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
