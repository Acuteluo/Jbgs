# sensor_interfaces 包: 传感器接口定义(仿旧项目 serial_driver_interfaces 的写法)
#
# 只生成 ROS2 接口, 不包含任何运行时代码:
#   - msg/EnvData.msg  环境数据消息(实时话题 /sensor/env 用)
#   - srv/QueryEnv.srv 按需查询服务(需要时一次读取并返回)
cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成接口代码(C++/Python 头文件与类型支持)
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/EnvData.msg"
    "srv/QueryEnv.srv"
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
