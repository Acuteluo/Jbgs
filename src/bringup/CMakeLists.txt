# bringup 包: 纯启动包(没有 C++ 代码), 只安装 launch 目录。
# 仿照旧项目 bringup 包的写法: find_package(ament_cmake) + install(launch)。
cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 把 launch 目录安装到 share/bringup/launch 下,
# 之后即可用 ros2 launch bringup xxx.launch.py 启动。
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
