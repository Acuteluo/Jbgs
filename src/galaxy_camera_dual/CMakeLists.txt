cmake_minimum_required(VERSION 3.8)
project(galaxy_camera_dual)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Enforce clean code
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# OpenCV: 用于发布 JPEG 压缩话题(供 rqt 流畅看图)
find_package(OpenCV REQUIRED)

## Core library: Galaxy SDK wrapper + simulated device + multi-camera node (component plugin)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/galaxy_device.cpp
  src/simulated_galaxy_device.cpp
  src/galaxy_camera_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  GalaxySDK/include
  ${OpenCV_INCLUDE_DIRS}
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC GalaxySDK/lib/amd64)
  install(
    DIRECTORY GalaxySDK/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC GalaxySDK/lib/arm64)
  install(
    DIRECTORY GalaxySDK/lib/arm64/
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  gxiapi
  ${OpenCV_LIBRARIES}
)

## Install public headers and the SDK headers they include
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY GalaxySDK/include/ DESTINATION include)

# 交互式相机/传感器配置向导: 独立可执行, 不依赖 ROS 运行时
# (GxIAPI + OpenCV + termios), 用于新机接线检查与左右相机绑定。
add_executable(camera_setup_wizard
  src/setup_wizard.cpp
  src/galaxy_device.cpp)
target_compile_features(camera_setup_wizard PRIVATE cxx_std_17)
target_include_directories(camera_setup_wizard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/GalaxySDK/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})
target_link_directories(camera_setup_wizard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/GalaxySDK/lib/amd64)
target_link_libraries(camera_setup_wizard gxiapi ${OpenCV_LIBRARIES} pthread)
install(TARGETS camera_setup_wizard RUNTIME DESTINATION lib/${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN galaxy_camera_dual::GalaxyCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
