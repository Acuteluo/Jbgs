cmake_minimum_required(VERSION 3.10)
project(ir_seepage_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(ir_seepage_demo
    src/ir_seepage_demo.cpp
    src/ir_seepage_detector.cpp
)
target_include_directories(ir_seepage_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ir_seepage_demo PRIVATE ${OpenCV_LIBS})
