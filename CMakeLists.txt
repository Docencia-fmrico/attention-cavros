cmake_minimum_required(VERSION 3.5)
project(attention-cavros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    control_msgs
    trajectory_msgs
    gazebo_msgs
    tf2_msgs
    ros2_knowledge_graph
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/attention-cavros/HeadController.cpp
    src/attention-cavros/Detector.cpp
    src/attention-cavros/PoseInMap.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(attention_main src/attention_main.cpp)
target_link_libraries(attention_main ${PROJECT_NAME})

install(DIRECTORY include config launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    attention_main
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
