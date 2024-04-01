
cmake_minimum_required(VERSION 3.8)
project(rosa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/actionclient/ActionClient.cpp
        src/actionservice/ActionService.cpp
        src/client/Client.cpp
        src/node/Node.cpp
        src/publisher/Publisher.cpp
        src/service/Service.cpp
        src/subscriber/Subscriber.cpp
        src/timer/Timer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_action
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
        rclcpp
        rclcpp_action
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/
)

ament_package()
