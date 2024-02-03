
cmake_minimum_required(VERSION 3.8)
project(utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(utilities
        src/Client.cpp
        src/Publisher.cpp
        src/Service.cpp
        src/Subscriber.cpp
        src/Timer.cpp
)

ament_export_targets(utilities HAS_LIBRARY_TARGET)
ament_target_dependencies(utilities
        rclcpp
)

install(
        DIRECTORY include
        DESTINATION include/utilities
)

install(
        TARGETS utilities
        EXPORT utilities
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/utilities
)

ament_package()
