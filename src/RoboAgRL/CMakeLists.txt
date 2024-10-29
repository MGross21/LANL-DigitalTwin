cmake_minimum_required(VERSION 3.10)
project(RoboAgRL)

# Specify the C++ standard (remove if not using C++)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Uncomment and add any additional packages your project depends on
# find_package(<other_dependency> REQUIRED)

# Include Cpp "include" directory if it exists
include_directories(include)

# Create Cpp executable (uncomment if you have C++ nodes)
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

# Install Cpp executables
# install(TARGETS
#   ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install files
install(DIRECTORY
  launch
  worlds
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
#   install(DIRECTORY
#     config
#     DESTINATION share/${PROJECT_NAME}/
#   )
# endif()

# Install include directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(DIRECTORY
    include/
    DESTINATION include
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()