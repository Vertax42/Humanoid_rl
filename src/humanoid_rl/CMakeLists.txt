cmake_minimum_required(VERSION 3.0.2)
project(humanoid_rl)

set(CMAKE_BUILD_TYPE "Debug")
## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall")
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

find_package(Torch REQUIRED HINTS "/home/xbot/libtorch")
find_package(Eigen3 REQUIRED)
# Catkin-specific configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs
)

# Include directories
include_directories(
  include
  include/common_tools/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TORCH_INCLUDE_DIRS}
)

# add_library(utils_common SHARED
#   ./include/common_tools/utils_common.cpp
# )

# target_link_libraries(utils_common
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${EIGEN3_LIBRARIES}
#   ${TORCH_LIBRARIES}
# )
# Declare the executable
add_executable(${PROJECT_NAME}_node
  src/humanoid_policy.cpp
  src/humanoid_rl_inference.cpp
  src/humanoid_state_machine.cpp
  src/body_control.cpp
  include/common_tools/log4z.cpp
  include/common_tools/utils_common.cpp
)

# Add target dependencies
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${TORCH_LIBRARIES}
)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

