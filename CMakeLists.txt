cmake_minimum_required(VERSION 3.8)
project(nora)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# use ORIGIN so executable looks for .so files in same folder
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nora_idl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(NATNET_LIB NatNet PATHS ${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/lib)

set(NORA_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/ConfigManager.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/OptitrackStateFetcher.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/RobotStateTracker.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${PROJECT_NAME} ${NORA_SOURCES})

include_directories(${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/NatNetSDK4.1/include
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
)

ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3 nora_idl)
target_link_libraries(${PROJECT_NAME} yaml-cpp ${NATNET_LIB})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY configuration/ DESTINATION share/nora/configuration)
install(FILES dependencies/NatNetSDK4.1/lib/libNatNet.so DESTINATION lib/${PROJECT_NAME})

ament_package()
