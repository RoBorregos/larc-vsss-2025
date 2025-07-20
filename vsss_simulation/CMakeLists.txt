cmake_minimum_required(VERSION 3.8)
project(vsss_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf_launch REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf scripts worlds launch config include
  DESTINATION share/${PROJECT_NAME})

# find dependencies
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Bullet REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(simulation_spawn scripts/simulation_spawn.cpp)
add_executable(RobotController scripts/RobotController.cpp)

add_library(Kinematic src/vsss_simulation/Kinematic.cpp)
add_library(MathU src/vsss_simulation/MathU.cpp)
add_library(Line src/vsss_simulation/Line.cpp)
add_library(UnivectorF src/vsss_simulation/UnivectorF.cpp)

target_include_directories(simulation_spawn  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(RobotController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(Kinematic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(MathU PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_include_directories(Line PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


  target_include_directories(UnivectorF PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(RobotController rclcpp std_msgs nav_msgs tf2 tf2_geometry_msgs tf2_eigen Bullet) 
target_link_libraries(RobotController Kinematic Line MathU UnivectorF)
ament_target_dependencies(Kinematic tf2)
ament_target_dependencies(Line tf2)
ament_target_dependencies(UnivectorF tf2)
ament_target_dependencies(MathU tf2)

target_compile_features(simulation_spawn PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simulation_spawn RobotController
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
