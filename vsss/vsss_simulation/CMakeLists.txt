cmake_minimum_required(VERSION 3.8)
project(vsss_simulation)

# find dependencies
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Bullet REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf_launch REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)




rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotAction.msg"
  DEPENDENCIES geometry_msgs std_msgs
)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY urdf scripts worlds launch config include
  DESTINATION share/${PROJECT_NAME})



add_executable(simulation_spawn scripts/simulation_spawn.cpp)
add_executable(RobotController scripts/RobotController.cpp)
add_executable(VectorGrapher scripts/VectorGrapher.cpp)
add_executable(Strategist scripts/Strategist.cpp)
add_executable(RobotActionVisualizer scripts/RobotActionVisualizer.cpp)

add_library(MsgConvert src/vsss_simulation/MsgConvert.cpp)
add_library(Kinematic src/vsss_simulation/Kinematic.cpp)
add_library(MathU src/vsss_simulation/MathU.cpp)
add_library(Line src/vsss_simulation/Line.cpp)
add_library(UnivectorF src/vsss_simulation/UnivectorF.cpp)
add_library(Polygon src/vsss_simulation/Polygon.cpp)
# Expose package include dir on libraries so consumers inherit it.
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

target_include_directories(MsgConvert PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(Polygon PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(RobotController rclcpp std_msgs nav_msgs tf2 tf2_geometry_msgs tf2_eigen Bullet) 
target_link_libraries(RobotController Kinematic Line UnivectorF MsgConvert ${cpp_typesupport_target})

ament_target_dependencies(VectorGrapher rclcpp visualization_msgs std_msgs nav_msgs tf2 tf2_geometry_msgs)
target_link_libraries(VectorGrapher UnivectorF MsgConvert Line   ${tf2_geometry_msgs_LIBRARIES} )

ament_target_dependencies(Strategist rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(Strategist Polygon Kinematic Line ${cpp_typesupport_target})

ament_target_dependencies(RobotActionVisualizer rclcpp visualization_msgs std_msgs nav_msgs tf2)
target_link_libraries(RobotActionVisualizer ${cpp_typesupport_target} )

ament_target_dependencies(MsgConvert tf2 tf2_geometry_msgs nav_msgs)
ament_target_dependencies(Kinematic tf2 tf2_geometry_msgs)

target_link_libraries(Kinematic MathU MsgConvert )
ament_target_dependencies(Line tf2)

ament_target_dependencies(UnivectorF tf2)
target_link_libraries(UnivectorF MathU Kinematic)

ament_target_dependencies(MathU tf2)


ament_target_dependencies(Polygon tf2)

target_compile_features(simulation_spawn PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simulation_spawn RobotController VectorGrapher Strategist RobotActionVisualizer
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/DefaultController.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
