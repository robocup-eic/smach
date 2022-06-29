cmake_minimum_required(VERSION 2.8.3)
project(cr3_moveit_control)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other
# catkin packages
find_package(catkin REQUIRED
             COMPONENTS controller_manager
                        effort_controllers
                        gazebo_ros
                        joint_state_controller
                        joint_state_publisher
                        joint_trajectory_controller
                        robot_state_publisher
                        roscpp
			                  rospy
			                  std_msgs
			                  message_generation  
                        tf
                        xacro
                        moveit_core
                        moveit_visual_tools
                        moveit_ros_planning
                        moveit_ros_planning_interface
                        controller_interface
                        hardware_interface
                        rosbridge_library
                        rosbridge_server
                        rosbridge_msgs
                        rosapi
                        tf2_web_republisher
                        moveit_servo
			)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
message(${JSONCPP_LIBRARIES})

# ################################################################################################################################
# catkin specific configuration ##
add_service_files(FILES cr3_pick.srv
                        cr3_place.srv
                        PickWithSide.srv
                        )

generate_messages(DEPENDENCIES std_msgs
  geometry_msgs)
# ################################################################################################################################
# The catkin_package macro generates cmake config files for your package Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS
               message_runtime
               moveit_core
               moveit_visual_tools
               moveit_ros_planning_interface
               controller_interface
               hardware_interface
               pluginlib
               moveit_servo
               DEPENDS
               )
# ################################################################################################################################
# Build ##
# ################################################################################################################################

# Specify additional locations of header files Your package locations should be listed before other locations
catkin_install_python(PROGRAMS 
                      src/pick_client_python.py
                      src/pick_client_select_side.py
                      src/place_client_python.py
                      src/furniture_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(${catkin_INCLUDE_DIRS})

add_executable(kan_algorithm_grasp src/kan_algorithm_grasp.cpp)
target_link_libraries(kan_algorithm_grasp ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})

add_executable(robot_control_node_sim src/robot_control_node_sim.cpp)
target_link_libraries(robot_control_node_sim ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})

add_executable(robot_control_node_sim_catesian src/robot_control_node_sim_catesian.cpp)
target_link_libraries(robot_control_node_sim_catesian ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})

add_executable(robot_control_node_real src/robot_control_node_real.cpp)
target_link_libraries(robot_control_node_real ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})

add_executable(robot_control_node_real_catesian src/robot_control_node_real_catesian.cpp)
target_link_libraries(robot_control_node_real_catesian ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})

add_executable(visual_servo src/visual_servo.cpp)
target_link_libraries(visual_servo ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})

add_executable(pick_service_select_side src/pick_service_select_side.cpp)
target_link_libraries(pick_service_select_side ${catkin_LIBRARIES})

add_executable(pick_service_adjust_yaw src/pick_service_adjust_yaw.cpp)
target_link_libraries(pick_service_adjust_yaw ${catkin_LIBRARIES})

add_executable(pick_service_visual_servo src/pick_service_visual_servo.cpp)
target_link_libraries(pick_service_visual_servo ${catkin_LIBRARIES})

add_executable(place_algorithm_demo src/place_algorithm_demo.cpp)
target_link_libraries(place_algorithm_demo ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})
add_dependencies(place_algorithm_demo cr3_moveit_control)

add_executable(pick_client src/pick_client.cpp)
target_link_libraries(pick_client ${catkin_LIBRARIES})

add_executable(constraint_publisher src/constraint_publisher.cpp)
target_link_libraries(constraint_publisher ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})

