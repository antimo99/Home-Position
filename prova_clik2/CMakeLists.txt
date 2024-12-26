cmake_minimum_required(VERSION 3.8)
project(prova_clik2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(moveit_ros_planning REQUIRED) # <-- needed to use robot model and robot state in the clik node



include_directories(include)


add_executable(clik src/clik_node_test.cpp)
add_executable(cartesian_trajectory src/cartesian_trajectory.cpp)


target_link_libraries(clik Eigen3::Eigen) # Eigen is not an ament package, we have to link it manually
target_link_libraries(cartesian_trajectory Eigen3::Eigen) # Eigen is not an ament package, we have to link it manually

ament_target_dependencies(clik 
      "rclcpp" "geometry_msgs" "sensor_msgs" "std_srvs" "moveit_ros_planning" )

ament_target_dependencies(cartesian_trajectory 
      "rclcpp" "geometry_msgs" "sensor_msgs" "std_srvs" "moveit_ros_planning" )

########## INSTALL nodes #######################
install(TARGETS
   clik cartesian_trajectory
  DESTINATION lib/${PROJECT_NAME}
)
################################################

########## INSTALL the launch files ############
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
################################################


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