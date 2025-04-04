cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package (rclcpp REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)

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

# let the compiler search for headers in the include folder
include_directories(include ${EIGEN_INCLUDE_DIRS})
 
# define a library target 
add_library( kinematics 
            src/kin.cpp 
            src/safety.cpp 
            src/fkin.cpp 
            src/ikin.cpp 
            src/fir_filter.cpp 
            src/iir_filter.cpp
            src/eigen_support.cpp)

# test program to test functionality of kinematics functions. 
#add_executable(kin_tester src/test.cpp src/safety.cpp src/kin.cpp)
#target_link_libraries(kin_tester Eigen3::Eigen)

add_executable(robot_param_server src/robot_param_server.cpp)
ament_target_dependencies(robot_param_server rclcpp)

target_link_libraries (kinematics Eigen3::Eigen)


# this line to exports the library
ament_export_targets(kinematics HAS_LIBRARY_TARGET)
 
# install the include/kinematics directory to install/include/kinematics
install(
  DIRECTORY include/kinematics
  DESTINATION include
)

install(
  TARGETS kinematics
  EXPORT kinematics
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  robot_param_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
