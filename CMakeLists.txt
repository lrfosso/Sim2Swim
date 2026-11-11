cmake_minimum_required(VERSION 3.8)
project(sim2swim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


ament_python_install_package(${PROJECT_NAME})


install(PROGRAMS
  sim2swim/sim2swim_node.py
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY sim2swim 
        DESTINATION share/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(sim2swim src/sim2swim.cpp)
# target_include_directories(sim2swim PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(sim2swim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# 
# install(TARGETS sim2swim
#   DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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
