cmake_minimum_required(VERSION 3.8)
project(task2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)
# # 找到 urdf 和 xacro 包
# find_package(urdf REQUIRED)
# # find_package(launch REQUIRED)
# find_package(xacro REQUIRED)
# find_package(gazebo_ros REQUIRED)

# 上一个项目
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED) # 添加openCV作为依赖项

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

# 增加编译规则 发布者 
# 2024年10月15日: 添加依赖opencv4 cv_bridge
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
# 订阅者
add_executable(listener src/subscriber_member_function.cpp )
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

# 增加安装规则
install(TARGETS
  talker
  listener # 订阅者
  DESTINATION lib/${PROJECT_NAME})

  
# 安装 URDF launch文件
install(DIRECTORY urdf launch config world maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
