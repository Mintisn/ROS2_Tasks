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
find_package(ament_cmake REQUIRED)

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


# 增加安装规则


  
# 安装 URDF launch文件
install(DIRECTORY urdf launch config world maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
