## 基于ROS2的智能协同系统

### 任务一: 基础

> **任务**
>
> + 配置ubuntu环境，注意ROS版本与ubuntu版本间的对应关系
>
> + 安装ROS 2框架
>
> + 使用Publisher-Subscriber结构，完成两个node节点间传输数据功能
>   + 传输文字与视频



#### STEP: 配置Ubuntu环境

安装VMware

[安装虚拟机（VMware）保姆级教程（附安装包）_vmware虚拟机-CSDN博客](https://blog.csdn.net/weixin_74195551/article/details/127288338)

下载 Ubuntu 22.04 LTS 镜像文件 [Alternative downloads | Ubuntu](https://ubuntu.com/download/alternative-downloads) 选择22.04的bittorrent文件,使用qbittorrent等软件下载就好.

配置好虚拟机

**可能遇到的问题**

[VMware Workstation 17 运行虚拟机总是闪屏，有时还导致原系统死机/黑屏_vmware虚拟机不断闪屏-CSDN博客](https://blog.csdn.net/m0_68736501/article/details/129243462)



#### **STEP: 安装ROS2框架**

按照这个步骤来  [ubuntu22.04安装ROS2 详细教程-CSDN博客](https://blog.csdn.net/shenliu128/article/details/127296318)



如果遇到这个问题: [关于ROS安装及ROS定位不到软件包的问题解决_rosdep install --rosdistro melodic --ignore-src ---CSDN博客](https://blog.csdn.net/YMMMAR/article/details/122142925#:~:text=检查网络连接是否正常，可以尝试使用以下命令更新 软件包 列表： ``` sudo apt-get update ```,源。 3. 检查 软件包 名称是否正确，有时候输入错误的 软件包 名称也会导致 定位不到软件包。)

可以尝试更换一下网络环境, 连接热点



#### **STEP: ROS2入门**

[ROS2——创建ROS2工作空间_ros2创建工作空间-CSDN博客](https://blog.csdn.net/hx1113/article/details/121972551)



跟着这个教程, 实现一个简单的订阅者和发布者:

[ROS2入门教程—创建一个简单的订阅者和发布者（C++版）_ros2 c++-CSDN博客](https://blog.csdn.net/qq_29923461/article/details/120413799)



过程中,可能需要安装`rosdep` 和 `colcon` :



> ## 安装colcon
>
> `colcon` 是用于构建 ROS 2 工作空间的工具，如果你的系统提示 `colcon: command not found`，说明你还没有安装 `colcon`。你可以按照以下步骤来安装并配置 `colcon`。
>
> ### 1. 安装 `colcon`
> 可以通过包管理器安装 `colcon`。首先，确保你的系统已经安装了 ROS 2，并且你的系统源已经配置好。
>
> #### Ubuntu 下安装 `colcon`：
> 执行以下命令来安装 `colcon` 和相关的构建工具：
>
> ```bash
> sudo apt update
> sudo apt install python3-colcon-common-extensions
> ```
>
> 这个命令会安装 `colcon` 以及一些常见的扩展，如用于自动补全的 `colcon-cmake`、`colcon-ament-cmake` 等。
>
> ### 2. 验证安装
> 安装完成后，你可以通过以下命令验证是否成功安装 `colcon`：
>
> ```bash
> colcon --version
> ```
>
> 如果命令输出 `colcon` 的版本号，则表示安装成功。**按照gpt步骤做到这里就ok了**, 之后跟着博客的步骤走 
>
> ### 3. 设置环境变量（可选）
> 确保你已经正确地设置了 ROS 2 的环境变量。你可以通过在终端运行以下命令来加载 ROS 2 的环境：
>
> ```bash
> source /opt/ros/<your_ros2_distro>/setup.bash
> ```
>
> 将 `<your_ros2_distro>` 替换为你正在使用的 ROS 2 发行版名称，比如 `foxy` 或 `humble`。
>
> 为了每次启动终端时自动加载这些环境变量，你可以将这行命令添加到 `~/.bashrc` 文件中：
>
> ```bash
> echo "source /opt/ros/<your_ros2_distro>/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```
>
> ### 4. 构建 ROS 2 工作空间
> 安装完成 `colcon` 之后，你就可以在工作空间中使用它来构建功能包。比如进入你的工作空间：
>
> ```bash
> cd ~/dev_ws
> colcon build
> ```
>
> 这将会自动构建你在 `src` 目录中的所有 ROS 2 功能包。
>
> ### 总结
> - 通过 `sudo apt install python3-colcon-common-extensions` 安装 `colcon`。
> - 确保正确加载 ROS 2 的环境变量。
> - 使用 `colcon build` 来构建你的工作空间。



在每次编译前, 需要确认功能包的依赖项是否安装好, 在dev_ws路径运行

```bash
rosdep install -i --from-path src --rosdistro humble -y
```



最后 实现两个节点的通信

```bash
. install/setup.bash
ros2 run cpp_pubsub listener
```



> ## 为什么需要. install/setup.bash
>
> 在 ROS 2 中，`. install/setup.bash` 是一个关键步骤，用于设置你工作空间中的环境变量，确保系统能够正确找到并运行你已经构建好的功能包。具体来说，这个步骤的主要作用如下：
>
> ### 1. 添加安装路径到环境变量
>
> 在你使用 `colcon build` 构建功能包时，构建的二进制文件和库会被安装到工作空间的 `install` 目录中。通过运行 `. install/setup.bash`，你会将 `install` 目录中的路径添加到你的环境变量（如 `PATH`、`LD_LIBRARY_PATH`、`PYTHONPATH`、`AMENT_PREFIX_PATH` 等），使系统可以找到这些文件。
>
> ### 2. 为什么需要 `source install/setup.bash`
>
> 在你运行 `ros2 run cpp_pubsub listener` 这样的命令时，ROS 2 需要知道：
> - 从哪里加载功能包。
> - 从哪里找到依赖库和其他资源（例如，已编译的节点、消息定义等）。
>
> 当你运行 `. install/setup.bash`（相当于 `source install/setup.bash`），这会在当前的 shell 中执行脚本，更新你的环境变量，以便 `ros2` 命令能够找到你的功能包以及所有依赖项。
>
> 如果不运行这个步骤，ROS 2 可能找不到你刚刚构建的功能包，导致无法正确执行 `ros2 run` 命令。
>
> ### 3. 什么时候需要运行 `install/setup.bash`
>
> - **首次构建后**：当你第一次通过 `colcon build` 构建了工作空间，你需要运行 `. install/setup.bash` 来让系统知道新的功能包和二进制文件的位置。
> - **每次新打开一个终端时**：在每个新的终端会话中，环境变量不会自动保留。因此，每次打开一个新的终端后，如果你想运行工作空间中的功能包，你需要再次运行 `. install/setup.bash`。
>
> ### 4. 使 `install/setup.bash` 自动加载（可选）
>
> 如果你不想每次都手动运行这条命令，你可以将它添加到你的 `~/.bashrc` 文件中，以便每次打开新终端时自动加载：
>
> ```bash
> echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```
>
> ### 总结
>
> - `. install/setup.bash` 的目的是将你的工作空间安装路径添加到环境变量中。
> - 它确保你能够使用 `ros2` 命令找到和运行你构建的功能包。
> - 每次打开新终端时，你都需要重新运行这条命令，或者将其加入 `~/.bashrc` 文件以自动执行。



按照上述步骤我们实现了两个node节点间传输文本的功能, 下一步需要实现传输视频的功能.

#### **STEP 两个节点间传输视频**

需要检查自己是否安装了`opencv2`(或者说`opencv4`) 和 `cv_bridge`, 要想实现视频的传输, 我们需要这两个库.

合理使用prompt, 让gpt根据[之前的基础模板](https://blog.csdn.net/qq_29923461/article/details/120413799)参考实现一个传输视频的发布者和订阅者.

完善代码后, 需要和之前一样添加并完善依赖项:

```xml
<!-- package.xml   -->
<!-- 添加依赖项 -->
  <buildtool_depend>rclcpp</buildtool_depend>
  <buildtool_depend>std_msgs</buildtool_depend>
  <buildtool_depend>sensor_msgs</buildtool_depend>
  <buildtool_depend>cv_bridge</buildtool_depend>
  <buildtool_depend>OpenCV</buildtool_depend>
```



```cmake
# CMakeLists.txt
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED) # 添加openCV作为依赖项

# 增加编译规则 发布者 
# 2024年10月15日: 添加依赖opencv4 cv_bridge
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
# 订阅者
add_executable(listener src/subscriber_member_function.cpp )
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
```

编译:

```bash
colcon build --packages-select cpp_pubsub
```

编译完成后分别运行`listener `和`talker`

```bash
. install/setup.bash
ros2 run cpp_pubsub listener
```

我们就可以看到虚拟机上出现视频流了(如果要视频流畅, 应该将定时器设置为40ms或以下)

![image-20241016142336530](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241016142336530.png)



### 任务二: 仿真

TODO

> + 完成URDF、rviz、Gazebo的探索学习
>+ 创建激光雷达，使用URDF创建自定义模型，在Gazebo中搭建仿真环境，在rviz模拟器中使用键盘完成对小车的控制，并实时在Gazebo中完成构图
> 
>[URDF+Gazebo+Rviz仿真_gazebo模型显示在rviz-CSDN博客](https://blog.csdn.net/weixin_43903639/article/details/126593762)
>
>[ROS_blinkroad的博客-CSDN博客](https://blog.csdn.net/ost_csdn/category_9655312.html)
>   



新建一个功能包task2, 添加依赖项: 

> 你需要在功能包的 `package.xml` 和 `CMakeLists.txt` 中配置依赖项，以便让你的功能包能够使用 `urdf` 和 `xacro` 这些库。
>
> ### 1. **在 `package.xml` 中添加依赖**
>
> 在 `package.xml` 文件中，你需要明确声明你的功能包依赖于 `urdf` 和 `xacro`。这可以确保在构建和运行时这些依赖项会被自动处理。
>
> 打开 `package.xml` 文件，在合适的地方（通常是在 `<build_depend>` 和 `<exec_depend>` 部分）添加如下依赖：
>
> ```xml
> <package format="3">
>   <!-- 其他已有内容 -->
>   
>   <!-- 添加 urdf 和 xacro 依赖 -->
>   <build_depend>urdf</build_depend>
>   <exec_depend>urdf</exec_depend>
>   
>   <build_depend>xacro</build_depend>
>   <exec_depend>xacro</exec_depend>
> 
>   <!-- 其他已有内容 -->
> </package>
> ```
>
> 解释：
> - `<build_depend>`：功能包在编译时所依赖的包。
> - `<exec_depend>`：功能包在运行时所依赖的包。
>
> ### 2. **在 `CMakeLists.txt` 中添加依赖**
>
> 在 `CMakeLists.txt` 中，你需要使用 `find_package` 来找到 `urdf` 和 `xacro` 包，以确保它们在构建过程中被正确加载和使用。
>
> 打开 `CMakeLists.txt`，添加如下内容：
>
> ```cmake
> cmake_minimum_required(VERSION 3.5)
> project(my_robot_package)
> 
> # 找到 urdf 和 xacro 包
> find_package(urdf REQUIRED)
> find_package(xacro REQUIRED)
> 
> # 其他已有内容 ...
> 
> # 安装 URDF 文件
> install(DIRECTORY urdf
>   DESTINATION share/${PROJECT_NAME}/
> )
> 
> # 其他已有内容 ...
> ```
>
> 解释：
> - `find_package(urdf REQUIRED)` 和 `find_package(xacro REQUIRED)` 会确保在构建时找到这两个包。
> - `install(DIRECTORY urdf DESTINATION ...)` 语句用于确保 `urdf` 文件夹中的 URDF 文件在安装包时被正确处理。
>
> ### 3. **重新构建功能包**
>
> 在修改完 `package.xml` 和 `CMakeLists.txt` 之后，返回到工作空间的根目录，然后重新构建功能包：
>
> ```bash
> cd ~/dev_ws
> colcon build
> ```
>
> 通过这些步骤，你的功能包将正确依赖 `urdf` 和 `xacro`，并且能够在启动时处理这些文件。



如果没有安装`xacro`, 先去安装

> 这个错误提示 CMake 找不到 `xacro` 包的配置文件，通常是因为 `xacro` 没有安装或没有被正确添加到 CMake 的查找路径中。可以按照以下步骤来解决问题：
>
> ### 1. **确认 `xacro` 是否已经安装**
>
> 首先，确保 `xacro` 已经正确安装。可以使用以下命令检查：
>
> ```bash
> ros2 pkg list | grep xacro
> ```
>
> 如果 `xacro` 没有安装，可以使用以下命令来安装它：
>
> ```bash
> sudo apt update
> sudo apt install ros-humble-xacro
> ```
>
> （如果你使用的不是 ROS2 Humble 版本，将 `humble` 替换为你的 ROS2 版本，比如 `foxy` 或 `galactic`。）
>
> ### 2. **设置 `xacro_DIR` 环境变量**
>
> 如果 `xacro` 已经安装，但 CMake 仍然找不到它，可以通过手动设置 `xacro_DIR` 来指定其路径。
>
> 首先，找到 `xacro` 的安装位置。使用以下命令查找 `xacro` 包的安装目录：
>
> ```bash
> ros2 pkg prefix xacro
> ```
>
> 这个命令会返回 `xacro` 包的安装路径，比如 `/opt/ros/humble`. 然后在你的 CMake 构建过程中，设置 `xacro_DIR` 环境变量：
>
> ```bash
> export xacro_DIR=/opt/ros/humble/share/xacro/cmake
> ```
>
> ### 3. **修改 `CMakeLists.txt`**
>
> 在你的 `CMakeLists.txt` 中，尝试使用 `ament_cmake` 来寻找 `xacro`，而不是直接调用 `find_package(xacro REQUIRED)`：
>
> ```cmake
> find_package(ament_cmake REQUIRED)
> find_package(xacro REQUIRED)
> ```
>
> 这可以确保 CMake 使用 `ament_cmake` 查找 ROS2 中的包，确保依赖项能够正确被识别和使用。
>
> ### 4. **重新构建功能包**
>
> 在确认 `xacro` 已安装并设置了正确路径后，重新构建你的功能包：
>
> ```bash
> cd ~/dev_ws
> colcon build
> ```
>
> ### 总结：
>
> - 确保 `xacro` 已经安装。
> - 使用 `ros2 pkg prefix xacro` 找到 `xacro` 包的路径，并设置 `xacro_DIR` 环境变量。
> - 修改 `CMakeLists.txt` 使用 `ament_cmake`。
> - 重新构建功能包。
>
> 这样应该可以解决 CMake 找不到 `xacro` 的问题。



```bash
colcon build
source install/setup.bash
ros2 launch task2 display_robot.launch.py
```

目前来说, 我只能配置`py`的launch文件, 并且启动launch文件, 这样也就能在RViz中显示机器人

![image-20241031165219024](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241031165219024.png)

启动之后添加RobotModel, 修改它的`description file`(找到自己的`urdf`文件)

同时我们还需要把`Fixed frame` 修改为"base_link"

这样就可以显示[URDF+Gazebo+Rviz仿真_gazebo模型显示在rviz-CSDN博客](https://blog.csdn.net/weixin_43903639/article/details/126593762) 中提到的机器人模型了.

> 教程的在这一步走给出的做法不work, 所以参考了这个视频的介绍
> [《ROS 2机器人开发从入门到实践》6.2.2 在RViz中显示机器人_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1Vq2LYoECn/?spm_id_from=333.788.videopod.sections&vd_source=6aef8a8b18d581883e4331f31b8d68f0)



> TODO: 目前还没有解决配置文件的问题,加载相关的config.rviz文件时,仍然需要使用绝对路径
>
> 解决方法:[《ROS 2机器人开发从入门到实践》4.6.3 launch使用进阶_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1BMbAeKEY8?spm_id_from=333.788.videopod.sections&vd_source=6aef8a8b18d581883e4331f31b8d68f0)
>
> ![image-20241105180115682](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241105180115682.png)



#### 学习使用xacro

学习[《ROS 2机器人开发从入门到实践》6.2.3使用Xacro简化URDF_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1WY2WYaEwc?spm_id_from=333.788.videopod.sections&vd_source=6aef8a8b18d581883e4331f31b8d68f0)

```
xacro /home/zychen/dev_ws/src/task2/urdf/first_robot.xacro
```

使用xacro命令生成urdf格式文件

然后设计一个组装的机器人

![image-20241112150929432](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241112150929432.png)

依次完成以下步骤:

+ 添加传感器,执行器和虚拟部件(贴合地面)
+ 给机器人部件添加物理属性(碰撞属性)

![image-20241113175830465](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241113175830465.png)

对比发现碰撞属性的外观和视觉属性一致

+ 为机器人添加质量和惯性
+ 在gazebo中加载机器人并使用Gazebo标签拓展urdf

![image-20241113195104843](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241113195104843.png)

+ 使用两轮差速插件控制机器人

```
ros2 topic list
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

![image-20241113201922583](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241113201922583.png)

第一次驾驶小车,直接翻车

+ 进行传感器仿真(深度传感器,惯性传感器,激光雷达)

### 任务三: 导航

> **任务**
>
> 使用自定义算法，完成寻路导航
>
> 两个阶段
>
> 1. 基础：在rviz中选定起点和终点，完成自主导航
> 2. 进阶：在rviz中指定路径，使小车能够循环运行，实现巡视功能
>
> **参考：**
>
> https://blog.csdn.net/qq_27865227/article/details/125069399
>
> https://blog.csdn.net/ost_csdn/category_9655312.html

安装完成建图的工具并启动

```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox
ros2 launch task2 gazebo_sim.launch.py
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
rviz2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

 ![image-20241114133520803](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241114133520803.png)

将地图保存为文件([《ROS 2机器人开发从入门到实践》7.2.2将地图保存为文件_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1kB1JYmEvR?spm_id_from=333.788.videopod.sections&vd_source=6aef8a8b18d581883e4331f31b8d68f0))

```
sudo apt install ros-$ROS_DISTRO-nav2-map-server
ros2 run nav2_map_server map_saver_cli -f map_name
```

 **配置好了地图,下一步配置导航**(Navigation2)

```bash
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
```

![image-20241114181914963](https://cdn.jsdelivr.net/gh/Mintisn/Images@main/githubPictures/image-20241114181914963.png)

我们启动了导航的`launch`文件, 可以看到全局代价地图和局部代价地图, 下面需要规划导航









### 任务四:协同

> **任务**
>
> 在任务2、3的基础上新增小车，设计协同寻路方案
>
> 自定义协同方式，注意解决通信问题
