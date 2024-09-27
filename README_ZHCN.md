![Ros2 SDK](https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/49edebbe-11b6-49c6-b82d-bc46257674bd)

# 欢迎来到Unitree Go2 ROS2 SDK项目!

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
![ROS2 Build](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
[![License](https://img.shields.io/badge/license-BSD--2-yellow.svg)](https://opensource.org/licenses/BSD-2-Clause)


我们很高兴向您介绍我们的Unitree Go2与ROS2通过Wi-Fi集成的方案,这是由才华横溢的[@tfoldi](https://github.com/tfoldi)设计的。您可以在[go2-webrtc](https://github.com/tfoldi/go2-webrtc)探索他的开创性工作。

这个仓库将使您的Unitree GO2 AIR/PRO/EDU机器人具备ROS2功能,同时使用WebRTC(Wi-Fi)和CycloneDDS(Ethernet)协议。

如果您使用WebRTC(Wi-Fi)协议,请在连接机器人之前关闭与移动应用程序的连接。

## 项目路线图:

1. URDF :white_check_mark: 
2. 实时关节状态同步 ✅
3. 实时IMU同步 ✅
4. 实时手柄控制 ✅
5. 实时Go2主题信息 ✅
6. 实时足部力传感器信息 ✅
7. 激光雷达流(已添加PointCloud2) ✅
8. 相机流 ✅
9. Foxglove桥接 ✅
10. 激光扫描 ✅
11. 多机器人支持 ✅
12. WebRTC和CycloneDDS支持 ✅
13. 创建点云地图并存储 ✅
14. SLAM(slam_toolbox) ✅
15. 导航(nav2) ✅
16. 物体检测
17. 自动驾驶

## 您的反馈和支持对我们来说意义重大。

如果您像我们一样对这个项目充满热情,请考虑给它一个⭐星标!!!

您的鼓励激发了我们的热情,并帮助我们进一步发展我们的路线图。我们欢迎您提供任何帮助或建议!

让我们一起推动Unitree Go2和ROS2的可能性边界!

## 令人兴奋的特性:

✨ 为您的Unitree GO2提供完整的ROS2 SDK支持

🤖 兼容AIR、PRO和EDU型号

👣 访问足部力传感器反馈(在某些GO2 PRO型号或EDU上可用)

## 实时 Go2 Air/PRO/EDU 关节同步:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/bf3f5a83-f02b-4c78-a7a1-b379ce057492" alt='Go2 joints sync'>
</p>

## Go2 Air/PRO/EDU 雷达点云:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/9c1c3826-f875-4da1-a650-747044e748e1" alt='Go2 point cloud'>
</p>


## 系统要求

已测试的系统和ROS2发行版
|systems|ROS2 distro|Build status
|--|--|--|
|Ubuntu 22.04|iron|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|humble|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|rolling|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)

## 安装

```shell
mkdir -p ros2_ws
cd ros2_ws
git clone --recurse-submodules https://github.com/abizovnuralem/go2_ros2_sdk.git src
sudo apt install ros-$ROS_DISTRO-image-tools
sudo apt install ros-$ROS_DISTRO-vision-msgs

sudo apt install python3-pip clang portaudio19-dev
cd src
pip install -r requirements.txt
cd ..
```
注意任何错误消息。如果`pip install`没有干净地完成,各种功能将无法工作。例如,`open3d`还不支持`python3.12`,因此您需要首先设置3.11 `venv`等。

按照这些[说明](https://www.rust-lang.org/tools/install)安装`rust`语言支持。然后,安装`cargo`(rust包管理器)的1.79版本。

```shell
rustup install 1.79.0
rustup default 1.79.0
```

`cargo`现在应该在终端中可用:

```shell
cargo --version
```

构建`go2_ros_sdk`。您需要安装`ros2`和`rosdep`。如果没有,请按照这些[说明](https://docs.ros.org/en/humble/Installation.html)进行操作。然后:
```shell
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## 使用

别忘了在Wi-Fi模式下设置你的Go2机器人并获取IP地址。你可以使用移动应用程序来获取它。进入设备 -> 数据 -> 自动机器检查,并查找STA网络: wlan0。

```shell
source install/setup.bash
export ROBOT_IP="robot_ip" #for muliple robots, just split by ,
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py
```

`robot.launch.py`代码同时启动了许多服务/节点,包括:

* robot_state_publisher
* ros2_go2_video (前置彩色相机)
* pointcloud_to_laserscan_node
* go2_robot_sdk/go2_driver_node
* go2_robot_sdk/lidar_to_pointcloud
* rviz2
* `joy` (通用操纵杆和游戏控制器的ROS2驱动程序)
* `teleop_twist_joy` (使用标准操纵杆远程操作基于Twist的ROS2机器人的工具。将joy消息转换为速度命令)       
* `twist_mux` (具有源优先级的twist多路复用器)        
* foxglove_launch (启动foxglove桥接)
* slam_toolbox/online_async_launch.py
* av2_bringup/navigation_launch.py

当你运行`robot.launch.py`时,`rviz`将启动,激光雷达数据将开始累积,前置彩色相机数据也将显示出来(通常在4秒后),你的机器狗将等待来自你的操纵杆(例如X-box控制器)的命令。然后,你可以操控机器狗穿过你的房子,例如,收集激光雷达映射数据。

### SLAM 和 Nav2

![Initial Rviz Display](doc_images/slam_nav_map.png)

好的,让我们继续翻译和解释文档的下一部分:

翻译:

### SLAM和Nav2

Show Image

SLAM的总体目标,特别是`slam_toolbox`,是创建一个地图。`slam_toolbox`是一个网格地图构建器 - 它将世界视为机器狗操作的固定网格。当机器狗最初穿过一个新空间时,数据会累积,并且正在开发的地图会被发布到`/map`主题。`Nav2`的目标是在这个地图中导航并执行其他任务。

初始启动时使用的`rviz`设置(由`ros2 launch go2_robot_sdk robot.launch.py`触发)展示了各种数据流。

- `RobotModel`是G02的尺寸正确的模型
- `PointCloud2`是转换为3D对象/约束的原始激光雷达数据
- `LaserScan`是转换为x,y,z坐标系之前的较低级别扫描数据
- `Image`是前置彩色相机的数据
- `Map`是由`slam_toolbox`创建的地图
- `Odometry`是机器狗的方向/移动历史

如果初始屏幕上显示的内容太多,可以取消选择`map`主题以便看得更清楚。

![Simplified Rviz Display](doc_images/slam_nav.png)

### 地图绘制 - 创建你的第一个地图

使用painter's tape 在地板上标记一个'停靠'矩形(或使用真实的停靠站)为你的机器狗创建一个定义好的起点。在`rviz`的`SlamToolboxPlugin`中,在你的`rviz`屏幕左侧,选择"Start At Dock"。然后,使用你的控制器手动探索一个空间,比如一系列房间。你会看到地图数据在`rviz`中累积。在这个地图中,白色、黑色和灰色像素分别代表自由、占用和未知空间。当你完成地图绘制后,在"Save Map"字段中输入一个文件名,然后点击"Save Map"。然后在"Serialize Map"字段中输入一个文件名,点击"Serialize Map"。现在,你应该在`/ros2_ws`中有2个新文件:

```shell
map_1.yaml: 地图的元数据以及.pgm图像文件的路径。
map_1.pgm: 图像文件,白色、黑色和灰色像素分别代表自由、占用和未知空间。
map_1.data: 
map_1.posegraph: 
```

下次启动系统时,可以加载地图,并准备好通过绘制更多空间来完成/扩展地图。重新启动并加载地图后,机器狗不知道它相对于你之前创建的地图的位置。假设你在标记的矩形中或实际的停靠站中重新启动了机器狗,它将有一个高质量的初始位置和角度。 

### 自主导航 - 在你的新地图中导航

如`rviz`的`Navigation 2`插件所示,系统将以以下状态启动:

```shell
Navigation: active
Localization: inactive
Feedback: unknown
```

然后,通过`SlamToolboxPlugin`加载你的地图(在'Deserialize Map'字段中输入你的地图文件名(不带任何扩展名),然后点击'Deserialize Map')。

**警告**: 请确保(1)机器狗相对于地图正确定位,以及(2)地图本身是合理的,并与你的房子相对应。特别是如果你有长走廊,整体地图可能会相对于现实发生扭曲,这意味着路径规划器将尝试让你的机器狗穿墙而过,在你的墙上留下长长的划痕。

现在你可以通过`rviz`菜单中的'Nav2 Goal'给机器狗设置第一个目标。使用鼠标光标提供一个要导航到的目标。

**注意**: `Nav2 Goal`光标既设置目标位置,也设置你希望机器狗到达目标时采取的最终角度(需要再次确认)。当你点击一个点并继续移动鼠标光标时显示的长绿色箭头是角度设置器。

在你有一些经验之前,我们建议跟随你的机器狗,并在它即将做一些愚蠢的事情时将其拿起。

**注意**: 几乎所有的故障行为 - 原地旋转、撞墙、试图穿墙等反映了(1)地图不正确,(2)机器狗相对于该地图的初始位置/角度不正确,或(3)由于控制循环过载而无法计算解决方案/路径。为了防止#3(导致无运动或持续旋转),关键循环率(`controller_frequency`: 3.0和`expected_planner_frequency`: 1.0)已被设置为非常保守的速率。

## 实时图像检测和跟踪

这个功能直接基于[J. Francis的工作](https://github.com/jfrancis71/ros2_coco_detector)。启动`go2_ro2_sdk`。几秒钟后,彩色图像数据将在`go2_camera/color/image`上可用。在另一个终端输入:

```bash
source install/setup.bash
ros2 run coco_detector coco_detector_node
```

第一次运行节点时,会有短暂的延迟,因为PyTorch TorchVision需要下载神经网络。你应该会看到一个下载进度条。TorchVision会缓存以供后续运行。

在另一个终端,查看检测消息:

```shell
source install/setup.bash
ros2 topic echo /detected_objects
```
检测消息包含检测到的对象(`class_id`)和`score`,分数从0到1。例如:`detections:results:hypothesis:class_id: giraffe`和`detections:results:hypothesis:score: 0.9989`。`bbox:center:x`和`bbox:center:y`包含对象中心点的像素坐标。这些数据可以用于实现动物和人的实时对象跟踪。人被检测为`detections:results:hypothesis:class_id: person`。

要查看带有标签和边界框注释的图像流:

```shell
source install/setup.bash
ros2 run image_tools showimage --ros-args -r /image:=/annotated_image
```

使用示例:
```shell
ros2 run coco_detector coco_detector_node --ros-args -p publish_annotated_image:=False -p device:=cuda -p detection_threshold:=0.7
```

这将运行coco检测器,不发布注释图像(默认为True),使用默认CUDA设备(默认device=cpu),并将检测阈值设置为0.7(默认为0.9)。检测阈值应在0.0到1.0之间;这个数字越高,越多的检测将被拒绝。如果你有太多误检,试着增加这个数字。因此,只有Detection2DArray消息会在/detected_objects主题上发布。

## 3D原始点云转储

要保存原始激光雷达数据,`export`以下内容:

```shell
export MAP_SAVE=True
export MAP_NAME="3d_map"
```

每10秒,点云数据(`.ply`格式)将被保存到仓库的根文件夹。

**注意**:这**不是**Nav2地图,而是原始激光雷达数据的转储,用于低级调试。

## 多机器人支持 
如果你想连接多个机器人进行协作:

```shell
export ROBOT_IP="robot_ip_1, robot_ip_2, robot_ip_N"
```

## 在webrtc连接(Wi-Fi)和CycloneDDS(以太网)之间切换

```shell
export CONN_TYPE="webrtc"
```
或者
```
export CONN_TYPE="cyclonedds"
```

## Foxglove

<p align="center">
<img width="1200" height="630" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/f0920d6c-5b7a-4718-b781-8cfa03a88095" alt='Foxglove bridge'>
</p>

要使用Foxglove,你需要安装Foxglove Studio:
```
sudo snap install foxglove-studio
```

1. 打开Foxglove Studio并按"Open Connection"。
2. 在"Open Connection"设置中,选择"Foxglove WebSocket"并使用默认配置ws://localhost:8765,然后按"Open"。

## WSL 2

如果你在WSL2下运行ROS2 - 你可能需要配置操纵杆/游戏手柄来导航机器人。

1. 第1步 - 与WSL2共享设备 按照这里的步骤https://learn.microsoft.com/en-us/windows/wsl/connect-usb 与WSL2共享你的控制台设备

2. 第2步 - 启用WSL2操纵杆驱动程序 WSL2默认不带操纵杆的模块。使用操纵杆驱动程序构建WSL2内核。按照这里的说明操作: https://github.com/dorssel/usbipd-win/wiki/WSL-support#building-your-own-wsl-2-kernel-with-additional-drivers  如果你熟悉WSL2,跳过导出步骤,从`安装先决条件`开始。 在构建之前,编辑`.config`文件并更新此GitHub问题中列出的CONFIG_值: https://github.com/microsoft/WSL/issues/7747#issuecomment-1328217406

3. 第3步 - 给予/dev/input设备权限 一旦你完成了第3步下的指南 - 你应该能在/dev/input下看到你的操纵杆设备

   ```bash
   ls /dev/input
   by-id  by-path  event0  js0
   ```

   默认情况下/dev/input/event*只有root权限,所以joy节点无法访问操纵杆 创建一个文件`/etc/udev/rules.d/99-userdev-input.rules`,内容如下: `KERNEL=="event*", SUBSYSTEM=="input", RUN+="/usr/bin/setfacl -m u:YOURUSERNAME:rw $env{DEVNAME}"` 以root身份运行: `udevadm control --reload-rules && udevadm trigger` https://askubuntu.com/a/609678

4. 第3步 - 验证joy节点是否能正确看到设备。 运行`ros2 run joy joy_enumerate_devices`

   ```bash
   ID : GUID                             : GamePad : Mapped : Joystick Device Name
   -------------------------------------------------------------------------------
   0 : 030000005e040000120b000007050000 :    true :  false : Xbox Series X Controller
   ```

## 致谢

特别感谢:

1. @tfoldi (Tamas)的想法和才能,创造了python和unitree GO2之间的webrtc连接方法;
2. @budavariam帮助解决激光雷达问题;
3. @legion1581提供了一种新的webrtc方法,可以与1.1.1固件更新一起使用;
4. @alex.lin对ros1集成的热情;
5. @alansrobotlab对机器人的热情和帮助我调试新的webrtc方法;
6. @giangalv (Gianluca Galvagn)帮助我调试webrtc的新问题;
7. 许多许多其他开源贡献者!和TheRoboVerse社区!

## 许可证

本项目采用BSD 2-clause许可证 - 详见[LICENSE](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/LICENSE)文件。
