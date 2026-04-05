### 功能包介绍

自瞄主要部分位于 rm_auto_aim 目录下。其中：

- armor_detector 负责识别装甲板，并获取其在相机坐标系下的位姿。
- armor_tracker 负责利用装甲板位姿对整车进行建模，获得对整车的状态估计。
- planning_trajectory 负责进行目标选择、弹道解算、开火判定。

流程链路末端为 planning_trajectory，最终发布云台期望欧拉角给 rm_serial_driver。

### 提交要求

使用该仓库 main 中提供的 hik_camera、rm_serial_driver 获取图像、与下位机通信，基于此前作业实现的装甲板识别器，实现解算出期望云台欧拉角。

分支名命名为姓名拼音。

### 编译

```bash
colcon build --symlink-install # 编译，对于安装项目进行符号链接
```

### 启动串口节点 

```bash
source install/setup.bash
sudo chmod 777 /dev/ttyACM0 # 给予串口权限
ros2 launch rm_serial_driver ros2_libxr_launch.py # 启动串口包launch
```

### 通过命令行发布云台控制量
```bash
ros2 topic pub /trajectory/send auto_aim_interfaces/msg/Send "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, is_fire: false, pitch: 0.0, yaw: 0.0, vel_yaw: 0.0, acc_yaw: 0.0, idx: 0}" --once
```


### 在视觉节点中获取云台当前姿态
```cpp
// gimbal_odom 为云台坐标系，pitch_link 为 pitch 连杆末端坐标系。相当于查询云台 pitch 和 yaw。
// 注意：我们的机器人的坐标系定义位于 robot_gimbal_description
// 其中链路为 gimbal_odom -> yaw_link -> pitch_link -> camera_link -> camera_optical_frame 
auto odom_to_gimbal = tf2_buffer_->lookupTransform(
    "gimbal_odom", "pitch_link", 
    img_msg->header.stamp, 
    rclcpp::Duration::from_seconds(0.01)); // 查询 gimbal_odom 到 pitch_link 的变换

// 查询到的旋转为一个四元数，我们可以通过 tf 来将其转换为欧拉角
tf2::Quaternion qu(
    odom_to_gimbal.transform.rotation.x, odom_to_gimbal.transform.rotation.y,
    odom_to_gimbal.transform.rotation.z, odom_to_gimbal.transform.rotation.w);
tf2::Matrix3x3 m(qu); // 利用四元数构造旋转矩阵
double roll = NAN, pitch = NAN, yaw = NAN;
m.getRPY(roll, pitch, yaw); // 通过内部函数获取 rpy
```

### 启动相机节点

```bash
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```

建议将识别器与相机在 launch 中以同一进程启动，参考：

```python
def get_camera_detector_container(camera_node):
    return ComposableNodeContainer(
        name="camera_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            camera_node,
            ComposableNode(
                package="armor_detector",
                plugin="rm_auto_aim::ArmorDetectorNode",
                name="armor_detector",
                parameters=[node_params, {"robot_type": robot_type}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
        emulate_tty=True,
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "armor_detector:=" + launch_params["detector_log_level"],
        ],
        on_exit=Shutdown(),
    )
```

前提为将节点注册为组件。参考：

```cpp
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its
// library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
```