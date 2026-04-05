
### 启动串口节点 

colcon build --symlink-install

source install/setup.bash

sudo chmod 777 /dev/ttyACM0

ros2 launch rm_serial_driver ros2_libxr_launch.py

### 通过命令行发布云台控制量
ros2 topic pub /trajectory/send auto_aim_interfaces/msg/Send "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, is_fire: false, pitch: 0.0, yaw: 0.0, vel_yaw: 0.0, acc_yaw: 0.0, idx: 0}" --once


### 在视觉节点中获取云台当前姿态
```
            auto odom_to_gimbal = tf2_buffer_->lookupTransform(
                "gimbal_odom", "pitch_link", img_msg->header.stamp,
                rclcpp::Duration::from_seconds(0.01));

            tf2::Quaternion qu(
                odom_to_gimbal.transform.rotation.x, odom_to_gimbal.transform.rotation.y,
                odom_to_gimbal.transform.rotation.z, odom_to_gimbal.transform.rotation.w);
            tf2::Matrix3x3 m(qu);
            double roll = NAN, pitch = NAN, yaw = NAN;
            m.getRPY(roll, pitch, yaw);
```