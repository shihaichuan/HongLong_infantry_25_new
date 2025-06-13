# pose_solver节点
## 节点说明
接受`/namespace/Top_status`话题下的`base_interfaces::msg::Kinematicstatus`消息，根据ekf解算出云台位姿，进行控制。

## 文件说明

### 参数文件
参数文件是一个ROS2的YAML文件，详情见参数文件。


## 使用方法
1. 执行launch文件
    ```zsh
    ros2 launch pose_solver pose_solver.launch.py
    ```
## 注意事项
1. 使用了timer，使得控制帧率和模型更新帧率分开，将控制的更加丝滑。
2. 在获取云台pitch角和yaw角时，采用了两次坐标变化
   + shooter_link_到shooter_yaw_link的坐标变化
    > 得到云台yaw角
   + shooter_yaw_link到shooter_pitch_link的坐标变化
    > 得到云台pitch角 

    因为tf位姿是用四元数存储的，两次坐标变化就不会因为多个角的变化关系导致yaw和pitch的数值不准。 
