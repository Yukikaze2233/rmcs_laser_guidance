# ROS2 Interface (Planned)

## Design Rule

ROS2 只做 bridge，不进入 core。

bridge 的职责固定为：

- 参数读取
- 输入采集
- 结果发布

不要在 bridge 中重新实现 WS30 协议解析。

## Package Shape

建议新增独立 package，例如：

- `ros2/rmcs_laser_guidance_bridge`
- 或 `ros2/ws30_lidar_bridge`

它依赖 `rmcs_laser_guidance_core`，但不反向污染 core。

## Topics

建议 topic：

- `/gimbal/laser_guidance/ws30/points`
  - `sensor_msgs/msg/PointCloud2`
- `/gimbal/laser_guidance/ws30/imu`
  - `sensor_msgs/msg/Imu`
- `/gimbal/laser_guidance/ws30/status`
  - `diagnostic_msgs/msg/DiagnosticArray`

## Parameters

- `device_ip`
- `points_port`
- `imu_port`
- `status_port`
- `frame_id`，默认 `ws30_lidar`
- `packet_timeout_ms`
- `publish_imu`
- `use_sensor_timestamp`

## Services

建议只保留一个最小 service：

- `/gimbal/laser_guidance/ws30/reopen`
  - `std_srvs/srv/Trigger`

## Visualization

点云查看建议支持两条路径：

1. Foxglove
2. RViz2

两者都消费标准 `PointCloud2`。

## Deferred Items

当前先不做：

- `/tf`
- 自定义复杂消息
- 把 Foxglove / RViz2 UI 逻辑并入 core
