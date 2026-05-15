# WS30 LiDAR

## Current Status

当前仓库已经落地的 WS30 能力只有 standalone scaffold：

- `Ws30UdpSocket`
- `Ws30PacketParser`
- `Ws30FrameAssembler`
- `Ws30Client`
- `tool_lidar_dump`

它的目标不是马上接控制链，而是先确认：

- UDP 收包稳定
- points / imu / status 解析正确
- 点云帧能稳定组装
- 无设备时超时路径明确

当前**尚未实现**：

- ROS2 `PointCloud2` 发布
- 与 `GuidancePipeline` 的深度融合

## Current Debug Entry

```bash
./build/tool_lidar_dump --help
./build/tool_lidar_dump --device-ip 192.168.137.200 --iterations 10
./build/tool_lidar_dump --device-ip 192.168.137.200 --iterations 100 --record-raw /tmp/ws30.rawlog
./build/tool_lidar_dump --replay /tmp/ws30.rawlog --write-pcd /tmp/ws30_pcd
```

当前 CLI 会：

- 请求 points / imu / serial number
- 打印点云完整帧摘要
- 打印 imu 摘要
- 打印 status / serial number 摘要
- 在无设备时清晰打印 timeout
- 录制 WS30 原始 UDP payload (`--record-raw`)
- 从 raw log 回放 (`--replay`)
- 将完整点云帧导出成 ASCII PCD (`--write-pcd`)

## Why Standalone First

当前优先级是确认 WS30 原始数据正确，而不是先接 ROS2 UI。

所以调试顺序应固定为：

1. `tool_lidar_dump`
2. raw replay / PCD export
3. ROS2 bridge
4. Foxglove / RViz2 可视化
5. guidance 深度接入

## Current Raw Log Format

raw log 当前是自定义二进制格式：

- 文件头：`WS30LOG1`
- entry header：`stream_kind + capture_unix_ns + payload_size`
- payload：WS30 原始 UDP datagram

这样可以把设备问题、parser 问题、bridge 问题分开排查。

## Current Next Step

现在最合理的下一步是：

1. ROS2 bridge package
2. `PointCloud2` 发布
3. RViz2 / Foxglove 可视化配置
4. 再接 `GuidancePipeline`
