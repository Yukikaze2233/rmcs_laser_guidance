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

- raw 包录制/回放
- PCD / PLY 导出
- ROS2 `PointCloud2` 发布
- 与 `GuidancePipeline` 的深度融合

## Current Debug Entry

```bash
./build/tool_lidar_dump --help
./build/tool_lidar_dump --device-ip 192.168.137.200 --iterations 10
```

当前 CLI 会：

- 请求 points / imu / serial number
- 打印点云完整帧摘要
- 打印 imu 摘要
- 打印 status / serial number 摘要
- 在无设备时清晰打印 timeout

## Why Standalone First

当前优先级是确认 WS30 原始数据正确，而不是先接 ROS2 UI。

所以调试顺序应固定为：

1. `tool_lidar_dump`
2. raw replay / PCD export（下一步）
3. ROS2 bridge
4. Foxglove / RViz2 可视化
5. guidance 深度接入

## Planned Next Step

下一步优先补：

- `--record-raw <file>`
- `--replay <file>`
- `--write-pcd <dir>`

这样可以把设备问题、parser 问题、bridge 问题分开排查。
