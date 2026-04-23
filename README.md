# rmcs_laser_guidance

`rmcs_laser_guidance` 是一个最小激光视觉引导骨架，当前阶段只覆盖：

- `V4L2/UVC` 采集卡取图
- 原始视频会话录制与离线抽帧导出
- YAML 配置加载
- `Pipeline` 统一视觉入口与可切换后端骨架
- 最小亮点检测 `Detector`
- 红色目标 ROI 精修骨架
- 调试 overlay 与回放样本
- 自动测试与人工运行示例

当前阶段**不包含**：

- RMCS 控制接入
- `/gimbal/*` 接口
- `HardSyncSnapshot`
- 发弹时机或 `fire_control`

## Build

仓库根目录直接构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

配套测试：

```bash
ctest --test-dir build --output-on-failure
```

如果你在带 Jazzy 的 ROS 工作区里，也可以继续使用：

```bash
build-rmcs --packages-select rmcs_laser_guidance
```

或：

```bash
cd rmcs_ws
colcon build --packages-select rmcs_laser_guidance --symlink-install --merge-install
```

说明：

- 当机器上没有 `/opt/ros/jazzy` 和 `ament_cmake` 时，ROS/`colcon` 路径不可用，但根目录直接 `cmake` 仍应可用。
- 当机器上有完整 Jazzy 环境时，两种构建方式都支持。
- 如果要启用 ONNX Runtime 骨架，额外传：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DRMCS_LASER_GUIDANCE_WITH_ONNXRUNTIME=ON \
  -DONNXRUNTIME_ROOT=/abs/path/to/onnxruntime
```

## Auto Tests

自动验证目标位于 `tests/*_test.cpp`，可通过：

```bash
ctest --output-on-failure
```

在对应构建目录中执行全部自动测试。Standalone 构建对应 `build/`，ROS 工作区构建对应 `build/rmcs_laser_guidance/`。

## Examples

离线冒烟：

```bash
ros2 run rmcs_laser_guidance example_offline_smoke
```

V4L2 预览：

```bash
ros2 run rmcs_laser_guidance example_v4l2_preview
```

也可以显式传配置路径：

```bash
ros2 run rmcs_laser_guidance example_v4l2_preview /abs/path/to/default.yaml
```

V4L2 采图录帧：

```bash
ros2 run rmcs_laser_guidance example_v4l2_capture
```

V4L2 原始视频会话录制：

```bash
ros2 run rmcs_laser_guidance example_v4l2_record_session
```

也可以显式传录制根目录和时长：

```bash
ros2 run rmcs_laser_guidance example_v4l2_record_session \
  /abs/path/to/default.yaml \
  /abs/path/to/session_root \
  30
```

回放预览：

```bash
ros2 run rmcs_laser_guidance example_replay_preview
```

检测 benchmark：

```bash
ros2 run rmcs_laser_guidance example_detector_benchmark
```

模型推理占位入口：

```bash
ros2 run rmcs_laser_guidance example_model_infer
```

也可以显式覆盖模型路径：

```bash
ros2 run rmcs_laser_guidance example_model_infer \
  /abs/path/to/default.yaml \
  /abs/path/to/replay_dir \
  /abs/path/to/model.onnx
```

离线抽帧生成待标注数据集：

```bash
ros2 run rmcs_laser_guidance example_export_training_frames \
  /abs/path/to/session_dir \
  /abs/path/to/dataset_root \
  train \
  200
```

## Notes

- 默认配置文件位于 `config/default.yaml`
- 默认样本回放位于 `test_data/sample_images`
- `models/` 用于放置 `.onnx` 模型文件
- public 头文件平铺在 `include/`，实现细节头收束在 `src/internal/`
- 默认 live 输入为 UVC 采集卡 `v4l2.device_path=/dev/video0`
- 默认 live 模式为 `1920x1080 @ 60 FPS`，优先 `mjpeg`
- 训练数据链路当前推荐“先录原始视频会话，再离线抽帧生成待标注图片”
- 抽帧导出会生成 `images/train|val|test` 和按会话区分的 `export_manifest.csv`
- 当前检测逻辑仍然是极简亮点检测实现，用于把工程链路跑通
- 红色目标精修当前以内部 `RedTargetRefiner` 形式存在，预留给后续模型 ROI 后处理使用
- `Pipeline` 通过 `inference.backend` 在 `bright_spot` 和 `model` 占位后端间切换
- `inference.model_path` 现在用于定位 `.onnx` 文件，但默认构建仍不带 ONNX Runtime
- `model` 后端当前已具备“可选 ONNX Runtime + 模型元数据读取 + 明确报错”骨架
- 没有模型专属 adapter 之前，`model` 后端不会静默返回空检测，而是明确报告未启用、缺模型或未适配输出契约
- 本仓库当前不负责本地训练；训练应在外部平台完成，仓库只负责数据集生成和模型接入
- 后续如果要接 RMCS，再单独增加 bridge 和控制接口

## Docs

- `plan.md`
- `docs/README.md`
- `docs/architecture.md`
- `docs/development.md`
- `docs/future_rmcs_integration.md`
