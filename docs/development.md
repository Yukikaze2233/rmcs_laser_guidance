# 开发说明

## 目录职责

- `include/`
  - 对外头文件。当前采用扁平布局，只放稳定公共接口。
- `src/`
  - 对应公共接口的实现。
- `src/internal/`
  - 不安装的私有实现头，只给库自身、examples 和白盒测试使用。
- `examples/`
  - 手工运行入口，不承担核心逻辑。
- `tests/`
  - 自动化验证目标，文件名统一为 `*_test.cpp`。
- `config/`
  - 默认配置。
- `models/`
  - 模型文件目录，当前用于放置 `.onnx`。
- `test_data/`
  - 样本回放与固定测试资源。

## 当前目录拆分原则

当前不要一开始就引入：

- `io/`
- `tasks/`
- `tools/`

原因不是这些目录一定不好，而是当前项目还没复杂到需要那种拆法。

如果后续出现下面任一情况，再考虑拆层：

- 检测器不止一个实现
- 需要视频源、相机源、回放源三套输入
- 引入 tracker / solver / bridge
- 多个入口共享大量工具逻辑

## 推荐改动顺序

当你要扩展功能时，优先级建议是：

1. 扩展 `TargetObservation`
2. 优先扩展 `Pipeline` 对外能力，再决定内部是否要拆 `Detector`
3. 扩展 `tests/` 自动验证
4. 扩展 `examples/` 人工验证路径
5. 最后才考虑新增更细的目录

这样可以避免一开始就把结构拆复杂。

## 文档维护规则

每次项目结构发生变化时，至少同步更新：

- `README.md`
- `AGENTS.md`
- `docs/architecture.md`

如果将来加入 RMCS bridge，还必须补：

- 新的数据流图
- 新的输入输出接口名
- 新的运行入口说明

## 当前完成标准

当前阶段的完成标准不是命中率或控制效果，而是：

- 相机稳定运行
- pipeline 能稳定处理正常图像和异常输入
- 自动测试与人工入口都存在
- 核心逻辑不依赖 ROS 控制链

## 当前模型接入约束

- ONNX Runtime 依赖当前必须保持为可选构建，不要把默认构建改成强依赖。
- 在没有真实模型契约前，不要提前把 `mean/std`、`NMS`、`class_names` 等模型专属配置写死进 public config。
- `model` 后端当前允许扩展内部 `ModelRuntime` / `ModelAdapter`，但不要把这些内部类型提升到 public API。
- 数据集生成相关能力当前应保持在 internal / examples 层，不要把 session/export manifest 结构提升到 public API。
- 本仓库只负责数据集生成和模型接入，不负责本地训练逻辑；训练流程默认放在外部平台。
