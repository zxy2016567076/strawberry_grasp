# 草莓眼手力柔性分拣机器人

面向易损果实的视觉分拣与运动控制项目。采用树莓派 5 / RA6M5 协同架构，结合 YOLOv8n 成熟度识别、压力接触检测和轻量 MLP 状态分类；提供完整的六轴抓放软件验证链路。

**展示重点：六运动关节 FK/IK · 多阶段抓放 · 关节步进插值 · 快慢双速 · 八状态调度 · UART 协议 · 压力反馈与 MLP。**

## 一分钟运行

```bash
python -m pip install -r software_v3/requirements.txt
python -m software_v3.demo
```

打开 `software_v3/artifacts/demo.html`，播放或拖动轨迹，查看六轴角度、TCP 路径、状态切换、压力序列及分类。无需相机、串口或控制板；完整数据保存为同目录 JSON。

<details><summary>软件演示预览</summary>

![六轴抓放软件验证](docs/software-demo.png)

</details>

演示输入为明确标注的合成目标、示例机构参数和压力序列。六轴链路验证在软件中完成，夹爪独立于六个运动关节。

## 当前能力

| 模块 | 实现 | 代码入口 |
|---|---|---|
| 系统架构 | 树莓派视觉与任务处理、RA6M5 执行与压力处理、UART 通信 | [Pi](vision/pi/main.py)、[MCU](mcu/hal_entry.c) |
| 视觉与坐标 | YOLOv8n 三成熟度检测；像素→工作平面→基座坐标 | [检测器](vision/pi/detector.py)、[坐标转换](software_v3/vision.py) |
| 六轴运动学 | 可配置串联 6R，完整位姿 FK/数值 IK、关节限位和工作空间检查 | [运动学](software_v3/kinematics.py) |
| 抓放路径 | 七个明确路径点、先提升后横移、快慢双速和关节步进插值 | [路径规划](software_v3/planning.py) |
| 调度与执行 | 八状态调度、协议编码、模拟 MCU 校验与执行、超时/急停退出 | [运行时](software_v3/runtime.py)、[协议](software_v3/PROTOCOL.md) |
| 压力接触 | 接触/保持/异常阈值；独立周期采样任务 | [采样](software_v3/pressure.py)、[夹持](software_v3/runtime.py) |
| 轻量 MLP | float 16→8→4→3，187 参数；三类抓取状态诊断 | [C 推理](mcu/tinyml_grasp.h)、[权重](mcu/tinyml_weights.h) |

```mermaid
flowchart LR
    A[成熟度与像素目标] --> B[基座坐标转换]
    B --> C[六轴 IK 与路径检查]
    C --> D[双速关节插值]
    D --> E[协议编码与模拟执行]
    E --> F[压力阈值夹持]
    F --> G[轻量 MLP 状态诊断]
```

[实现说明](docs/IMPLEMENTATION.md) · [能力与代码对应](docs/CAPABILITY_MAP.md) · [六轴设计](software_v3/README.md) · [实际测试结果](docs/SOFTWARE_VALIDATION.md)

## YOLOv8n 成熟度检测

分类为 `ripe / semi_ripe / unripe`。训练配置：[yolov8n、640 输入、100 轮](vision/runs/strawberry_v12/args.yaml)。下表统一取 [results.csv 第 100 轮](vision/runs/strawberry_v12/results.csv)：

| 指标 | 结果 |
|---|---|
| mAP50 | **98.8%**（原始值 0.98801） |
| mAP50-95 | 78.9% |
| Precision | 97.4% |
| Recall | 94.5% |

![训练曲线](vision/runs/strawberry_v12/results.png)

检测指标与抓取结果分别评估；当前演示使用样例检测结果，不运行 YOLO。模型权重不包含在仓库内。

## 压力与端侧推理

压力 delta 阈值用于接触检测、停止闭合与异常力退出。轻量 MLP 取最近 16 个 delta，前补零并除以 1000，输出 `STABLE / SLIP_RISK / OVERFORCE`。分类用于状态诊断，最大 logit 不等于概率。

提供 [训练与预处理说明](docs/TINYML.md)、[训练脚本](vision/train_tinyml.py) 和 C/Python 推理一致性测试。采样与运动任务在模拟调度中解耦，支持配置 5 ms 周期；该数值是调度目标。

## 测试与故障演示

```bash
python -m pytest software_v3/tests pickup_v2/pi/tests -q
python -m software_v3.demo --fault unreachable
python -m software_v3.demo --fault timeout
python -m software_v3.demo --fault estop
python -m software_v3.demo --fault no-contact
```

已完成 **126 项软件测试**，正常样例运行 444 步并返回 IDLE；四类故障均停止调度并锁存原因。正常退出码 0，故障演示预期退出码 2。C/Python MLP 对 37 个压力窗口的类别与最大 logit 一致。详见 [测试记录](docs/SOFTWARE_VALIDATION.md)。

## 实现范围

六轴演示采用示例模型和模拟 MCU，验证数学、路径约束、协议与调度。板端代码与六轴模拟协议的接口区别见 [实现说明](docs/IMPLEMENTATION.md)。真实六轴标定、机械碰撞、舵机跟踪、ADC 时序和果实抓取效果仍需硬件验证；MLP 分类尚不参与夹持闭环调节。

## 许可

见 [LICENSE](LICENSE)。
