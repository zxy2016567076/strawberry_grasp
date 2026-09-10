# 简历能力与代码对应

| 能力 | 可查看的实现 | 验证方式 |
|---|---|---|
| 树莓派/RA6M5 协同与 UART | vision/pi/main.py、vision/pi/serial_comm.py、mcu/hal_entry.c | 查看视觉任务、串口及板端执行代码 |
| 视觉目标与基座坐标 | vision/pi/detector.py、software_v3/vision.py | 单应变换和基座刚体变换测试 |
| 六轴 FK/IK（软件验证） | software_v3/kinematics.py | 完整位姿往返、满秩六轴、零位/限位、不可达 |
| 多阶段抓放和过渡 | software_v3/planning.py | 七路径点，三料碗路线，TCP 净空检查 |
| 步进插值与快慢双速 | software_v3/planning.py:joint_steps/segment | 单步幅度/速度上界，低位横移拒绝 |
| 八状态调度与模拟执行 | software_v3/runtime.py | 正常回 IDLE，超时/急停/无接触退出 |
| 六轴协议 | software_v3/protocol.py | 帧、CRC、分片、能力握手、重放拒绝 |
| 压力接触与异常保护 | mcu/hal_entry.c、software_v3/runtime.py | delta 阈值接触/保持/异常力，不依赖分类 |
| 独立采样任务 | software_v3/pressure.py:Sampler | 可配 5 ms 调度目标；迟到不补造样本 |
| 轻量 MLP 分类 | mcu/tinyml_grasp.h、mcu/tinyml_weights.h、software_v3/pressure.py | float 187 参数；37 窗口 C/Python 一致 |
| YOLOv8n mAP50 98.8% | vision/runs/strawberry_v12/results.csv 第 100 轮 | 原始值 0.98801，属于检测指标 |
| 可运行软件展示 | software_v3/demo.py、software_v3/report.html | 轨迹回放、状态、六轴角度、压力诊断 |

六轴部分为软件验证；板端与模拟端的接口范围见 [IMPLEMENTATION.md](IMPLEMENTATION.md)。MLP 输出用于诊断，不作为已验证的闭环夹持调节。全部测试结果见 [SOFTWARE_VALIDATION.md](SOFTWARE_VALIDATION.md)。
