# 能力到代码与验证映射

| 能力 | 历史入口/证据 | 新入口（software_v3 下） | 验证与边界 |
|---|---|---|---|
| Pi/RA6M5 分工 | vision/pi/main.py、mcu/hal_entry.c | runtime.py:Scheduler/SimMCU | 新 MCU 是 Python 模型，无 FSP 移植 |
| UART | serial_comm.py、protocol_v2.py | protocol.py | 帧、CRC、分片、能力握手、重放拒绝；无物理串口 |
| 成熟度目标 | vision/pi/detector.py | vision.py:from_detection | 适配历史 Detection；默认是合成输入，不运行 YOLO |
| 坐标转换 | pickup_v2/pi/coord_transform.py | vision.py:target_to_base | 单应矩阵+基座刚体变换；已知平面高度，不估计深度 |
| 六运动轴 FK/IK | 历史六通道含夹爪 | kinematics.py:Arm | 全位姿往返、满秩六自由度、零位、限位、不可达 |
| 七路径点 | 历史八姿态常量保留 | planning.py:Waypoint/task_waypoints | 三成熟度料碗完整预检 |
| 步进插值/双速 | robot_apply_pose_ex、INTERP_* | planning.py:joint_steps/segment | 逐步幅度/速度上界；无加速度保证 |
| 安全过渡 | g_pose_transit | planning.py:segment | 先提升后横移、TCP 净空；不检测自碰撞或障碍物 |
| 八状态 | 主线 state_t | runtime.py:State/Scheduler | 正常回 IDLE；故障独立锁存 |
| 超时/急停 | 历史停止逻辑保留 | SimMCU.halt/tick、Scheduler.wait | 不可达零下发、看门狗、急停、接触超时 |
| 压力接触 | fsr_grasp_with_feedback | SimMCU.tick | 原始 delta 阈值夹持，与 MLP 分开 |
| 独立采样 | 历史阻塞闭爪循环 | pressure.py:Sampler | 5 ms 可配、迟到不补造样本；非板端 200 Hz |
| float MLP | mcu/tinyml_grasp.h、tinyml_weights.h | pressure.py:prepare_input/classify | 37 窗口主机 C/Python 一致；非真实分类精度 |
| YOLO 指标 | vision/runs/strawberry_v12/results.csv | README 指向第 100 轮 | 原日志不改；不是抓取成功率 |
| 无硬件展示 | 历史实物图保留 | demo.py、report.html | 可播放路径、关节角、状态、压力；合成数据显式标注 |

新增测试在 `software_v3/tests/test_software.py`；历史 `pickup_v2/pi/tests/` 原样执行。结果见 [SOFTWARE_VALIDATION.md](SOFTWARE_VALIDATION.md)。
