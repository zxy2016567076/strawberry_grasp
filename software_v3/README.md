# V3 六轴软件扩展

赛后新增，无硬件，非原竞赛版本。模拟 MCU 使用 Python，不提供已部署的 RA6M5 六轴固件，不发真实 UART，不改历史控制代码。

## 运行

仓库根目录执行 `python -m software_v3.demo`，生成 `software_v3/artifacts/demo.html` 和 `demo.json`。`--config software_v3/config/example.json --target software_v3/examples/target.json` 可替换配置和目标；`--fault timeout|unreachable|estop|no-contact` 输出故障报告并返回 2。默认输入均标注 SYNTHETIC，不是 YOLO 推理或真实压力。

## 模型约定

右手基座系 Z 向上，mm/deg，4×4 齐次位姿。第 i 级为 `T_i = Rot(axis_i, radians(q_i+zero_i)) · Trans(link_i)`；axis 在父级表达，link 在旋转后的本级表达。六级相乘得到 TCP。夹爪开合独立于六关节角，不进入 FK/IK。

JSON 可配置六轴方向、连杆向量、零位、限位、HOME。默认为 Z-Y-Y-X-Y-X 串联 6R 球腕：基座高度 120、上臂 160、前臂 140、工具长度 60 mm。几何零位 q=0 时 TCP=(360,0,120)、旋转为单位阵；HOME 单独配置。**所有尺寸和限位都是软件示例，不来自历史标定。**限位作用于命令角 q，几何角=q+zero，不是旧舵机 PWM 角。

`Arm.ik` 使用有界数值最小二乘，残差为毫米位置误差拼接 100×旋转向量（rad）。优先上一解，再尝试确定性备用初值；接受条件为位置 <0.05 mm、方向 <1e-4 rad 且满足关节限位。工作空间盒与连杆长度检查是必要条件，最终仍需 IK 成功。预算内没找到解也明确退出，不保证穷举全部解或全局可达性判定。测试在非奇异构型验证雅可比满秩六，未提供完整奇异规避。

## 七路径点与八状态

| 路径点 | 用途 |
|---|---|
| home | 开始与结束参考；回位时复用 |
| pre_grasp | 目标上方净空高度，快移接近 |
| grasp | 竖直慢降，随后独立闭爪 |
| lift | 原地抬升，携物横移前提 |
| transit | 净空高度横移到所选料碗上方 |
| place | 慢降到放置高度，独立开爪 |
| retreat | 竖直离开料碗，随后返回 home |

`planning.py:Waypoint` 是任务数据，不持有状态枚举；`runtime.py:State` 为 `IDLE → BELT_RUN → BELT_STOP → PRE_GRASP → GRASP → LIFT → PLACE → RETURN → IDLE`。PLACE 管理 transit/place；RETURN 管理 retreat/home。故障为独立锁存标志，保留最后状态。历史八姿态常量原样保留，不删常量凑数量。

完整任务先预检再执行：笛卡尔分段、姿态 SLERP、连续初值 IK、关节步进、逐步限位与 TCP 工作空间检查。默认快 60°/s、慢 15°/s、20 ms 运动周期、1° 单步上限；慢速实际最大步幅 0.3°。横移必须在 180 mm 净空高度以上，允许 0.1 mm 数值容差；先提升再横移。

只检查 TCP 与净空规则，**没有自碰撞、连杆扫掠或环境障碍物检测**。线性关节步进不保证加速度/jerk 连续。

## Pi / MCU 与压力任务

Pi 模型：检测结果接口、坐标转换、IK、路径预检、插值、八状态调度。模拟 MCU：版本/能力握手、CRC/序号、限位/步幅复核、独立夹爪、压力任务和看门狗。

模拟按 1 ms 推进事件循环，运动等待不阻塞 `Sampler`。压力默认 5 ms 可配置；调用迟到只采当前值，不补造历史样本。未来应使用 RA6M5 GPT/ADC 完成事件写入环形缓冲，定时运动任务更新 PWM，记录真实时间戳和超期计数。本版本未移植 FSP 定时器，虚拟时钟不等于 RA6M5 实测 200 Hz。

合成 delta 达 400 判接触，达到 600 停止闭合，异常阈值 1200 故障退出，2 s 未保持则超时。阈值均为软件示例。随后调用历史 float 权重做分类诊断，不驱动夹持反馈。

## 故障退出

预检失败零下发；协议拒绝、急停、异常力、接触超时均停带、冻结关节和夹爪命令、锁存故障。丢 ACK 不盲目重试，1 s 无通信触发模拟 MCU 看门狗。故障后不自动回零/开爪/重启；需显式新建模拟运行。物理断能、制动与持物策略待硬件设计。

参见 [协议](PROTOCOL.md)、[TinyML](../docs/TINYML.md)、[测试结果](../docs/SOFTWARE_VALIDATION.md)。
