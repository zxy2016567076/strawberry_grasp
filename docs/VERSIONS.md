# 版本身份与证据

检查基线 `f4a2d350baa62cc431039be72d89d59efc5fabf9`，工作分支 `feat/six-axis-software-demo`。目标仓库和适用父目录未找到 `AGENTS.md`。

| 版本 | 身份 | 入口 |
|---|---|---|
| 主线 | 历史竞赛固定姿态分拣，五运动关节加夹爪 | `vision/pi/main.py`、`mcu/hal_entry.c` |
| pickup_v2 | 赛后实验升级，固定世界俯仰解析 IK；保留标定与串口复盘 | `pickup_v2/pi/main_pickup.py`、`pickup_v2/mcu/hal_entry_pickup_v2.c` |
| software_v3 | 本次六运动关节加独立夹爪软件扩展；无硬件 | `software_v3/demo.py` |

主线保留 `g_pose_home/pre_grasp/grasp/lift/transit/place_a/place_b/place_c` 八姿态常量，没有删除旧常量凑“七姿态”。主线八状态与姿态数量无等价关系。V3 七路径点为 home/pre_grasp/grasp/lift/transit/place/retreat，回位复用 home，状态机独立。

| 表述 | 证据 | 当前结论 |
|---|---|---|
| mAP50 98.8% | 原 results.csv 第 100 轮 0.98801 | 可保留，属于检测验证 |
| 历史六自由度臂 | 主线关节/通道枚举 | 五运动关节+夹爪，新六运动轴另列 |
| INT8 1D-CNN | 实际 tinyml_grasp.h / weights.h | float MLP，187 参数 |
| 实测 200 Hz | 主线包含阻塞延时，未见时间戳统计 | 不作实测宣称；新 5 ms 仅调度目标 |
| TinyML 闭环调力 | GRASP 分类后仅 printf | 模型诊断与阈值夹持分开 |
| <1 ms 推理/微秒保护 | 历史注释，未见测量数据 | 当前展示不承诺 |
| 91.1% 抓取成功率 | 未见逐次实验、样本量和计算来源 | 不新增为能力或测试结论 |

旧中文设计稿及源码注释保留为历史档案，部分含早期目标或未复核指标。当前能力以 README、本说明、能力映射与软件验证为准。历史事实不能用本次模拟结果追认。
