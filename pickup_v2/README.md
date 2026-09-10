# 视觉定位与固定俯仰抓取

本目录提供基于工作平面标定和固定工具俯仰约束的抓取模块。

| 入口 | 内容 |
|---|---|
| pi/main_pickup.py | 树莓派视觉定位与抓取任务 |
| pi/coord_transform.py | 像素、工作面、基座坐标转换 |
| pi/kinematics.py | 固定世界俯仰约束解析 FK/IK |
| pi/protocol_v2.py | M/K/J/OPEN/CLOSE/HOME/PLACE 指令 |
| calibration/ | 相机、平面和关节标定工具 |
| mcu/hal_entry_pickup_v2.c | 对应 RA6M5 接收端 |
| pi/tests/ | 坐标、运动学和协议测试 |

K 接口为五运动关节加夹爪，共六舵机通道；与六运动轴模型不同。完整六运动轴的可运行软件演示见 [software_v3](../software_v3/README.md)，接口范围见 [实现说明](../docs/IMPLEMENTATION.md)。

在仓库根目录执行 `python -m pytest pickup_v2/pi/tests -q`。部署需对应的硬件标定、串口配置和控制固件。
