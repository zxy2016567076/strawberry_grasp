# 软件验证记录

执行日期：2026-09-10。Windows / Python 3.12.4，NumPy 1.26.4、SciPy 1.13.1、pytest 7.4.4、MinGW GCC 8.1.0；基线 `f4a2d350baa62cc431039be72d89d59efc5fabf9`，工作分支 `feat/six-axis-software-demo`。所有新增执行结果均为主机软件测试，未连接相机、串口或 RA6M5。

## 自动测试

最终实际执行：

```bash
python -m pytest software_v3/tests pickup_v2/pi/tests -q -p no:cacheprovider --basetemp=D:/VS_code/projects/rubbish_car/strawberry-sorting-robot/tmp/pytest-run-02
```

结果：**126 passed in 16.12s**，无失败、无跳过。含原 pickup_v2 91 项、新 V3 35 项。一般环境直接运行 `python -m pytest software_v3/tests pickup_v2/pi/tests -q` 即可；上述临时目录是本机特有设置，复用 `--basetemp` 时 pytest 会清理该专用目录，不要指向工作文件目录。

最初测试曾受 Windows 临时目录权限影响（111 passed、12 项初始化错误），改为仓库内专用临时目录后完成验证；不是通过跳过测试规避失败。

覆盖：六个独立运动轴与夹爪分离；全位姿 FK/IK 往返；可配置零位、限位和非法参数拒绝；超工作空间和盒内几何不可达；像素到基座；三成熟度料碗七路径点；步进幅度、双速、净空规则；协议 CRC/版本/分片/重放/握手；接收端超步幅与看门狗；非阻塞采样、不补造样本；接触/异常压力；正常八状态和故障冻结；MLP 诊断不参与控制。

主机 GCC 编译现有 `mcu/tinyml_grasp.h` 与权重，对 **37 个压力窗口**比较 C/Python 类别和最大 logit（容差 2e-5）。这是软件浮点一致性，不是分类准确率或 RA6M5 推理性能。权重 SHA256：`d2b3842468ed72a76fdeac173b1fba2212f4045542e78a3a3024e0b5b6cd289e`。

## 实际演示结果

| 命令（均以 python -m software_v3.demo 开头） | 返回码 | 结果 | 主机确认步数 | 虚拟时长 ms |
|---|---|---|---|---|
| 无附加参数 | 0 | DONE → IDLE | 444 | 9231 |
| --fault timeout | 2 | ACK_TIMEOUT: UART_WATCHDOG | 0 | 1221 |
| --fault unreachable | 2 | outside configured workspace | 0 | 0 |
| --fault estop | 2 | ESTOP | 2 | 250 |
| --fault no-contact | 2 | NO_CONTACT_TIMEOUT | 140 | 5000 |

timeout 中模拟 MCU 已接受第一个运动步、ACK 被丢弃，所以主机确认步数为 0；JSON 的 `mcu_accepted_steps=1` 记录这一差异。没有重复发送。不可达零下发；所有故障均停带并锁存，不自动继续或返回 HOME。

正常输入像素 (320,240)、ripe，经软件示例标定得到基座 (220,20,70) mm。5 ms 虚拟压力任务产生 1847 次调度采样，夹持窗口最后 16 点得到 STABLE，logits 约 `[3.859208,-3.024003,2.783882]`。这些数字来自合成压力与虚拟时钟，不是训练集评估、实物抓取、板端 200 Hz 或模型闭环调力证据。

报告包含输入来源、完整配置、七位姿矩阵、状态时间线、关节/TCP 轨迹、独立夹爪最终命令、压力序列、权重摘要和握手字节。正常/故障 CLI 返回码均由子进程检查。HTML 已用本机无窗口 Edge 渲染并目视检查，无外部资源依赖。

## 复现环境与持续验证

```bash
python --version
python -c "import numpy,scipy,pytest; print(numpy.__version__,scipy.__version__,pytest.__version__)"
gcc --version
```

新增 `.github/workflows/software.yml` 在 Linux/Python 3.11 执行相同测试并生成演示报告 artifact。该 CI 配置尚未在 GitHub 远端运行，本次结果全部为本机实跑。没有推送主分支。

## 尚需硬件验证

六轴实际机构与夹爪独立执行通道；尺寸/零位/限位/相机标定；环境与自碰撞、负载与舵机跟踪；真实 UART 带宽与电气可靠性（V3 JSON 十六进制帧不保证能以历史 115200 波特率完成当前模拟步率）；RA6M5 定时器/ADC 抖动；压力阈值、夹持损伤与真实试验统计；TinyML 独立实物数据评估与任何分类闭环。当前 TCP 净空检查不能替代这些工作。
