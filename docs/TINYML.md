# 实际 TinyML：float MLP

`mcu/tinyml_grasp.h` 实现 16→8→4→3：两个 ReLU 隐藏层、线性 logits、argmax。`tinyml_weights.h` 保存 float，共 `16×8+8+8×4+4+4×3+3=187` 参数。无卷积、INT8、量化尺度或零点。

类别：0 STABLE（训练名 STABLE_GRASP）、1 SLIP_RISK、2 OVERFORCE。C 的 confidence 输出为最大 logit，不是置信概率。

## 预处理与来源

固件采基线后记录非负 FSR delta；`tinyml_prepare_input` 取最后 16 个，不足前补零，再除以 1000.0f。按样本序号取窗口，无重采样；板端采样间隔未证明恒定 5 ms。

权重头注明由 `vision/train_tinyml.py` 导出，但没有训练运行 ID、数据 manifest、独立测试集或真实压力 CSV。脚本默认生成合成数据（seed 42，每类 300），初始化 seed 0，800 轮，训练后在同一数据打印拟合准确率。不能当作独立评估或真实果实分类精度；不能仅凭头注释确认当前权重训练数据来源。

V3 直接解析现有主线权重，不重训、不覆盖。每次报告记录 SHA256。pickup_v2 的旧副本保留。固定周期合成压力也不消除训练与真机分布差异。

## 复现

```bash
python -m software_v3.demo
python -m pytest software_v3/tests/test_software.py -k mlp -q
```

主机有 GCC/cc 时，测试临时编译原 C 头文件，比较 37 个确定性窗口的类别和最大 logit，包含空、短、截断、不同幅值。无编译器则显式 skip。该测试不编译 RA6M5 工程，也不测板端时延。

## 分类不控制夹持

板端 GRASP 流程：慢速到抓取姿态 → `fsr_grasp_with_feedback` 阈值闭爪 → `tinyml_prepare_input/classify` → printf → 等待提升，没有根据模型结果调夹爪的分支。

V3 同样分开两者，测试注入 OVERFORCE 模型诊断不会改变调度；真正异常力由原始 delta 阈值触发。未来模型闭环仍需真实数据、可靠标签、独立评估、失效策略与真机验证。
