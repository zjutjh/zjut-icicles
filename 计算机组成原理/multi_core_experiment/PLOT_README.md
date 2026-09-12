# 图表生成说明

本目录已经整理好了：

- `windows_summary.csv`
- `linux_summary.csv`
- `plot_results.py`

你可以在本地有 Python 的环境中执行：

```bash
python plot_results.py
```

如果提示缺少 matplotlib，请先安装：

```bash
pip install matplotlib
```

执行成功后，会在当前目录下自动生成：

- `plots/windows_runtime.png`
- `plots/windows_speedup.png`
- `plots/windows_efficiency.png`
- `plots/linux_runtime.png`
- `plots/linux_speedup.png`
- `plots/linux_efficiency.png`

---

## 图像意义说明

### Windows 图像
1. `windows_runtime.png`
   - 横轴：线程数
   - 纵轴：平均运行时间
   - 用于展示随着线程增加，运行时间整体下降，但后期下降幅度减小

2. `windows_speedup.png`
   - 横轴：线程数
   - 纵轴：加速比
   - 用于展示加速效果并非严格线性增长

3. `windows_efficiency.png`
   - 横轴：线程数
   - 纵轴：并行效率
   - 用于展示线程数增加后，单位线程的有效利用程度下降

### Linux 图像
1. `linux_runtime.png`
   - 展示不同矩阵规模在不同线程数下的运行时间变化

2. `linux_speedup.png`
   - 展示不同矩阵规模下的加速比变化

3. `linux_efficiency.png`
   - 展示不同矩阵规模下的并行效率变化

---

## 报告中推荐怎么用

建议至少在正式报告中放入：

- Windows 线程数-运行时间图
- Windows 线程数-并行效率图
- Linux 不同规模运行时间对比图

如果篇幅允许，再补：

- Windows 加速比图
- Linux 加速比图
- Linux 并行效率图
