# Linux 服务器运行说明

把整个 `multi_core_experiment` 文件夹传到 Linux 服务器后，按下面步骤执行。

---

## 1. 进入目录
```bash
cd multi_core_experiment
```

## 2. 赋予脚本执行权限
```bash
chmod +x run_tests.sh
```

## 3. 运行实验
```bash
./run_tests.sh
```

---

## 运行前建议先看一下环境
建议顺手执行并保存输出：

```bash
uname -a
lscpu
free -h
g++ --version
nproc
```

如果 `g++` 不存在，需要先安装。

---

## 这个脚本会做什么
它会自动：

- 编译 `matrix_omp.cpp`
- 运行矩阵规模：`600 1000 1400`
- 运行线程数：`1 2 4 8`
- 每组重复 3 次
- 生成结果文件：

```bash
results_linux.csv
```

---

## 如果服务器内存紧张
可以先编辑 `run_tests.sh`，把：

```bash
MATRIX_SIZES=(600 1000 1400)
```

改成：

```bash
MATRIX_SIZES=(600 1000)
```

先跑小一点的规模，确认稳定后再补跑 1400。

---

## 跑完后请发回这些文件
至少发我：

- `results_linux.csv`

如果方便，也请附带：

- `uname -a` 输出
- `lscpu` 输出
- `free -h` 输出
- `g++ --version` 输出
- `nproc` 输出
- Linux 终端运行截图

---

## Windows 这边建议补跑
由于你之前 Windows 的 1线程数据波动较大，建议本地额外运行：

```bash
rerun_low_threads.bat
```

它会补跑：
- 1线程
- 2线程
- 4线程

每组 5 次，并输出：

```bash
results_windows_low_threads.csv
```

这个文件也请一起发我。
