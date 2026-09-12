快速使用说明

一、准备环境
1. 需要安装 g++，并且支持 OpenMP。
2. Windows 和 Linux 都可以运行同一份 `matrix_omp.cpp`。

二、文件说明
- `matrix_omp.cpp`：主实验代码
- `run_tests.bat`：Windows 一键测试脚本
- `run_tests.sh`：Linux 一键测试脚本
- `results_template.csv`：结果格式示意

三、Windows 运行方式
1. 直接双击 `run_tests.bat`
2. 它会自动：
   - 编译 `matrix_omp.cpp`
   - 使用 `MATRIX_SIZE=2400`
   - 测试线程数 `1 2 4 8 16 24 32 48 64`
   - 每组重复 3 次
   - 把结果保存到 `results_windows.csv`

四、Linux 运行方式
1. 进入实验文件夹后运行：
   ```bash
   chmod +x run_tests.sh
   ./run_tests.sh
   ```
2. 它会自动：
   - 编译 `matrix_omp.cpp`
   - 测试矩阵规模 `600 1000 1400`
   - 测试线程数 `1 2 4 8`
   - 每组重复 3 次
   - 把结果保存到 `results_linux.csv`

五、程序单独运行方式
Windows 示例：
```bash
matrix_omp.exe 2400 windows_laptop 8
```

Linux 示例：
```bash
./matrix_omp 1000 linux_server 4
```

参数含义依次为：
1. 矩阵规模
2. 机器标签
3. 请求线程数（用于记录）

六、输出结果说明
程序会输出：
- Machine label
- Matrix size
- Requested threads
- Actual threads
- Time
- Check value
- 一行以 `CSV,` 开头的机器可读结果

批处理脚本和 shell 脚本会自动提取这行结果，写入 CSV 文件，便于后续统计平均值、加速比和并行效率。

七、如果编译失败
说明当前机器里暂时没有可用的 g++ 编译器，或者当前 g++ 不支持 `-fopenmp`。

八、如果要调整规模
### Windows
修改 `run_tests.bat` 中：
```bat
set MATRIX_SIZE=2400
```

### Linux
修改 `run_tests.sh` 中：
```bash
MATRIX_SIZES=(600 1000 1400)
```

九、结果回传建议
实验完成后，请把以下文件发回：
- `results_windows.csv`
- `results_linux.csv`
- 如有的话，再附上任务管理器截图或 Linux 终端截图

这样后续可以直接生成实验报告部分和图表分析。
