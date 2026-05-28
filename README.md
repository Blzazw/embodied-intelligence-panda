# 具身智能项目：PyBullet 仿真机械臂控制

## 项目简介
本项目使用 PyBullet 物理仿真引擎，控制 Franka Panda 机械臂完成精确的末端执行器定位、物体抓取与搬运任务。通过逆运动学求解和力控制，实现了"抓取-移动-释放"的完整流水线。

## 项目结构
```
embodied-intelligence/
├── main.py                  # 主程序：机械臂逆向运动学定位
├── grasp_demo.py            # 抓取-移动-释放完整流水线
├── parameter_comparison.py  # 多参数对比实验
├── requirements.txt         # 依赖环境
├── README.md               # 本文件
├── report/                  # 报告输出目录
│   └── 具身智能项目报告.docx
└── ppt/                     # PPT输出目录
    └── 具身智能项目展示.pptx
```

## 安装与运行

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 运行基础演示（机械臂移动到目标点）
```bash
python main.py
```

### 3. 运行抓取演示（完整流水线）
```bash
python grasp_demo.py
```

### 4. 运行参数对比实验
```bash
python parameter_comparison.py
```

## 核心功能

### 基础定位 (`main.py`)
- 加载 Franka Panda 机械臂 URDF 模型
- 使用逆运动学计算关节角度
- 控制机械臂末端移动到指定三维坐标
- 实时显示关节角度和末端位置

### 抓取流水线 (`grasp_demo.py`)
- 完整的"抓取-移动-释放"流程
- 支持自定义目标位置和抓取高度
- 基于力传感器的抓取检测
- 分阶段运动控制

### 参数对比 (`parameter_comparison.py`)
- 对比不同目标位置下的机械臂姿态
- 记录并输出各关节角度变化
- 分析逆运动学解的连续性

## 依赖环境
- Python 3.8+
- PyBullet >= 3.2.5
- NumPy >= 1.21.0
- 操作系统：Windows / macOS / Linux
