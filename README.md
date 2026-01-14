# MuJoCo 小车控制系统 / MuJoCo Car Control System

本目录包含 MuJoCo 小车仿真的控制系统。/ This directory contains MuJoCo car simulation control systems.

---

## 📂 项目结构 / Project Structure

```
Just4Test/
├── model/                     # 仿真模型
├── Mujoco4CarTest/           # 双轮差速小车 (2WD)
│   ├── control.py           # 控制器类
│   ├── keyboard.py          # 键盘控制
│   └── demo.py              # 演示脚本
└── Mujoco4Nano/             # 三麦克纳姆轮全向移动平台 (3WD Omni) ⭐
    ├── omni_controller.py   # 全向轮控制器
    ├── test_omni_viewer.py # 可视化演示
    └── README_OMNI.md       # 详细文档
```

---

## 🚗 快速开始 / Quick Start

### 1. 双轮差速小车 (2WD Car)

```bash
cd Mujoco4CarTest

# 键盘控制
python keyboard.py

# 自动演示
python demo.py
```

**控制键 / Controls:**
- `w/i`: 前进 / Forward
- `s/k`: 后退 / Backward
- `a/j`: 左转 / Turn left
- `d/l`: 右转 / Turn right
- `Space`: 停止 / Stop
- `q`: 退出 / Quit

### 2. 三麦克纳姆轮平台 (3WD Omni Platform) ⭐

```bash
cd Mujoco4Nano

# 运行可视化演示
python test_omni_viewer.py

# 选择演示模式:
# 1 - 自动演示 (9种运动模式)
# 2 - 圆形轨迹
# 3 - 方形轨迹
# 4 - 简单演示
# 5 - 手动控制 (自定义参数)
```

**特性 / Features:**
- ✅ 全向移动 (前后左右 + 原地旋转)
- ✅ 基于 LeKiwi 官方运动学实现
- ✅ 可视化仿真界面

---

## 📖 使用文档 / Documentation

- **Mujoco4Nano**: 详见 `README_OMNI.md`
- **Mujoco4CarTest**: 详见 `README.md` (当前文件)

---

## 💡 代码示例 / Code Examples

### 双轮差速小车 / 2WD Car

```python
from control import CarController
import mujoco

model = mujoco.MjModel.from_xml_path('model/car.xml')
data = mujoco.MjData(model)
controller = CarController(model, data)

# 控制: 前进0.5, 转向0.3
controller.set_control(forward=0.5, turn=0.3)
controller.apply_control()
mujoco.mj_step(model, data)
```

### 三麦克纳姆轮平台 / 3WD Omni Platform

```python
from omni_controller import OmniWheelController
import mujoco

model = mujoco.MjModel.from_xml_path('model/assets/scene.xml')
data = mujoco.MjData(model)
controller = OmniWheelController(model, data)

# 全向移动: 前进 0.5 m/s
controller.set_velocity(linear_speed=0.5, vx=1, vy=0, omega=0)
controller.apply_control()
mujoco.mj_step(model, data)
```

---

## ⚙️ 控制器对比 / Controller Comparison

| 特性 | 2WD Car | 3WD Omni |
|------|---------|----------|
| **移动方式** | 前进 + 转向 | 全向移动 (前后左右 + 旋转) |
| **执行器数** | 2 | 3 |
| **控制接口** | `set_control(forward, turn)` | `set_velocity(speed, vx, vy, omega)` |
| **运动自由度** | 2 DOF | 3 DOF (全向) |
| **适用场景** | 简单移动 | 精确定位、狭窄空间 |

---

## 📦 依赖要求 / Requirements

```bash
pip install mujoco numpy
```

---

## 📚 参考资料 / References

- **MuJoCo**: https://mujoco.readthedocs.io/
- **LeKiwi项目**: `/home/dora/RoboOs/New/doralekiwi/lekiwi`

---

**最后更新 / Last Updated**: 2026-01-14
