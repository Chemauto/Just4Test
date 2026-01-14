# 三麦克纳姆轮底盘控制器使用说明

## 📋 概述

`OmniWheelController` 是用于控制三麦克纳姆轮底盘的运动控制器,基于**LeKiwi官方运动学实现**,支持全向移动(平移和旋转)。

**运动学来源**: `/home/dora/RoboOs/New/doralekiwi/lekiwi/packages/lekiwi_sim/lekiwi_sim/kinematics.py`

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `omni_controller.py` | 三麦克纳姆轮底盘控制器类 (基于LeKiwi运动学) |
| **`test_omni_viewer.py`** | **可视化演示程序(带MuJoCo仿真器)** ⭐ |
| `quick_reference.py` | 快速参考文档 |
| `README_OMNI.md` | 本文档 |

## 🚀 快速开始

### 运行可视化演示

```bash
cd /home/dora/RoboOs/New/Sim/Just4Test/Mujoco4Nano
python test_omni_viewer.py
```

### 选择演示模式

```
============================================================
三麦克纳姆轮底盘控制器演示
============================================================

选择演示:
  1 - 自动演示 (展示9种运动模式)
  2 - 圆形轨迹 (沿圆形路径移动)
  3 - 方形轨迹 (沿方形路径移动)
  4 - 简单演示 (前进 -> 旋转 -> 停止)
  5 - 手动控制 (自定义参数) ⭐

请输入选择 (1-5):
```

**💡 推荐顺序:**
1. **选项4** - 简单演示(快速了解)
2. **选项1** - 自动演示(所有9种运动)
3. **选项5** - 手动控制(自定义参数)
4. **选项2/3** - 轨迹演示

### 演示模式说明

#### 1️⃣ 自动演示
展示9种运动模式,每种3秒:
- 前进、后退、左移、右移
- 原地逆时针/顺时针旋转
- 前进+旋转组合
- 斜向运动

#### 2️⃣ 圆形轨迹
持续前进+旋转 = 圆形运动

#### 3️⃣ 方形轨迹
前进和旋转交替 = 方形路径

#### 4️⃣ 简单演示
前进(5秒) → 旋转(5秒) → 停止(3秒)

#### 5️⃣ 手动控制 ⭐
完全自定义,输入以下参数:
- `linear_speed`: 速度大小 (m/s)
- `vx`: x方向分量 (-1到1), 1=向前
- `vy`: y方向分量 (-1到1), 1=向左
- `omega`: 旋转角速度 (rad/s), 正值=逆时针
- `duration`: 持续时间 (秒)

---

## 💻 在代码中使用

### 基本使用

```python
import mujoco
import mujoco.viewer
from omni_controller import OmniWheelController

# 加载场景
model_path = "/home/dora/RoboOs/New/Sim/Just4Test/model/assets/scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
controller = OmniWheelController(model, data)

# 启动仿真器
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 前进 0.5 m/s
        controller.set_velocity(linear_speed=0.5, vx=1, vy=0, omega=0)

        # 应用控制
        controller.apply_control()

        # 仿真步进
        mujoco.mj_step(model, data)

        # 同步查看器
        viewer.sync()
```

---

## 🎮 控制函数

### set_velocity() - 主要控制函数

```python
controller.set_velocity(linear_speed, vx, vy, omega=0)
```

**参数:**

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| `linear_speed` | float | ≥0 | 速度大小 (m/s) |
| `vx` | float | -1 到 1 | x方向分量, 1=向前, -1=向后 |
| `vy` | float | -1 到 1 | y方向分量, 1=向左, -1=向右 |
| `omega` | float | 任意 | 旋转角速度 (rad/s), 正值=逆时针, 默认=0 |

**示例:**

```python
# 前进 0.5 m/s
controller.set_velocity(0.5, 1, 0, 0)

# 左移 0.3 m/s
controller.set_velocity(0.3, 0, 1, 0)

# 原地逆时针旋转 1 rad/s
controller.set_velocity(0, 0, 0, 1)

# 前进同时旋转
controller.set_velocity(0.3, 1, 0, 0.5)

# 停止
controller.stop()
```

### set_velocity_raw() - 直接控制

```python
controller.set_velocity_raw(vx, vy, omega=0)
```

直接指定速度分量(m/s),不归一化。

---

## 📝 代码示例

### 示例1: 基本运动

```python
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 前进 5秒
    start = time.time()
    while time.time() - start < 5:
        controller.set_velocity(0.5, 1, 0, 0)
        controller.apply_control()
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 示例2: 方形路径

```python
with mujoco.viewer.launch_passive(model, data) as viewer:
    state = 0  # 0=前进, 1=旋转
    timer = 0

    while viewer.is_running():
        if state == 0:  # 前进3秒
            controller.set_velocity(0.5, 1, 0, 0)
            timer += model.opt.timestep
            if timer >= 3.0:
                state = 1
                timer = 0
        elif state == 1:  # 旋转90度(1.57秒)
            controller.set_velocity(0, 0, 0, 1)
            timer += model.opt.timestep
            if timer >= 1.57:
                state = 0
                timer = 0

        controller.apply_control()
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 示例3: 圆形路径

```python
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 持续前进+旋转 = 圆形
        controller.set_velocity(0.3, 1, 0, 0.5)
        controller.apply_control()
        mujoco.mj_step(model, data)
        viewer.sync()
```

---

## 📚 API参考

### OmniWheelController 类

| 方法 | 说明 |
|------|------|
| `set_velocity(linear_speed, vx, vy, omega=0)` | 设置速度(推荐) |
| `set_velocity_raw(vx, vy, omega=0)` | 直接设置速度分量 |
| `apply_control()` | 应用控制到执行器 |
| `stop()` | 停止所有轮子 |
| `get_wheel_velocities()` | 返回 (左轮, 右轮, 后轮) 角速度 |
| `get_robot_position()` | 返回 [x, y, z] 位置 |
| `get_robot_orientation()` | 返回 [w, x, y, z] 四元数姿态 |

---

## 🔧 运动学实现

### 轮子配置
- **布局**: 120°间隔的三轮全向轮
- **左轮**: 30° to forward direction
- **右轮**: 150° from forward
- **后轮**: 270° from forward

### 运动学矩阵

使用**LeKiwi官方**的标准运动学矩阵:

```python
F_matrix = r * [[√3/2, -√3/2, 0],
                 [-1/2,  -1/2,   1],
                 [-1/(3L), -1/(3L), -1/(3L)]]
```

其中:
- `r` = 轮子半径 (0.051 m)
- `L` = 机器人半径 (0.0923 m)

这个矩阵将机器人速度 `[vx, vy, omega]` 映射到轮子角速度 `[left, right, back]`。

---

## ⚙️ 系统参数

| 参数 | 值 |
|------|-----|
| 执行器速度范围 | -3.14 到 3.14 rad/s |
| 轮子半径 | 0.051 m |
| 机器人半径 | 0.0923 m |
| 推荐最大线速度 | ~0.5 m/s |
| 推荐最大旋转速度 | ~2.0 rad/s |

---

## 🎯 使用建议

### 速度选择
- **精细操作**: 0.2-0.3 m/s
- **一般移动**: 0.4-0.5 m/s
- **快速移动**: 0.5-0.8 m/s

### 旋转速度
- **精细旋转**: 0.3-0.5 rad/s
- **一般旋转**: 0.8-1.0 rad/s
- **快速旋转**: 1.5-2.0 rad/s

### 组合运动
平移+旋转时降低速度,避免执行器饱和:
```python
# ✓ 好
controller.set_velocity(0.3, 1, 0, 0.5)

# ✗ 可能饱和
controller.set_velocity(0.5, 1, 0, 1.0)
```

---

## ❓ 常见问题

### Q: 有轻微偏转正常吗?
A: **是的**,这是正常现象。可能由以下原因造成:
- 物理仿真中的摩擦力不均匀
- 轮子制造公差
- 数值积分误差

偏转量通常很小(几厘米),实际应用中可接受。

### Q: 如何精确旋转90度?
A: 使用时间控制:
```python
import time
start = time.time()
while time.time() - start < 1.57:  # π/2 / 1.0
    controller.set_velocity(0, 0, 0, 1.0)
    controller.apply_control()
    mujoco.mj_step(model, data)
```

### Q: 如何实现平滑加速?
A: 渐变速度:
```python
for i in range(100):
    speed = 0.5 * i / 100  # 0 → 0.5
    controller.set_velocity(speed, 1, 0, 0)
    controller.apply_control()
    mujoco.mj_step(model, data)
```

---

## 📖 相关资源

- **LeKiwi项目**: `/home/dora/RoboOs/New/doralekiwi/lekiwi`
- **运动学源码**: `packages/lekiwi_sim/lekiwi_sim/kinematics.py`
- **MuJoCo文档**: https://mujoco.readthedocs.io/

---

**版本**: 2.0 (基于LeKiwi官方运动学)
**最后更新**: 2026-01-14
**状态**: ✅ 已测试验证
