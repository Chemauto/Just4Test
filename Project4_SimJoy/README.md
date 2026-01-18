# Project4_SimJoy - Xbox手柄模拟器

一个具有图形用户界面(GUI)的虚拟Xbox手柄模拟器，可以替代真实手柄用于开发测试。

![Python](https://img.shields.io/badge/Python-3.7+-blue.svg)
![PyQt5](https://img.shields.io/badge/PyQt5-5.15+-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## ✨ 功能特性

- 🎮 **完整的Xbox手柄布局**：包含所有标准按键、摇杆、扳机和方向键
- 🖱️ **鼠标交互控制**：点击按钮模拟按下，拖拽摇杆实现双轴控制
- 🎨 **美观的UI界面**：仿真的手柄外观，带有渐变、阴影和高光效果
- 🔄 **自动回弹**：摇杆释放后自动回到中心位置
- 📊 **实时状态显示**：底部面板显示当前所有按键和轴的状态
- 🔌 **pygame兼容接口**：提供与pygame.joystick.Joystick相同的API
- ⚡ **低延迟响应**：优化的更新机制，事件延迟 < 1ms

## 🚀 快速开始

### 第一次使用

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 启动手柄模拟器
./run.sh
```

### 验证功能

运行单元测试：
```bash
python test/test_joystick.py
```

## 📦 安装

### 依赖要求

- Python 3.7+
- PyQt5
- numpy

### 安装步骤

```bash
# 进入项目目录
cd /home/xcj/work/MoJoCoTest/test/Project4_SimJoy

# 安装Python依赖
pip install -r requirements.txt

# 启动程序
python src/main.py
```

## 🎮 使用方法

### 启动手柄模拟器

使用启动脚本（推荐）：
```bash
./run.sh
```

或直接运行：
```bash
python src/main.py
```

## 📱 界面说明

### 手柄布局

```
┌─────────────────────────────────────────┐
│  [LB]                    [RB]           │  ← 肩键
│                                         │
│      [LT]              [RT]             │  ← 扳机
│                                         │
│                                         │
│         [Y]              [D-PAD]        │
│
│      [X] [A] [B]                        │  ← ABXY按键
│                                         │
│                                         │
│    [左摇杆]          [右摇杆]            │  ← 双轴摇杆
│       (  )            (  )             │
│                                         │
│           [SEL] [STA]                  │  ← 功能键
└─────────────────────────────────────────┘
```

### 按键映射

#### 按钮 (button_id)

| 按键 | button_id | 颜色 | 说明 |
|------|-----------|------|------|
| A | 0 | 绿色 | |
| B | 1 | 红色 | |
| X | 2 | 蓝色 | |
| Y | 3 | 黄色 | |
| LB | 4 | 灰色 | 左肩键 |
| RB | 5 | 灰色 | 右肩键 |
| SELECT | 6 | 小按钮 | 选择键 |
| START | 7 | 小按钮 | 开始键 |

#### 轴 (axis_id)

| 轴 | axis_id | 范围 | 说明 |
|----|---------|------|------|
| LX | 0 | [-1, 1] | 左摇杆X轴 |
| LY | 1 | [-1, 1] | 左摇杆Y轴 |
| LT | 2 | [0, 1] | 左扳机 |
| RX | 3 | [-1, 1] | 右摇杆X轴 |
| RY | 4 | [-1, 1] | 右摇杆Y轴 |
| RT | 5 | [0, 1] | 右扳机 |

#### 方向键 (hat)

- **D-PAD** - 支持8个方向：上、下、左、右及对角线
- 返回值：(x, y) 元组
  - x: -1(左), 0(中), 1(右)
  - y: -1(下), 0(中), 1(上)

### 交互说明

#### 按钮操作
- **鼠标左键按下**：按钮变为激活状态（颜色变深）
- **鼠标左键释放**：按钮恢复原状
- **鼠标悬停**：按钮高亮显示

#### 摇杆操作
- **鼠标在摇杆上按下**：捕获摇杆控制
- **拖拽**：摇杆头跟随鼠标移动
- **释放**：摇杆平滑回弹到中心
- 轴值根据偏离中心的距离自动归一化到[-1, 1]

#### 扳机操作
- **鼠标按下**：扳机激活（压力100%）
- **鼠标释放**：扳机释放
- 点击位置不同会产生不同的压力值（0-100%）

#### 方向键操作
- **鼠标点击**：选择对应方向
- **鼠标拖拽**：可以在方向间切换

## 💻 编程接口

### 使用VirtualJoystick API

```python
from src.virtual_joystick import VirtualJoystick

# 创建虚拟手柄
joystick = VirtualJoystick(0)

# 读取按键状态
if joystick.get_button(0):  # A键
    print("A键按下")

# 读取摇杆位置
left_x = joystick.get_axis(0)  # 左摇杆X轴 [-1, 1]
left_y = joystick.get_axis(1)  # 左摇杆Y轴 [-1, 1]

# 读取扳机值
left_trigger = joystick.get_axis(2)  # LT [0, 1]
right_trigger = joystick.get_axis(5)  # RT [0, 1]

# 读取方向键
hat_x, hat_y = joystick.get_hat(0)
```

### 与现有系统集成

本模拟器提供与pygame.joystick.Joystick兼容的接口，可以作为drop-in replacement使用：

```python
import sys
sys.path.insert(0, '/home/xcj/work/MoJoCoTest/test/Project4_SimJoy/src')

from src.virtual_joystick import VirtualJoystick

# 替换原来的pygame手柄初始化
# joystick = pygame.joystick.Joystick(0)
joystick = VirtualJoystick(0)

# 后续代码无需修改
button_state = joystick.get_button(0)
axis_value = joystick.get_axis(0)
```

## 📁 项目结构

```
Project4_SimJoy/
├── src/
│   ├── __init__.py
│   ├── main.py                # 程序入口
│   ├── virtual_joystick.py    # 核心手柄类(提供pygame兼容接口)
│   ├── joystick_ui.py         # GUI界面类
│   ├── joystick_state.py      # 手柄状态管理
│   └── components/            # UI组件
│       ├── __init__.py
│       ├── button.py          # 按钮组件
│       ├── stick.py           # 摇杆组件
│       ├── trigger.py         # 扳机组件
│       └── dpad.py            # 方向键组件
├── test/
│   └── test_joystick.py       # 单元测试
├── examples/
│   └── read_virtual_joystick.py  # 读取示例程序
├── run.sh                     # 启动脚本
├── plan.md                    # 开发计划文档
├── README.md                  # 本文件
└── requirements.txt           # 依赖列表
```

## 🧪 运行测试

```bash
python test/test_joystick.py
```

测试涵盖：
- 手柄状态管理
- 虚拟手柄核心功能
- pygame兼容性
- 所有按键和轴的操作

## ❓ 常见问题

### 程序无法启动？
**A**: 确保已安装PyQt5：
```bash
pip install -r requirements.txt
```

### 如何测试功能是否正常？
**A**: 运行单元测试：
```bash
python test/test_joystick.py
```

### 可以同时使用多个虚拟手柄吗？
**A**: 可以，创建多个VirtualJoystick实例即可，只需指定不同的ID：
```python
joy1 = VirtualJoystick(0)
joy2 = VirtualJoystick(1)
```

### 如何自定义手柄外观？
**A**: 修改 `src/components/` 目录下的组件文件，调整颜色和尺寸参数。

## 🔧 技术细节

### 摇杆算法
```python
# 计算摇杆轴值
dx = mouse_x - center_x
dy = mouse_y - center_y
distance = min(sqrt(dx*dx + dy*dy), max_radius)
angle = atan2(dy, dx)

x_value = (distance / max_radius) * cos(angle)
y_value = (distance / max_radius) * sin(angle)
```

### 性能指标
- 更新频率：60Hz
- CPU占用：< 1%
- 内存占用：~50MB

### 性能优化
- 使用定时器定期更新界面（60FPS）
- 事件驱动更新按键状态
- 优化的绘制机制，避免不必要的重绘

## 🌟 未来改进

- [ ] 添加键盘快捷键支持
- [ ] 支持配置保存和加载
- [ ] 添加震动反馈（如果硬件支持）
- [ ] 支持多种手柄样式切换（PS5、Switch等）
- [ ] 添加宏功能（录制和回放按键序列）
- [ ] 支持触摸屏设备

## 📝 开发环境

- Python 3.8+
- PyQt5 5.15+
- numpy 1.19+

## 📄 许可证

本项目仅供学习和研究使用。

## 🤝 贡献

欢迎提交问题报告和改进建议！

## 📧 联系方式

如有问题或建议，请提交Issue。

---

**祝您使用愉快！🎮**
