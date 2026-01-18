# Unitree Go2 MuJoCo 部署

将IsaacLab训练的Unitree Go2行走策略部署到MuJoCo仿真环境。

## 快速开始

```bash
conda activate ros2_env
python mujoco_deploy.py
```

## 交互界面

程序启动后会在终端显示交互菜单，**机器人将持续执行当前命令**：

```
==============================================================
MuJoCo策略部署
==============================================================

重置到初始状态...
基座高度: 0.400

开始仿真...
==============================================================
当前命令: vx=0.00, vy=0.00, wz=0.00
输入新命令 (vx vy wz) 改变运动，例如: 0.5 0 0
输入 'q' 或按 Ctrl+C 退出
==============================================================

> 0.5 0 0          ← 在这里输入命令
✓ 命令已更新: vx=0.50, vy=0.00, wz=0.00

步骤:120 | 命令:[0.50, 0.00, 0.00] | 位置:[0.15, 0.02, 0.40] | 速度:[0.48, 0.01]
```

## 常用命令

```
0.5 0 0    - 前进 0.5 m/s（机器人持续执行）
s          - 停止（保持站立）
q          - 退出
```

## 文件说明

- **`mujoco_deploy.py`** - 主程序（持续执行模式）
- **`DEPLOY_SUCCESS.md`** - 详细技术文档
- **`USAGE_GUIDE.md`** - 使用指南
- **`README.md`** - 本文件

## 策略信息

- **文件**: `../policy/Rough_Walk_policy4Mujoco.pt`
- **训练**: 499次迭代
- **状态**: ✅ 成功部署，机器人保持站立（基座高度 ~0.40m）

## 训练配置参考

- **训练配置**: `/home/robot/work/BiShe/IsaacLabBisShe/source/MyProject/MyProject/tasks/manager_based/WalkTest/walk_test_env_cfg.py`
- **IsaacLab配置**: `/home/robot/work/IsaacLab-main/source/isaaclab_assets/isaaclab_assets/robots/unitree.py`

## 交互方式

- ✅ **持续执行模式** - 机器人持续执行当前命令
- 随时输入新命令改变运动方向
- 显示实时位置和速度信息（每秒更新）
- 支持 Ctrl+C 中断或输入 'q' 退出
