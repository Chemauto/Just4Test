#!/bin/bash
# 自动测试脚本

conda activate ros2_env

# 使用管道自动输入命令
(echo "0.5 0 0"; sleep 5; echo "0 0 0"; sleep 2; echo "q") | python mujoco_deploy.py
