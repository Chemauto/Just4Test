#!/usr/bin/env python3
"""
自动测试部署 - 机器人持续执行命令
"""
import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import time
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import torch
import sys

# 导入主程序的类
from mujoco_deploy import MuJoCoDemo

def auto_test():
    """自动测试"""
    model_path = "/home/robot/work/LekiwiTest/UntrieeGo2/model/go2/scene.xml"
    policy_path = "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt"

    demo = MuJoCoDemo(model_path, policy_path)

    print("\n" + "="*60)
    print("自动测试模式")
    print("="*60)
    print("\n测试序列:")
    print("  1. 站立 2秒")
    print("  2. 前进 4秒 (vx=0.3)")
    print("  3. 停止 2秒")
    print("  4. 原地旋转 3秒 (wz=0.5)")
    print("  5. 停止 2秒")
    print("  6. 退出")
    print("="*60 + "\n")

    # 重置
    demo.reset_to_initial_state()
    time.sleep(0.5)

    # 运行自动测试
    with mujoco.viewer.launch_passive(demo.model, demo.data) as viewer:
        step_count = 0

        # 测试序列: (vx, vy, wz, duration)
        test_sequence = [
            (0.0, 0.0, 0.0, 2.0),  # 站立
            (0.3, 0.0, 0.0, 4.0),  # 前进
            (0.0, 0.0, 0.0, 2.0),  # 停止
            (0.0, 0.0, 0.5, 3.0),  # 旋转
            (0.0, 0.0, 0.0, 2.0),  # 停止
        ]

        seq_idx = 0
        seq_start_time = time.time()

        try:
            while viewer.is_running() and seq_idx < len(test_sequence):
                # 获取当前命令
                vx, vy, wz, duration = test_sequence[seq_idx]
                demo.vx, demo.vy, demo.wz = vx, vy, wz

                # 检查是否切换到下一个命令
                elapsed = time.time() - seq_start_time
                if elapsed >= duration:
                    seq_idx += 1
                    seq_start_time = time.time()
                    if seq_idx < len(test_sequence):
                        next_cmd = test_sequence[seq_idx]
                        print(f"\n切换命令: vx={next_cmd[0]:.1f}, vy={next_cmd[1]:.1f}, wz={next_cmd[2]:.1f}")

                # 执行控制
                demo.step()

                # 每100步打印一次
                step_count += 1
                if step_count % 100 == 0:
                    base_pos = demo.data.qpos[0:3]
                    base_vel = demo.data.qvel[0:3]
                    print(f"\r步骤:{step_count} | 命令:[{vx:.1f}, {vy:.1f}, {wz:.1f}] | "
                          f"位置:[{base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}] | "
                          f"速度:[{base_vel[0]:.2f}, {base_vel[1]:.2f}]", end="", flush=True)

                viewer.sync()
                time.sleep(1.0/60.0)

        except KeyboardInterrupt:
            print("\n\n用户中断")

    print("\n\n测试完成!")
    print(f"总步数: {step_count}")

if __name__ == "__main__":
    auto_test()
