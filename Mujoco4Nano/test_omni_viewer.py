"""
三麦克纳姆轮底盘运动控制测试示例 - 带可视化
演示如何使用 OmniWheelController 控制机器人运动,并在MuJoCo查看器中显示
"""

import time
import mujoco
import mujoco.viewer
from omni_controller import OmniWheelController


def run_auto_demo(model_path, duration=30):
    """
    自动演示模式 - 机器人将展示多种运动模式

    参数:
        model_path: XML 模型路径
        duration: 运行时长（秒）
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = OmniWheelController(model, data)

    print("\n" + "="*60)
    print("三麦克纳姆轮底盘 - 自动演示模式")
    print("="*60)
    print("\n运动阶段:")
    print("  0-3s:   前进")
    print("  3-6s:   后退")
    print("  6-9s:   左移")
    print("  9-12s:  右移")
    print("  12-15s: 原地逆时针旋转")
    print("  15-18s: 原地顺时针旋转")
    print("  18-21s: 前进+逆时针旋转")
    print("  21-24s: 右前斜向45度")
    print("  24-27s: 停止")
    print("="*60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        phase_start_time = time.time()
        phase = 0
        phase_duration = 3.0

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # 每3秒切换一个运动模式
            if time.time() - phase_start_time > phase_duration:
                phase = (phase + 1) % 9
                phase_start_time = time.time()

                # 打印新阶段
                phase_names = [
                    "前进",
                    "后退",
                    "左移",
                    "右移",
                    "原地逆时针旋转",
                    "原地顺时针旋转",
                    "前进+逆时针旋转",
                    "右前斜向45度",
                    "停止"
                ]
                print(f"[Phase {phase}] {phase_names[phase]}")

            # 根据当前阶段设置不同的运动
            if phase == 0:
                controller.set_velocity(linear_speed=0.5, vx=1, vy=0, omega=0)
            elif phase == 1:
                controller.set_velocity(linear_speed=0.5, vx=-1, vy=0, omega=0)
            elif phase == 2:
                controller.set_velocity(linear_speed=0.5, vx=0, vy=1, omega=0)
            elif phase == 3:
                controller.set_velocity(linear_speed=0.5, vx=0, vy=-1, omega=0)
            elif phase == 4:
                controller.set_velocity(linear_speed=0, vx=0, vy=0, omega=1.0)
            elif phase == 5:
                controller.set_velocity(linear_speed=0, vx=0, vy=0, omega=-1.0)
            elif phase == 6:
                controller.set_velocity(linear_speed=0.3, vx=1, vy=0, omega=0.5)
            elif phase == 7:
                controller.set_velocity(linear_speed=0.5, vx=1, vy=-1, omega=0)
            elif phase == 8:
                controller.stop()

            # 应用控制
            controller.apply_control()

            # 物理步进
            mujoco.mj_step(model, data)

            # 打印状态
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_robot_position()
                left, right, back = controller.get_wheel_velocities()
                print(f"\r时间: {data.time:.2f}s | 位置: [{pos[0]:.2f}, {pos[1]:.2f}] | 轮子: [{left:.2f}, {right:.2f}, {back:.2f}]", end="")

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        # 停止机器人
        controller.stop()
        controller.apply_control()

    print("\n\n演示完成!")


def run_circle_demo(model_path, duration=20):
    """
    圆形轨迹演示 - 机器人沿圆形路径移动

    参数:
        model_path: XML 模型路径
        duration: 运行时长（秒）
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = OmniWheelController(model, data)

    print("\n" + "="*60)
    print("圆形演示 - 机器人沿圆形路径移动")
    print("="*60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # 圆形运动: 前进 + 持续旋转
            controller.set_velocity(linear_speed=0.3, vx=1, vy=0, omega=0.5)
            controller.apply_control()

            # 物理步进
            mujoco.mj_step(model, data)

            # 打印状态
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_robot_position()
                print(f"\r时间: {data.time:.2f}秒 | 位置: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_square_demo(model_path, duration=30):
    """
    方形轨迹演示 - 机器人沿方形路径移动

    参数:
        model_path: XML 模型路径
        duration: 运行时长（秒）
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = OmniWheelController(model, data)

    print("\n" + "="*60)
    print("方形演示 - 机器人沿方形路径移动")
    print("="*60 + "\n")

    # 状态机
    # 状态: 0=前进, 1=旋转90度, 2=前进, 3=旋转90度, 等等
    state = 0
    state_timer = 0
    forward_duration = 3.0  # 前进持续时间（秒）
    rotate_duration = 1.57  # 旋转90度需要的时间 (π/2 / 1 rad/s)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # 状态机逻辑
            if state == 0:  # 前进状态
                controller.set_velocity(linear_speed=0.5, vx=1, vy=0, omega=0)
                state_timer += model.opt.timestep
                if state_timer >= forward_duration:
                    state = 1
                    state_timer = 0
            elif state == 1:  # 旋转状态
                controller.set_velocity(linear_speed=0, vx=0, vy=0, omega=1.0)
                state_timer += model.opt.timestep
                if state_timer >= rotate_duration:
                    state = 0
                    state_timer = 0
                    print(f"\n完成一边，时间: {time.time() - start_time:.1f}秒")

            controller.apply_control()

            # 物理步进
            mujoco.mj_step(model, data)

            # 打印状态
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_robot_position()
                state_name = "前进" if state == 0 else "旋转"
                print(f"\r时间: {data.time:.2f}秒 | 状态: {state_name} | 位置: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_simple_demo(model_path, duration=15):
    """
    简单演示 - 只展示前进和旋转

    参数:
        model_path: XML 模型路径
        duration: 运行时长（秒）
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = OmniWheelController(model, data)

    print("\n" + "="*60)
    print("简单演示 - 前进 -> 旋转 -> 停止")
    print("="*60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()
            elapsed = time.time() - start_time

            # 前5秒: 前进
            if elapsed < 5:
                controller.set_velocity(linear_speed=0.5, vx=1, vy=0, omega=0)
                if elapsed < 0.1:
                    print("阶段1: 前进 (0-5秒)\n")

            # 5-10秒: 旋转
            elif elapsed < 10:
                controller.set_velocity(linear_speed=0, vx=0, vy=0, omega=1.0)
                if elapsed < 5.1:
                    print("阶段2: 原地旋转 (5-10秒)\n")

            # 10-15秒: 停止
            else:
                controller.stop()
                if elapsed < 10.1:
                    print("阶段3: 停止 (10-15秒)\n")

            # 应用控制
            controller.apply_control()

            # 物理步进
            mujoco.mj_step(model, data)

            # 打印状态
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_robot_position()
                left, right, back = controller.get_wheel_velocities()
                print(f"\r时间: {data.time:.2f}s | 位置: [{pos[0]:.2f}, {pos[1]:.2f}] | 轮子: [{left:.2f}, {right:.2f}, {back:.2f}]", end="")

            # 同步查看器
            viewer.sync()

            # 时间控制
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_manual_control(model_path):
    """
    手动控制模式 - 用户输入参数控制机器人

    参数:
        model_path: XML 模型路径
    """
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    controller = OmniWheelController(model, data)

    print("\n" + "="*60)
    print("手动控制模式")
    print("="*60)
    print("\n参数说明:")
    print("  linear_speed: 速度大小 (m/s), 例如: 0.5")
    print("  vx: x方向分量 (-1到1)")
    print("       1 = 向前, -1 = 向后, 0 = 无x方向运动")
    print("  vy: y方向分量 (-1到1)")
    print("       1 = 向左, -1 = 向右, 0 = 无y方向运动")
    print("  omega: 旋转角速度 (rad/s)")
    print("         正值 = 逆时针, 负值 = 顺时针, 0 = 不旋转")
    print("  duration: 控制时间 (秒)")
    print("\n示例:")
    print("  前进5秒: speed=0.5, vx=1, vy=0, omega=0, duration=5")
    print("  左移3秒: speed=0.3, vx=0, vy=1, omega=0, duration=3")
    print("  旋转2秒: speed=0, vx=0, vy=0, omega=1, duration=2")
    print("="*60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            try:
                # 获取用户输入
                print("\n--- 新的控制指令 ---")
                linear_speed = float(input("输入速度大小 linear_speed (m/s, 例如0.5): ") or "0")
                vx = float(input("输入x方向分量 vx (-1到1, 例如1=向前): ") or "0")
                vy = float(input("输入y方向分量 vy (-1到1, 例如1=向左): ") or "0")
                omega = float(input("输入旋转角速度 omega (rad/s, 正值=逆时针): ") or "0")
                duration = float(input("输入持续时间 duration (秒, 例如5): ") or "5")

                print(f"\n执行: speed={linear_speed}, vx={vx}, vy={vy}, omega={omega}, duration={duration}秒")
                print("按 Ctrl+C 可提前停止...\n")

                start_time = time.time()

                # 执行控制
                while viewer.is_running() and (time.time() - start_time) < duration:
                    step_start = time.time()

                    # 设置速度
                    controller.set_velocity(linear_speed=linear_speed, vx=vx, vy=vy, omega=omega)
                    controller.apply_control()

                    # 物理步进
                    mujoco.mj_step(model, data)

                    # 打印状态
                    if int(data.time * 100) % 50 == 0:
                        pos = controller.get_robot_position()
                        left, right, back = controller.get_wheel_velocities()
                        elapsed = time.time() - start_time
                        print(f"\r时间: {elapsed:.2f}s/{duration}s | 位置: [{pos[0]:.2f}, {pos[1]:.2f}] | 轮子: [{left:.2f}, {right:.2f}, {back:.2f}]", end="")

                    # 同步查看器
                    viewer.sync()

                    # 时间控制
                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)

                # 时间到,停止机器人
                controller.stop()
                controller.apply_control()
                print(f"\n\n控制完成! 运行时间: {duration}秒\n")

            except KeyboardInterrupt:
                # 用户按Ctrl+C
                controller.stop()
                controller.apply_control()
                print("\n\n当前控制已停止")
                continue
            except ValueError as e:
                print(f"\n输入错误: {e}")
                print("请重新输入!\n")
                continue
            except Exception as e:
                print(f"\n错误: {e}")
                import traceback
                traceback.print_exc()
                break

    print("\n手动控制模式结束")


def main():
    """主入口函数 - 运行不同的演示"""
    model_path = '/home/dora/RoboOs/New/Sim/Just4Test/model/assets/scene.xml'

    print("\n" + "="*60)
    print("三麦克纳姆轮底盘控制器演示")
    print("="*60)
    print("\n选择演示:")
    print("  1 - 自动演示 (展示9种运动模式)")
    print("  2 - 圆形轨迹 (沿圆形路径移动)")
    print("  3 - 方形轨迹 (沿方形路径移动)")
    print("  4 - 简单演示 (前进 -> 旋转 -> 停止)")
    print("  5 - 手动控制 (自定义参数)")

    try:
        choice = input("\n请输入选择 (1-5): ").strip()

        if choice == '1':
            run_auto_demo(model_path)
        elif choice == '2':
            run_circle_demo(model_path)
        elif choice == '3':
            run_square_demo(model_path)
        elif choice == '4':
            run_simple_demo(model_path)
        elif choice == '5':
            run_manual_control(model_path)
        else:
            print("无效选择，运行自动演示...")
            run_auto_demo(model_path)

    except KeyboardInterrupt:
        print("\n\n演示被用户停止")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
