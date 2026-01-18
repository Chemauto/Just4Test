"""
虚拟手柄读取示例
演示如何读取虚拟手柄设备
"""
import sys
import os

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time


def read_with_pygame():
    """使用pygame读取虚拟手柄"""
    try:
        import pygame

        print("=" * 50)
        print("使用pygame读取虚拟手柄")
        print("=" * 50)
        print()

        # 初始化pygame
        pygame.init()
        pygame.joystick.init()

        # 检测手柄数量
        joystick_count = pygame.joystick.get_count()
        print(f"检测到 {joystick_count} 个手柄设备")

        if joystick_count == 0:
            print("\n未检测到手柄！")
            print("请确保：")
            print("1. 虚拟手柄模拟器正在运行")
            print("2. uinput已正确配置")
            print("3. /dev/input/js0 设备存在")
            return

        # 连接第一个手柄
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print(f"\n已连接: {joystick.get_name()}")
        print(f"按键数: {joystick.get_numbuttons()}")
        print(f"轴数: {joystick.get_numaxes()}")
        print(f"方向键数: {joystick.get_numhats()}")
        print("\n按Ctrl+C退出...")
        print("-" * 50)

        # 持续读取
        try:
            while True:
                # 处理pygame事件
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        print(f"[按键] 按钮 {event.button} 按下")
                    elif event.type == pygame.JOYBUTTONUP:
                        print(f"[按键] 按钮 {event.button} 释放")
                    elif event.type == pygame.JOYAXISMOTION:
                        if abs(event.value) > 0.1:  # 忽略微小移动
                            print(f"[轴] 轴 {event.axis} 值: {event.value:.3f}")
                    elif event.type == pygame.JOYHATMOTION:
                        print(f"[方向键] 值: {event.value}")

                # 也可以直接查询状态
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\n退出读取程序")

    except ImportError:
        print("pygame未安装！")
        print("安装命令: pip install pygame")


def read_from_api():
    """使用VirtualJoystick API"""
    try:
        from src.virtual_joystick import VirtualJoystick

        print("=" * 50)
        print("使用VirtualJoystick API读取")
        print("=" * 50)
        print("\n注意：这只是演示API使用")
        print("实际的虚拟手柄需要在GUI中操作")
        print()

        # 创建虚拟手柄对象
        joystick = VirtualJoystick(0)

        print(f"手柄名称: {joystick.get_name()}")
        print(f"按键数: {joystick.get_numbuttons()}")
        print(f"轴数: {joystick.get_numaxes()}")
        print(f"方向键数: {joystick.get_numhats()}")

        print("\n按Ctrl+C退出...")
        print("-" * 50)

        # 持续读取
        last_state = {}

        try:
            while True:
                # 读取按键
                buttons_changed = False
                for i in range(joystick.get_numbuttons()):
                    state = joystick.get_button(i)
                    if last_state.get(f'btn_{i}') != state:
                        if state:
                            print(f"[按键] 按钮 {i} 按下")
                        last_state[f'btn_{i}'] = state
                        buttons_changed = True

                # 读取轴
                for i in range(joystick.get_numaxes()):
                    value = joystick.get_axis(i)
                    if abs(value) > 0.1:  # 忽略微小移动
                        key = f'axis_{i}'
                        if last_state.get(key) != value:
                            print(f"[轴] 轴 {i} 值: {value:.3f}")
                            last_state[key] = value

                # 读取方向键
                hat_x, hat_y = joystick.get_hat(0)
                if last_state.get('hat') != (hat_x, hat_y):
                    if hat_x != 0 or hat_y != 0:
                        print(f"[方向键] X={hat_x}, Y={hat_y}")
                    last_state['hat'] = (hat_x, hat_y)

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\n退出读取程序")

    except Exception as e:
        print(f"错误: {e}")


def read_device_directly():
    """直接读取设备节点（高级）"""
    try:
        print("=" * 50)
        print("直接读取设备节点 /dev/input/js0")
        print("=" * 50)
        print()

        device_path = "/dev/input/js0"

        if not os.path.exists(device_path):
            print(f"设备 {device_path} 不存在！")
            print("请确保虚拟手柄模拟器正在运行")
            return

        # 打开设备
        import struct
        fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)

        # js_event结构体格式
        # struct js_event {
        #     uint32_t time;     // 事件时间戳
        #     int16_t value;     // 事件值
        #     uint8_t type;      // 事件类型
        #     uint8_t number;    // 轴/按键编号
        # };
        event_format = 'IhBB'
        event_size = struct.calcsize(event_format)

        print(f"正在监听 {device_path}...")
        print("按Ctrl+C退出...")
        print("-" * 50)

        try:
            while True:
                try:
                    # 读取事件
                    data = os.read(fd, event_size)
                    if len(data) == event_size:
                        timestamp, value, type_, number = struct.unpack(event_format, data)

                        # 解析事件类型
                        if type_ & 0x01:  # JS_EVENT_BUTTON
                            action = "按下" if value else "释放"
                            print(f"[按键] 按钮 {number} {action}")
                        elif type_ & 0x02:  # JS_EVENT_AXIS
                            if abs(value) > 100:  # 忽略微小移动
                                axis_value = value / 32767.0  # 归一化到[-1, 1]
                                print(f"[轴] 轴 {number} 值: {axis_value:.3f}")

                except BlockingIOError:
                    # 没有数据可读
                    time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n\n退出读取程序")
        finally:
            os.close(fd)

    except PermissionError:
        print("权限不足！")
        print("请使用: sudo " + " ".join(sys.argv))
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    print("\n虚拟手柄读取示例程序")
    print("=" * 50)
    print()
    print("请选择读取方式：")
    print("1. 使用pygame（推荐）")
    print("2. 使用VirtualJoystick API")
    print("3. 直接读取设备节点（高级）")
    print()

    choice = input("请输入选项 (1-3，直接回车使用默认1): ").strip()

    if choice == "2":
        read_from_api()
    elif choice == "3":
        read_device_directly()
    else:
        read_with_pygame()
