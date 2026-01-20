#!/usr/bin/env python3
from evdev import InputDevice, ecodes

dev = InputDevice('/dev/input/event1')
print(f'设备名称: {dev.name}')
print(f'设备路径: {dev.path}')

# 显示所有支持的能力
caps = dev.capabilities()
print('\n支持的类型:', [ecodes.EV[t] for t in caps.keys()])

# 显示绝对轴详情
if ecodes.EV_ABS in caps:
    print('\n支持的轴:')
    for item in caps[ecodes.EV_ABS]:
        if isinstance(item, tuple):
            code, info = item
            print(f'  {ecodes.ABS[code]}: {info}')

print('\n请摇动摇杆或按键，Ctrl+C退出...\n')

for event in dev.read_loop():
    if event.type == ecodes.EV_ABS:
        abs_name = ecodes.ABS.get(event.code, f'Unknown({event.code})')
        print(f'轴: {abs_name:20s} 值: {event.value:5d}')
    elif event.type == ecodes.EV_KEY:
        key_name = ecodes.KEY.get(event.code, f'Unknown({event.code})')
        state = '按下' if event.value == 1 else '释放'
        print(f'按键: {key_name:30s} {state}')
