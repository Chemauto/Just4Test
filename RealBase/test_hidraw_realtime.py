#!/usr/bin/env python3
import sys
import time

print("实时监控hidraw0数据...")
print("请摇动摇杆或按键，观察数据变化\n")
print("格式: 字节索引(值变化的高亮)")
print("="*80)

with open('/dev/hidraw0', 'rb') as f:
    last_data = None
    count = 0

    try:
        while True:
            data = f.read(32)
            hex_str = ' '.join(f'{b:02x}' for b in data)

            # 标记变化的字节
            if last_data is not None:
                marked = []
                for i, (curr, last) in enumerate(zip(data, last_data)):
                    if curr != last:
                        marked.append(f'\033[1;31m{curr:02x}\033[0m')  # 红色高亮
                    else:
                        marked.append(f'{curr:02x}')
                hex_str = ' '.join(marked)

            # 只显示前20字节（通常摇杆数据在前20字节）
            display_data = ' '.join(f'{b:02x}' for b in data[:20])
            print(f"\r[{count:5d}] {display_data:<60}", end='', flush=True)

            last_data = data
            count += 1
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n停止监控")
