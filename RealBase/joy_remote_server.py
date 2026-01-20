#!/usr/bin/env python3
"""
开发板端服务器程序
接收来自电脑的手柄指令并控制底盘
"""

import sys
import time
import argparse
import socket
import pickle

from motor_controller import OmniWheelController


class RemoteServer:
    """远程控制服务器"""

    def __init__(self, host, port, serial_port):
        """初始化服务器

        Args:
            host: 监听地址 (0.0.0.0表示所有接口)
            port: 监听端口
            serial_port: 串口端口
        """
        self.host = host
        self.port = port
        self.serial_port = serial_port
        self.server_sock = None
        self.client_sock = None
        self.controller = None
        self.running = False

    def start(self):
        """启动服务器"""
        # 初始化底盘控制器
        print("初始化底盘控制器...")
        self.controller = OmniWheelController(port=self.serial_port)

        print(f"\n正在连接底盘...")
        if not self.controller.connect():
            print("\n连接失败,请检查:")
            print(f"  - 串口端口 '{self.serial_port}' 是否正确")
            print("  - 电机是否已上电并连接")
            print(f"  - 电机ID是否正确({self.controller.WHEEL_IDS})")
            return False
        print("✓ 底盘连接成功!")

        # 创建socket
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(1)

        print(f"\n✓ 服务器已启动，监听 {self.host}:{self.port}")
        print("等待电脑连接...")
        print(f"  请在电脑上运行: python joy_remote_client.py --host {self.get_local_ip()}")

        self.running = True

        try:
            while self.running:
                # 等待客户端连接
                try:
                    self.client_sock, addr = self.server_sock.accept()
                    print(f"\n✓ 客户端已连接: {addr[0]}:{addr[1]}")
                    print("开始接收控制指令...\n")

                    # 处理客户端请求
                    self.handle_client()

                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"\n✗ 连接错误: {e}")
                    continue

        except KeyboardInterrupt:
            print("\n\n服务器被中断")

        finally:
            self.stop()

        return True

    def handle_client(self):
        """处理客户端连接"""
        self.client_sock.settimeout(None)  # 永不超时

        try:
            while self.running:
                # 接收数据
                data = self.client_sock.recv(1024)
                if not data:
                    print("\n客户端已断开连接")
                    break

                # 解析指令
                try:
                    command = pickle.loads(data)
                    if command['type'] == 'velocity':
                        vx = command['vx']
                        vy = command['vy']
                        omega = command['omega']

                        # 发送速度指令
                        if vx == 0.0 and vy == 0.0 and omega == 0.0:
                            self.controller.stop()
                        else:
                            self.controller.set_velocity_raw(vx=vx, vy=vy, omega=omega)

                except Exception as e:
                    print(f"\n✗ 解析指令失败: {e}")

        except ConnectionResetError:
            print("\n客户端强制断开连接")
        except Exception as e:
            print(f"\n✗ 处理客户端时出错: {e}")

        finally:
            # 停止机器人
            self.controller.stop()
            if self.client_sock:
                try:
                    self.client_sock.close()
                except:
                    pass
                self.client_sock = None
            print("等待重新连接...")

    def stop(self):
        """停止服务器"""
        print("\n正在停止服务器...")
        self.running = False

        # 停止机器人
        if self.controller:
            self.controller.stop()
            self.controller.disconnect()

        # 关闭socket
        if self.client_sock:
            try:
                self.client_sock.close()
            except:
                pass
            self.client_sock = None

        if self.server_sock:
            try:
                self.server_sock.close()
            except:
                pass
            self.server_sock = None

        print("服务器已停止")

    @staticmethod
    def get_local_ip():
        """获取本地IP地址"""
        try:
            # 创建一个UDP socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 连接到一个外部地址（不会实际发送数据）
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "192.168.0.155"  # 返回默认IP


def main(port, serial_port):
    """主函数"""
    print("=" * 60)
    print("Xbox手柄远程控制服务器")
    print("=" * 60)
    print(f"串口: {serial_port}")
    print(f"监听端口: {port}")
    print("=" * 60)

    server = RemoteServer(host='0.0.0.0', port=port, serial_port=serial_port)

    try:
        server.start()
    except Exception as e:
        print(f"\n服务器错误: {e}")
        import traceback
        traceback.print_exc()
        server.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="开发板端远程控制服务器")
    parser.add_argument(
        '--port',
        type=int,
        default=9999,
        help='监听端口'
    )
    parser.add_argument(
        '--serial',
        type=str,
        default='/dev/ttyACM0',
        help='电机控制器连接的串口端口'
    )
    args = parser.parse_args()
    main(args.port, args.serial)
