# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.net
 模块职责: UDP 遥测/指令服务器、子网广播服务器及网络辅助函数
--------------------------------------------------------------------------------
   get_subnet_broadcast     自动探测本机子网广播地址
   _normalize_car_address   规范化小车回包地址（固定发往小车监听端口）
   BroadcastServer          子网广播发送器（可靠重发）
   UDPServer                UDP 主服务器：接收遥测、周期广播小车状态、
                            连接健康检查、离线清理

 遥测格式（小车 -> 服务器，10 Hz）:
   CARn:x,y,yaw,voltage,vx,vy,vz
   x,y 为 UWB 标签融合坐标；接收时经 field.tag_to_center 换算为车中心坐标。

 依赖关系: config、state、models（Car）、field（标签偏移补偿）。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time
import socket
import threading

from . import config
from . import state
from . import field
from .models import Car

DEFAULT_BROADCAST_ADDRESS = "192.168.31.255"


def get_subnet_broadcast():
    """自动探测本机可用的子网广播地址；失败时回退到默认地址。"""
    try:
        import netifaces
        for interface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    ip = addr_info['addr']
                    if ip.startswith('127.') or ip.startswith('169.254.'):
                        continue
                    if 'broadcast' in addr_info:
                        broadcast_addr = addr_info['broadcast']
                        print(f" 发现广播地址: {broadcast_addr} (接口: {interface})")
                        return broadcast_addr
                    netmask = addr_info.get('netmask', '255.255.255.0')
                    ip_parts = list(map(int, ip.split('.')))
                    mask_parts = list(map(int, netmask.split('.')))
                    broadcast_parts = [str(ip_parts[i] | (255 - mask_parts[i])) for i in range(4)]
                    broadcast_addr = '.'.join(broadcast_parts)
                    print(f" 计算广播地址: {broadcast_addr} (接口: {interface})")
                    return broadcast_addr
        print(f" 无法自动获取广播地址，使用默认: {DEFAULT_BROADCAST_ADDRESS}")
        return DEFAULT_BROADCAST_ADDRESS
    except ImportError:
        print(" 未安装netifaces库，使用默认广播地址")
        return DEFAULT_BROADCAST_ADDRESS
    except Exception as e:
        print(f" 获取广播地址失败: {e}，使用默认广播地址")
        return DEFAULT_BROADCAST_ADDRESS


def _normalize_car_address(addr):
    """使用最近上报源IP，回包固定发往小车监听端口。"""
    if not addr or len(addr) < 2:
        return addr
    return (addr[0], config.CAR_COMMAND_PORT)


class BroadcastServer:
    """子网广播发送器：负责向广播地址发送指令/状态，支持可靠重发。"""

    def __init__(self, port=config.BROADCAST_PORT):
        self.port = port
        self.socket = None
        self.running = False
        self.broadcast_address = DEFAULT_BROADCAST_ADDRESS

    def start(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('', self.port))
            self.running = True
            print(f"广播服务器启动成功，绑定端口 {self.port}")
            print(f"使用子网广播地址: {self.broadcast_address}")
            return True
        except Exception as e:
            print(f"广播服务器启动失败: {e}")
            return False

    def broadcast_data(self, data):
        try:
            target = (self.broadcast_address, self.port)
            self.socket.sendto(data.encode('utf-8'), target)
            return True
        except Exception as e:
            print(f"广播发送失败: {e}")
            return False

    def broadcast_command_reliable(self, command, retries=5, delay=0.04):
        success_count = 0
        for i in range(retries):
            if self.broadcast_data(command):
                success_count += 1
                if i < retries - 1:
                    time.sleep(delay)
        print(f"广播指令 '{command}' 发送 {success_count}/{retries} 次")
        return success_count > 0

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()


class UDPServer:
    """UDP 主服务器：遥测接收、状态广播、健康检查与离线清理。"""

    def __init__(self, host=config.UDP_HOST, port=config.UDP_PORT):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.send_lock = threading.Lock()
        self.broadcast_server = BroadcastServer(config.BROADCAST_PORT)

    def start(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
            self.socket.bind((self.host, self.port))
            self.running = True
            print(f"UDP服务器启动在 {self.host}:{self.port}")
            print("等待小车连接...")

            for target in (self._receive_loop, self._broadcast_loop,
                           self._cleanup_loop, self._health_check_loop):
                threading.Thread(target=target, daemon=True).start()

            if not self.broadcast_server.start():
                print(" 广播服务器启动失败，但UDP服务器继续运行")

            return True
        except Exception as e:
            print(f" UDP服务器启动失败: {e}")
            return False

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(4096)
                if data:
                    self._handle_car_data(data, addr)
            except BlockingIOError:
                time.sleep(0.001)
            except OSError:
                if not self.running:
                    break
                time.sleep(0.01)
            except Exception as e:
                print(f" UDP接收错误: {e}")
                time.sleep(0.01)

    def _handle_car_data(self, data, addr):
        try:
            text = data.decode('utf-8', errors='ignore').strip()
            if not text:
                return

            # 遥测格式: CARn:x,y,yaw,voltage,vx,vy,vz
            parts = text.split(':')
            if len(parts) != 2:
                return
            car_id = parts[0]
            if car_id not in config.CAR_INDEX_MAP:
                return
            values = parts[1].split(',')
            if len(values) < 7:
                return

            x_raw = float(values[0])
            y_raw = float(values[1])
            yaw = float(values[2])
            voltage = float(values[3])
            vx = float(values[4])
            vy = float(values[5])
            vz = float(values[6])
            x, y = field.tag_to_center(car_id, x_raw, y_raw, yaw)

            current_time = time.time()
            reconnect_event = False
            normalized_addr = _normalize_car_address(addr)

            with state.car_lock:
                car = state.cars.get(car_id)
                if car is None:
                    car = Car(car_id, normalized_addr)
                    state.cars[car_id] = car
                    print(f" 新小车连接: {car_id} from {normalized_addr}")
                    reconnect_event = True
                else:
                    if car.address != normalized_addr:
                        print(f" 小车 {car_id} 地址变化: {car.address} -> {normalized_addr}")
                        car.address = normalized_addr
                        reconnect_event = True
                    if not car.connected:
                        print(f" 小车 {car_id} 重新连接: {normalized_addr}")
                        car.connected = True
                        car.connection_attempts = 0
                        reconnect_event = True

                car.raw_position = {"x": x_raw, "y": y_raw}
                car.position = {"x": x, "y": y}
                car.heading = yaw
                car.battery = voltage
                car.velocity = {"vx": vx, "vy": vy, "vz": vz}
                car.speed = (vx ** 2 + vy ** 2) ** 0.5
                car.last_update = current_time
                car.update_count += 1

            if reconnect_event:
                self._send_reconnect_ack(car_id)
                threading.Thread(target=self._broadcast_all_cars_data, daemon=True).start()

        except ValueError:
            return
        except Exception as e:
            print(f" 处理小车数据失败: {e}")

    def _send_reconnect_ack(self, car_id):
        ack_msg = f"RECONNECT_ACK:{car_id},SERVER_READY"
        try:
            with state.car_lock:
                car = state.cars.get(car_id)
                address = car.address if car and car.connected else None
            if address:
                with self.send_lock:
                    self.socket.sendto(ack_msg.encode('utf-8'), address)
                print(f" 向 {car_id} 发送重连确认")
        except Exception as e:
            print(f" 发送重连确认失败: {e}")

    def _broadcast_loop(self):
        last_broadcast = 0
        while self.running:
            try:
                current_time = time.time()
                if state.broadcast_enabled and (current_time - last_broadcast >= state.broadcast_interval):
                    self._broadcast_all_cars_data()
                    last_broadcast = current_time
                sleep_time = max(0.001, state.broadcast_interval - (time.time() - last_broadcast))
                time.sleep(sleep_time)
            except Exception as e:
                print(f" 广播循环错误: {e}")
                time.sleep(0.01)

    def _split_cars_into_groups(self, car_list):
        groups = []
        car_ids = sorted(car_list.keys(), key=lambda cid: config.CAR_INDEX_MAP.get(cid, 99))
        for i in range(0, len(car_ids), state.broadcast_group_size):
            group_car_ids = car_ids[i:i + state.broadcast_group_size]
            groups.append({car_id: car_list[car_id] for car_id in group_car_ids})
        return groups

    def _broadcast_all_cars_data(self):
        """按 [N C1 x y yaw vx vy vz ...] 格式广播在线小车状态（车中心坐标）。"""
        current_time = time.time()
        with state.car_lock:
            connected_cars = {
                car_id: car for car_id, car in state.cars.items()
                if car.connected and current_time - car.last_update < 3.0
            }
            if not connected_cars:
                return False

            cars_to_broadcast = {}
            if state.topology_enabled:
                for car_id, car in connected_cars.items():
                    car_index = config.CAR_INDEX_MAP[car_id]
                    if sum(state.communication_topology[car_index]) > 0:
                        cars_to_broadcast[car_id] = car
            else:
                cars_to_broadcast = connected_cars

            messages = []
            for group_cars in self._split_cars_into_groups(cars_to_broadcast):
                broadcast_parts = [f"[{len(group_cars)}"]
                for car_id, car in group_cars.items():
                    short_id = f"C{car_id[3:]}"
                    broadcast_parts.append(
                        f"{short_id} {car.position['x']:.2f} {car.position['y']:.2f} "
                        f"{car.heading:.1f} {car.velocity['vx']:.4f} "
                        f"{car.velocity['vy']:.4f} {car.velocity['vz']:.4f}")
                    car.last_broadcast_time = current_time
                messages.append(" ".join(broadcast_parts) + "]")

        all_success = True
        for msg in messages:
            if not self.broadcast_server.broadcast_data(msg):
                all_success = False
        return all_success

    def _get_visible_cars_for_car(self, target_car_id):
        if not state.topology_enabled:
            return [cid for cid in config.ALL_CAR_IDS if cid != target_car_id]
        return state.topology_cache.get(target_car_id, [])

    def _health_check_loop(self):
        while self.running:
            try:
                current_time = time.time()
                disconnected_cars = []
                with state.car_lock:
                    for car_id, car in state.cars.items():
                        if current_time - car.last_update > 5.0 and car.connected:
                            disconnected_cars.append(car_id)
                            car.connected = False
                for car_id in disconnected_cars:
                    print(f" 小车 {car_id} 超时未更新，标记为断开")
                time.sleep(2.0)
            except Exception as e:
                print(f" 健康检查错误: {e}")
                time.sleep(1.0)

    def _cleanup_loop(self):
        while self.running:
            try:
                current_time = time.time()
                with state.car_lock:
                    cleanup_cars = [
                        car_id for car_id, car in state.cars.items()
                        if not car.connected and current_time - car.last_update > 60.0
                    ]
                    for car_id in cleanup_cars:
                        del state.cars[car_id]
                for car_id in cleanup_cars:
                    print(f" 清理长时间离线小车: {car_id}")
                time.sleep(10.0)
            except Exception as e:
                print(f" 清理循环错误: {e}")
                time.sleep(1.0)

    def send_to_car(self, car_id, message, quiet=False):
        """向单车单播一条指令；quiet=True 时不打印（用于 10 Hz 控制指令）。"""
        with state.car_lock:
            car = state.cars.get(car_id)
            if car is None:
                if not quiet:
                    print(f" 小车 {car_id} 不存在")
                return False
            if not car.connected:
                if not quiet:
                    print(f" 小车 {car_id} 已断开连接")
                return False
            address = car.address
        if not message.endswith('\n'):
            message += '\n'
        try:
            with self.send_lock:
                self.socket.sendto(message.encode('utf-8'), address)
            if not quiet:
                print(f" 向 {car_id} 发送: {message.strip()}")
            return True
        except Exception as e:
            print(f" 向 {car_id} 发送失败: {e}")
            return False

    def send_to_car_reliable(self, car_id, message, max_retries=4):
        for _ in range(max_retries):
            if self.send_to_car(car_id, message):
                return True
            time.sleep(0.05)
        return False

    def broadcast_global_command(self, command):
        return self.broadcast_server.broadcast_command_reliable(command, retries=5, delay=0.01)

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()
        self.broadcast_server.stop()
