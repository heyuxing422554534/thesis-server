# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.app
 模块职责: Flask 应用装配与程序主入口
--------------------------------------------------------------------------------
   _configure_utf8_console  统一 Windows 控制台与标准输出为 UTF-8
   create_app               创建 Flask 应用、启用 CORS、注册全部蓝图
   get_local_ip             获取本机对外 IP（用于打印访问地址）
   get_network_info         打印本机网络接口信息
   main                     程序主入口：加载场地配置 -> 启动 UDP -> 启动到点控制 -> 运行 Flask

 装配要点:
   在此创建 UDPServer 单例并赋值给 state.udp_server，供其它模块统一访问。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import sys
import socket

# 兼容“直接运行本文件”（例如在 IDE 中对 app.py 按运行/调试）的场景。
# 推荐的启动方式仍是根目录的 web_car_server.py 或 `python -m server_app`。
if __package__ in (None, ""):
    _PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _PROJECT_ROOT not in sys.path:
        sys.path.insert(0, _PROJECT_ROOT)
    __package__ = "server_app"

from flask import Flask
from flask_cors import CORS

from . import config
from . import state
from . import net
from . import field
from . import goto
from . import topology as topology_ops
from .routes import register_all_blueprints


def _configure_utf8_console():
    """尽量让 Windows 控制台和 Python 标准输出统一使用 UTF-8。"""
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if hasattr(sys.stderr, "reconfigure"):
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    if os.name == "nt":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleOutputCP(65001)
            kernel32.SetConsoleCP(65001)
        except Exception:
            pass


def create_app():
    """创建并配置 Flask 应用：启用 CORS，注册全部蓝图。"""
    template_dir = os.path.join(config.PROJECT_ROOT, "templates")
    app = Flask(__name__, template_folder=template_dir)
    CORS(app)
    register_all_blueprints(app)
    return app


def get_local_ip():
    """获取本机对外 IP 地址（失败返回提示串）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "无法获取IP"


def get_network_info():
    """打印本机网络接口信息（需要 netifaces，缺失则忽略）。"""
    try:
        import netifaces
        interfaces = netifaces.interfaces()
        print(" 网络接口信息:")
        for interface in interfaces:
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    print(f"  {interface}: {addr_info['addr']} - 广播地址: {addr_info.get('broadcast', 'N/A')}")
    except ImportError:
        print(" 无法获取网络接口信息（未安装 netifaces）")
    except Exception as e:
        print(f" 获取网络接口信息失败: {e}")


def main():
    """程序主入口：加载场地配置、启动 UDP 服务器与到点控制线程、运行 Flask。"""
    _configure_utf8_console()

    get_network_info()
    field.load()

    # 创建 UDP 服务器单例并注入共享状态，供全体模块访问
    state.udp_server = net.UDPServer(config.UDP_HOST, config.UDP_PORT)
    topology_ops.update_topology_cache()

    app = create_app()

    if state.udp_server.start():
        print(" UDP服务器启动成功")
        goto.start_controller()
        print(f" 广播频率: {1 / state.broadcast_interval:.0f}Hz ({state.broadcast_interval * 1000:.0f}ms间隔)")
        print(f" 广播分组大小: 每组最多 {state.broadcast_group_size} 辆小车")
        print(f" 使用子网广播地址，端口: {config.BROADCAST_PORT}")
        local_ip = get_local_ip()
        print(f" 服务器本地IP地址: {local_ip}")
        print(f" 访问 http://{local_ip}:{config.WEB_PORT} 打开控制界面")
        app.run(host='0.0.0.0', port=config.WEB_PORT, debug=False, use_reloader=False, threaded=True)
    else:
        print(" UDP服务器启动失败，无法运行应用")


if __name__ == '__main__':
    main()
