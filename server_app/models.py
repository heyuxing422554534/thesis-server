# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.models
 模块职责: 纯数据模型定义（无业务逻辑、无外部依赖）
--------------------------------------------------------------------------------
   Car   单辆小车的运行时状态快照（位置/朝向/电量/速度/连接状态等）

 位置约定:
   raw_position  小车上报的 UWB 标签融合坐标（未做标签偏移补偿）
   position      换算后的车中心坐标（控制、显示、日志均以此为准）

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time


class Car:
    """单辆小车的运行时状态。"""

    def __init__(self, car_id, address):
        self.car_id = car_id
        self.mac_address = f"CAR_{car_id}"
        self.address = address
        self.raw_position = {"x": 0.0, "y": 0.0}
        self.position = {"x": 0.0, "y": 0.0}
        self.heading = 0.0
        self.battery = 100
        self.velocity = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.speed = 0.0
        self.connected = True
        self.last_update = time.time()
        self.status = "正常"
        self.update_count = 0
        self.last_broadcast_time = 0
        self.connection_attempts = 0
