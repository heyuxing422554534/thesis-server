# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.cars
 模块职责: 小车列表查询接口
--------------------------------------------------------------------------------
 提供 GET /api/cars，返回当前所有已知小车的状态快照。
 position 为车中心坐标（已做 UWB 标签偏移补偿），raw_position 为上报原值。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time

from flask import Blueprint, jsonify

from .. import config
from .. import state

cars_bp = Blueprint('cars', __name__)


@cars_bp.route('/api/cars')
def get_cars():
    """返回所有已知小车的状态快照列表（按车号排序）。"""
    now = time.time()
    with state.car_lock:
        car_list = [{
            'id': car_id,
            'position': car.position,
            'raw_position': car.raw_position,
            'heading': car.heading,
            'battery': car.battery,
            'velocity': car.velocity,
            'speed': car.speed,
            'connected': car.connected,
            'status': car.status,
            'last_update': car.last_update,
            'age': now - car.last_update,
            'update_count': car.update_count,
        } for car_id, car in state.cars.items()]
    car_list.sort(key=lambda c: config.CAR_INDEX_MAP.get(c['id'], 99))
    return jsonify(car_list)
