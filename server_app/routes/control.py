# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.control
 模块职责: 单车实时速度遥控接口（调试页键盘 / 手柄）
--------------------------------------------------------------------------------
   POST /api/control_velocity   下发实时速度指令 [M,CARn,vx,vy,vz]（低延迟单发）

 安全拦截: 该车正在执行到点任务时拒绝遥控，避免两路指令互相覆盖。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, request, jsonify

from .. import state
from .. import goto

control_bp = Blueprint('control', __name__)


@control_bp.route('/api/control_velocity', methods=['POST'])
def control_car_velocity():
    """实时速度控制接口（用于键盘 / 手柄遥控）。"""
    data = request.get_json(silent=True) or {}
    car_id = data.get('car_id')
    if not car_id:
        return jsonify({'success': False, 'error': '缺少car_id'})
    if goto.is_active(car_id):
        return jsonify({'success': False, 'error': f'{car_id} 正在执行到点任务，已锁定遥控'})

    try:
        vx = float(data.get('vx', 0.0))
        vy = float(data.get('vy', 0.0))
        vz = float(data.get('vz', 0.0))
    except (TypeError, ValueError):
        return jsonify({'success': False, 'error': '速度参数无效'})

    cmd_str = f"[M,{car_id},{vx:.3f},{vy:.3f},{vz:.3f}]"
    success = state.udp_server.send_to_car(car_id, cmd_str, quiet=True)
    return jsonify({'success': success})
