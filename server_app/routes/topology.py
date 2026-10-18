# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.topology
 模块职责: 通信拓扑设置与查询接口
--------------------------------------------------------------------------------
   POST /api/topology                设置 NxN 拓扑矩阵并启用/禁用，随后广播下发
   GET  /api/topology/status         查询当前拓扑矩阵与启用状态
   POST /api/topology/toggle         仅启用/禁用拓扑过滤（不改矩阵）
   GET  /api/topology/visible/<id>   查询某辆车当前能收到哪些车的状态

 注意: 小车端 ESP32 固件需支持 8 车矩阵（[T,M,...] 共 64 个值）。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, request, jsonify

from .. import config
from .. import state
from .. import topology as topology_ops

topology_bp = Blueprint('topology', __name__)


@topology_bp.route('/api/topology', methods=['POST'])
def set_topology():
    """设置拓扑矩阵并启用/禁用，随后广播下发给小车。"""
    data = request.get_json(silent=True) or {}
    matrix = data.get('topology')
    enable = bool(data.get('enable', False))
    if not topology_ops.is_valid_matrix(matrix):
        n = config.MAX_CARS
        return jsonify({'success': False, 'error': f'拓扑矩阵必须是 {n}x{n} 的 0/1 矩阵'})
    topology_ops.apply_topology(matrix, enable)
    print(f" 通信拓扑已更新, 启用={enable}")
    return jsonify({
        'success': True,
        'message': f'拓扑已更新并{"启用" if enable else "禁用"}',
        'topology': state.communication_topology,
        'topology_enabled': state.topology_enabled,
    })


@topology_bp.route('/api/topology/status')
def get_topology_status():
    return jsonify({
        'topology': state.communication_topology,
        'topology_enabled': state.topology_enabled,
        'car_ids': config.ALL_CAR_IDS,
    })


@topology_bp.route('/api/topology/toggle', methods=['POST'])
def toggle_topology():
    data = request.get_json(silent=True) or {}
    enable = bool(data.get('enable', False))
    state.topology_enabled = enable
    topology_ops.update_topology_cache()
    broadcast_success = state.udp_server.broadcast_global_command(f"[T,E,{1 if enable else 0}]")
    status = "启用" if enable else "禁用"
    print(f" 拓扑通信 {status}")
    return jsonify({
        'success': True,
        'message': f'拓扑通信已{status}',
        'topology_enabled': state.topology_enabled,
        'broadcast_success': broadcast_success,
    })


@topology_bp.route('/api/topology/visible/<car_id>')
def get_visible_cars(car_id):
    return jsonify({
        'car_id': car_id,
        'visible_cars': state.udp_server._get_visible_cars_for_car(car_id),
        'topology_enabled': state.topology_enabled,
    })
