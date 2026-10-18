# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.topology
 模块职责: 通信拓扑矩阵的构建、缓存与下发
--------------------------------------------------------------------------------
   update_topology_cache      根据当前矩阵刷新“每辆车可见哪些车”的缓存
   copy_topology              深拷贝矩阵（避免共享引用）
   flatten_topology           把 NxN 矩阵拉平为逗号分隔字符串（用于下发指令）
   apply_topology             应用新矩阵并向小车广播启用/矩阵/禁用指令
   burst_broadcast_command    连发多次广播指令，对抗 UDP 丢包

 依赖关系:
   通过 state.udp_server 单例访问广播能力，避免与 net 层形成循环导入。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time

from . import config
from . import state


def burst_broadcast_command(command, repeat=3, delay=0.05):
    """连发多次全局广播指令，提升 UDP 丢包环境下的送达率。"""
    for i in range(repeat):
        state.udp_server.broadcast_global_command(command)
        if i < repeat - 1:
            time.sleep(delay)


def update_topology_cache():
    """根据当前通信拓扑矩阵刷新“每辆车能收到哪些车状态”的缓存。"""
    state.topology_cache = {}
    car_ids = config.ALL_CAR_IDS
    car_mapping = config.CAR_INDEX_MAP
    for target_car in car_ids:
        visible_cars = []
        target_index = car_mapping[target_car]
        for other_car in car_ids:
            if other_car != target_car:
                other_index = car_mapping[other_car]
                if state.communication_topology[other_index][target_index] == 1:
                    visible_cars.append(other_car)
        state.topology_cache[target_car] = visible_cars


def copy_topology(matrix):
    """返回矩阵的逐行深拷贝。"""
    return [row[:] for row in matrix]


def flatten_topology(matrix):
    """把二维矩阵拉平为逗号分隔的字符串（用于构造 [T,M,...] 指令）。"""
    return ','.join(str(cell) for row in matrix for cell in row)


def is_valid_matrix(matrix):
    """检查是否为 MAX_CARS x MAX_CARS 的 0/1 矩阵。"""
    n = config.MAX_CARS
    return (isinstance(matrix, list) and len(matrix) == n and
            all(isinstance(row, list) and len(row) == n and
                all(cell in (0, 1) for cell in row) for row in matrix))


def apply_topology(matrix, enable):
    """应用新的拓扑矩阵，并向小车广播启用/矩阵/禁用指令。"""
    state.communication_topology = copy_topology(matrix)
    update_topology_cache()

    if enable and not state.topology_enabled:
        burst_broadcast_command("[T,E,1]")

    topology_cmd = f"[T,M,{flatten_topology(state.communication_topology)}]"
    burst_broadcast_command(topology_cmd)

    if not enable and state.topology_enabled:
        burst_broadcast_command("[T,E,0]")

    state.topology_enabled = enable
