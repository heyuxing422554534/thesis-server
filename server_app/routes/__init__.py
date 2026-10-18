# -*- coding: utf-8 -*-
"""
================================================================================
 子包名称: server_app.routes
 子包职责: 按业务域拆分的 Flask 蓝图集合
--------------------------------------------------------------------------------
   page_bp        Web 首页（/）
   cars_bp        小车列表查询（/api/cars）
   broadcast_bp   广播开关与参数（/api/broadcast*）
   control_bp     单车实时速度遥控（/api/control_velocity）
   topology_bp    通信拓扑设置与查询（/api/topology*）
   goto_bp        场地配置、到点实验、全局急停（/api/field、/api/goto/*、/api/estop）

 register_all_blueprints(app) 由 app 模块调用，集中完成全部蓝图注册。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from .page import page_bp
from .cars import cars_bp
from .broadcast import broadcast_bp
from .control import control_bp
from .topology import topology_bp
from .goto import goto_bp


def register_all_blueprints(app):
    """把本子包内的全部蓝图注册到给定的 Flask 应用（不加 url_prefix）。"""
    for bp in (page_bp, cars_bp, broadcast_bp, control_bp, topology_bp, goto_bp):
        app.register_blueprint(bp)
