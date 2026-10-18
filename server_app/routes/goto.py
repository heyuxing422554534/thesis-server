# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.goto
 模块职责: 到点实验、场地配置与全局急停接口
--------------------------------------------------------------------------------
   GET  /api/field                 场地配置（基站、留白、可运动区域、车体尺寸、标签偏移）
   POST /api/goto/start            启动单点 / 多点任务
   POST /api/goto/stop             停止某车任务
   POST /api/estop                 全部急停
   GET  /api/goto/status           全部任务状态（含到达记录与统计）
   POST /api/goto/measure          填写某次到达的卷尺实测坐标（或跳过）
   GET  /api/goto/params           查询控制参数
   POST /api/goto/params           修改控制参数
   GET  /api/goto/logs/<filename>  下载 logs/goto 下的 CSV

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os

from flask import Blueprint, request, jsonify, send_from_directory, abort

from .. import config
from .. import field
from .. import goto

goto_bp = Blueprint('goto', __name__)


def _num(value, name):
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} 不是有效数字")


@goto_bp.route('/api/field')
def get_field():
    return jsonify(field.get_config())


@goto_bp.route('/api/goto/start', methods=['POST'])
def start_goto():
    data = request.get_json(silent=True) or {}
    car_id = data.get('car_id')
    try:
        points = [(_num(p.get('x'), 'x'), _num(p.get('y'), 'y')) for p in data.get('points', [])]
        yaw = _num(data.get('yaw', 0.0), '航向')
        repeats = int(data.get('repeats', 1))
        dwell = data.get('dwell_time')
        dwell = None if dwell in (None, '') else _num(dwell, '停留时间')
    except (ValueError, AttributeError) as e:
        return jsonify({'success': False, 'error': str(e)})
    ok, msg = goto.start_task(car_id, points, yaw, repeats, dwell)
    return jsonify({'success': ok, 'message' if ok else 'error': msg})


@goto_bp.route('/api/goto/stop', methods=['POST'])
def stop_goto():
    data = request.get_json(silent=True) or {}
    car_id = data.get('car_id')
    if not car_id:
        return jsonify({'success': False, 'error': '缺少car_id'})
    goto.stop_task(car_id)
    return jsonify({'success': True, 'message': f'{car_id} 已停止'})


@goto_bp.route('/api/estop', methods=['POST'])
def emergency_stop():
    car_ids = goto.stop_all("全部急停")
    return jsonify({'success': True, 'message': f'已急停 {len(car_ids)} 辆在线小车', 'cars': car_ids})


@goto_bp.route('/api/goto/status')
def goto_status():
    return jsonify(goto.get_status())


@goto_bp.route('/api/goto/measure', methods=['POST'])
def goto_measure():
    data = request.get_json(silent=True) or {}
    car_id = data.get('car_id')
    try:
        index = int(data.get('index'))
        skip = bool(data.get('skip', False))
        mx = my = None
        if not skip:
            mx = _num(data.get('x'), '实测 x')
            my = _num(data.get('y'), '实测 y')
    except (TypeError, ValueError) as e:
        return jsonify({'success': False, 'error': str(e)})
    ok, result = goto.set_measurement(car_id, index, mx, my, skip)
    if not ok:
        return jsonify({'success': False, 'error': result})
    return jsonify({'success': True, 'record': result})


@goto_bp.route('/api/goto/params', methods=['GET', 'POST'])
def goto_params():
    if request.method == 'GET':
        return jsonify({'params': config.GOTO_PARAMS, 'limits': config.GOTO_PARAM_LIMITS})
    ok, result = goto.update_params(request.get_json(silent=True) or {})
    if not ok:
        return jsonify({'success': False, 'error': result})
    return jsonify({'success': True, 'params': result})


@goto_bp.route('/api/goto/logs/<path:filename>')
def goto_log_file(filename):
    log_dir = os.path.join(config.PROJECT_ROOT, config.GOTO_LOG_DIRNAME)
    if os.path.basename(filename) != filename or not filename.endswith('.csv'):
        abort(404)
    return send_from_directory(log_dir, filename, as_attachment=True)
