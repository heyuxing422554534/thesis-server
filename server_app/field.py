# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.field
 模块职责: 场地配置加载、可运动区域判定与 UWB 标签偏移补偿
--------------------------------------------------------------------------------
 配置来源: 工程根目录 field_config.json（基站坐标、边界留白、车体尺寸、
 UWB 标签相对车中心的偏移）。基站移动后只需修改该文件并重启服务器。

   load                 读取配置文件（失败时使用内置默认值）
   get_config           返回当前配置字典（供前端绘制场地）
   work_area            返回可运动区域 (xmin, xmax, ymin, ymax)
   in_work_area         判断点是否在可运动区域内
   tag_offset           返回某车的标签偏移 (前, 左)
   tag_to_center        UWB 标签坐标 -> 车中心坐标

 坐标约定: A0 为原点，+x 指向 A3，+y 指向 A1；航向 0°=+x，逆时针为正。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import json
import math

from . import config

CONFIG_PATH = os.path.join(config.PROJECT_ROOT, "field_config.json")

_DEFAULT = {
    "anchors": {"A0": [0.0, 0.0], "A1": [0.0, 3.6], "A2": [3.0, 3.6], "A3": [3.0, 0.0]},
    "margin": 0.3,
    "car": {"length": 0.26, "width": 0.18},
    "tag_offset_default": [0.04, 0.03],
    "tag_offset_per_car": {},
}

_cfg = dict(_DEFAULT)


def load():
    """读取 field_config.json；文件缺失或格式错误时保留默认值并打印提示。"""
    global _cfg
    cfg = json.loads(json.dumps(_DEFAULT))
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            user_cfg = json.load(f)
        for key in _DEFAULT:
            if key in user_cfg:
                cfg[key] = user_cfg[key]
        print(f" 场地配置已加载: {CONFIG_PATH}")
    except FileNotFoundError:
        print(f" 未找到 {CONFIG_PATH}，使用默认场地配置")
    except Exception as e:
        print(f" 场地配置读取失败({e})，使用默认场地配置")
    _cfg = cfg
    xmin, xmax, ymin, ymax = work_area()
    print(f" 可运动区域: x[{xmin:.2f}, {xmax:.2f}] y[{ymin:.2f}, {ymax:.2f}]")
    return _cfg


def get_config():
    """返回当前配置（附带计算出的可运动区域）。"""
    xmin, xmax, ymin, ymax = work_area()
    out = json.loads(json.dumps(_cfg))
    out["work_area"] = {"xmin": xmin, "xmax": xmax, "ymin": ymin, "ymax": ymax}
    return out


def _anchor_bounds():
    xs = [p[0] for p in _cfg["anchors"].values()]
    ys = [p[1] for p in _cfg["anchors"].values()]
    return min(xs), max(xs), min(ys), max(ys)


def work_area():
    """基站包围框向内收缩 margin 后的可运动区域。"""
    xmin, xmax, ymin, ymax = _anchor_bounds()
    m = float(_cfg.get("margin", 0.0))
    return xmin + m, xmax - m, ymin + m, ymax - m


def in_work_area(x, y, tol=0.0):
    """点是否在可运动区域内（tol>0 时允许略微超出）。"""
    xmin, xmax, ymin, ymax = work_area()
    return (xmin - tol) <= x <= (xmax + tol) and (ymin - tol) <= y <= (ymax + tol)


def tag_offset(car_id):
    """返回该车 UWB 标签相对车中心的偏移 (前, 左)，单位 m。"""
    per_car = _cfg.get("tag_offset_per_car") or {}
    off = per_car.get(car_id, _cfg.get("tag_offset_default", [0.0, 0.0]))
    return float(off[0]), float(off[1])


def tag_to_center(car_id, x_tag, y_tag, yaw_deg):
    """把 UWB 标签坐标换算为车中心坐标: p_center = p_tag - R(yaw)·offset。"""
    fwd, left = tag_offset(car_id)
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return x_tag - (fwd * c - left * s), y_tag - (fwd * s + left * c)
