# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.goto
 模块职责: 单车到点控制（单点 / 多点测试）、到达判定、实测记录与日志
--------------------------------------------------------------------------------
 执行链路（每车一个任务，统一控制线程按 GOTO_PARAMS["rate_hz"] 运行）:
   车中心坐标(net 已做标签偏移补偿) -> 全局速度 -> 按航向转到车体系
   -> [T,CARn,vx,vy,yaw] 下发（ESP32 透传 vx/vy 并闭环保持航向）

 控制律（e = 目标 - 当前, d = |e|）:
   d >  approach_dist         行进: |v| = min(vmax, kp·d)
   tol_in <= d <= approach    逼近: |v| = clamp(kp·d, vmin, vmax)
   d <  tol_in                停车: v = 0，并开始计时
   连续 settle_time 秒满足 d<tol_in、车速<settle_speed、航向误差<yaw_tol -> 到达
   到达后保持 v=0；只有 d > tol_out 时才重新校正（进出容差迟滞）。
   全局速度矢量的变化率限制在 accel_max 以内。

 安全:
   - 状态数据年龄 > state_timeout: 立即发 0 速度并暂停；持续 2 s 仍无数据则终止任务。
   - 车中心超出可运动区域(含 5 cm 容差): 停车并终止任务。
   - 任务结束 / 急停 / 服务器退出: 连发 3 次 [M,CARn,0,0,0]。

 日志（logs/goto/，每个任务两个 CSV）:
   *_trace.csv    每个控制周期一行
   *_summary.csv  每次到达(或超时)一行，含实测坐标与误差；有新记录时整体重写

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import csv
import math
import time
import atexit
import threading
from datetime import datetime

from . import config
from . import state
from . import field

# 状态数据持续丢失超过该时长则终止任务（秒）
STATE_LOST_ABORT_S = 2.0
# 运行中越界判定的容差（米），避免 UWB 噪声在边界附近误触发
BOUNDARY_TOL = 0.05
# 每个任务在内存中保留的到达记录上限
MAX_RECORDS = 500

PHASE_TEXT = {
    "travel": "行进",
    "approach": "末端逼近",
    "settling": "稳定判定",
    "arrived": "已到达",
    "dwell": "停留",
    "hold": "数据超时暂停",
    "done": "完成",
    "stopped": "已停止",
    "error": "错误",
}

_lock = threading.RLock()
_tasks = {}            # car_id -> GotoTask（含已结束任务，便于前端查看记录）
_thread = None
_thread_stop = threading.Event()


def _wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def _log_dir():
    path = os.path.join(config.PROJECT_ROOT, config.GOTO_LOG_DIRNAME)
    os.makedirs(path, exist_ok=True)
    return path


def send_stop(car_id, repeat=3):
    """连发 0 速度手动指令，使 ESP32 进入手动模式并输出 [0,0,0]。"""
    if not state.udp_server:
        return False
    ok = False
    for i in range(repeat):
        ok = state.udp_server.send_to_car(car_id, f"[M,{car_id},0.000,0.000,0.000]", quiet=True) or ok
        if i < repeat - 1:
            time.sleep(0.02)
    return ok


class GotoTask:
    """单车到点任务：按顺序到达 points，重复 repeats 轮。"""

    def __init__(self, car_id, points, target_yaw, repeats, dwell_time):
        self.car_id = car_id
        self.points = points
        self.target_yaw = target_yaw
        self.repeats = repeats
        self.dwell_time = dwell_time
        self.multi = len(points) > 1 or repeats > 1

        self.active = True
        self.phase = "travel"
        self.message = ""
        self.point_idx = 0
        self.round_idx = 0
        self.start_time = time.time()
        self.point_start = self.start_time
        self.settle_start = None
        self.dwell_start = None
        self.hold_since = None
        self.phase_before_hold = "travel"
        self.arrived_once = False

        self.cmd_global = (0.0, 0.0)
        self.cmd_body = (0.0, 0.0)
        self.last_dist = None
        self.last_tick = None

        self.records = []

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(_log_dir(), f"goto_{car_id}_{stamp}")
        self.trace_path = base + "_trace.csv"
        self.summary_path = base + "_summary.csv"
        self._trace_file = open(self.trace_path, "w", newline="", encoding="utf-8-sig")
        self._trace = csv.writer(self._trace_file)
        self._trace.writerow([
            "time", "t_rel_s", "car_id", "round", "point_idx",
            "raw_x", "raw_y", "x", "y", "yaw_deg",
            "target_x", "target_y", "target_yaw_deg", "dist_m", "phase",
            "cmd_vx_global", "cmd_vy_global", "cmd_vx_body", "cmd_vy_body", "state_age_s",
        ])

    # ------------------------------------------------------------------
    @property
    def target(self):
        return self.points[self.point_idx]

    def to_dict(self):
        tx, ty = self.target
        return {
            "car_id": self.car_id,
            "active": self.active,
            "phase": self.phase,
            "phase_text": PHASE_TEXT.get(self.phase, self.phase),
            "message": self.message,
            "multi": self.multi,
            "target": {"x": tx, "y": ty, "yaw": self.target_yaw},
            "points": [{"x": p[0], "y": p[1]} for p in self.points],
            "point_idx": self.point_idx,
            "point_count": len(self.points),
            "round_idx": self.round_idx,
            "repeats": self.repeats,
            "dist": self.last_dist,
            "cmd_body": {"vx": self.cmd_body[0], "vy": self.cmd_body[1]},
            "cmd_global": {"vx": self.cmd_global[0], "vy": self.cmd_global[1]},
            "elapsed": time.time() - self.start_time,
            "point_elapsed": time.time() - self.point_start,
            "records": self.records,
            "stats": self.stats(),
            "trace_file": os.path.basename(self.trace_path),
            "summary_file": os.path.basename(self.summary_path),
        }

    def stats(self):
        est = [r["est_err"] for r in self.records if not r["timed_out"]]
        meas = [r["meas_err"] for r in self.records if r.get("meas_err") is not None]
        timeouts = sum(1 for r in self.records if r["timed_out"])

        def _summ(vals):
            if not vals:
                return {"count": 0, "mean": None, "max": None}
            return {"count": len(vals), "mean": sum(vals) / len(vals), "max": max(vals)}

        return {"arrivals": len(est), "timeouts": timeouts, "est": _summ(est), "meas": _summ(meas)}

    # ------------------------------------------------------------------
    def _write_trace(self, now, car, dist, age):
        tx, ty = self.target
        raw = car.raw_position if car else {"x": "", "y": ""}
        pos = car.position if car else {"x": "", "y": ""}
        self._trace.writerow([
            datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
            f"{now - self.start_time:.3f}", self.car_id, self.round_idx + 1, self.point_idx + 1,
            _fmt(raw["x"]), _fmt(raw["y"]), _fmt(pos["x"]), _fmt(pos["y"]),
            _fmt(car.heading if car else ""),
            f"{tx:.4f}", f"{ty:.4f}", f"{self.target_yaw:.1f}",
            _fmt(dist), self.phase,
            f"{self.cmd_global[0]:.4f}", f"{self.cmd_global[1]:.4f}",
            f"{self.cmd_body[0]:.4f}", f"{self.cmd_body[1]:.4f}",
            _fmt(age, 3),
        ])

    def write_summary(self):
        try:
            with open(self.summary_path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow([
                    "index", "time", "car_id", "round", "point_idx", "target_x", "target_y",
                    "est_x", "est_y", "est_err_m", "raw_x", "raw_y", "yaw_deg",
                    "meas_x", "meas_y", "meas_err_m", "elapsed_s", "timed_out",
                ])
                for r in self.records:
                    w.writerow([
                        r["index"], r["time"], self.car_id, r["round"], r["point_idx"],
                        f"{r['target_x']:.4f}", f"{r['target_y']:.4f}",
                        f"{r['est_x']:.4f}", f"{r['est_y']:.4f}", f"{r['est_err']:.4f}",
                        f"{r['raw_x']:.4f}", f"{r['raw_y']:.4f}", f"{r['yaw']:.1f}",
                        _fmt(r.get("meas_x")), _fmt(r.get("meas_y")), _fmt(r.get("meas_err")),
                        f"{r['elapsed']:.2f}", int(r["timed_out"]),
                    ])
        except Exception as e:
            print(f" 写到点汇总日志失败: {e}")

    def _add_record(self, now, car, timed_out):
        tx, ty = self.target
        x, y = car.position["x"], car.position["y"]
        self.records.append({
            "index": len(self.records) + 1,
            "time": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
            "round": self.round_idx + 1,
            "point_idx": self.point_idx + 1,
            "target_x": tx, "target_y": ty,
            "est_x": x, "est_y": y,
            "est_err": math.hypot(tx - x, ty - y),
            "raw_x": car.raw_position["x"], "raw_y": car.raw_position["y"],
            "yaw": car.heading,
            "elapsed": now - self.point_start,
            "timed_out": timed_out,
            "meas_x": None, "meas_y": None, "meas_err": None, "skipped": False,
        })
        if len(self.records) > MAX_RECORDS:
            self.records.pop(0)
        self.write_summary()

    def close(self):
        try:
            self._trace_file.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    def finish(self, phase, message=""):
        self.active = False
        self.phase = phase
        self.message = message
        self.cmd_global = (0.0, 0.0)
        self.cmd_body = (0.0, 0.0)
        send_stop(self.car_id)
        self.write_summary()
        self.close()
        print(f" 到点任务结束 {self.car_id}: {PHASE_TEXT.get(phase, phase)} {message}")

    def _next_point(self, now):
        self.point_idx += 1
        if self.point_idx >= len(self.points):
            self.point_idx = 0
            self.round_idx += 1
        if self.round_idx >= self.repeats:
            self.point_idx = len(self.points) - 1
            self.round_idx = self.repeats - 1
            self.finish("done", "全部点位完成")
            return
        self.phase = "travel"
        self.point_start = now
        self.settle_start = None
        self.dwell_start = None
        self.arrived_once = False

    def _velocity(self, ex, ey, dist, p):
        """返回期望全局速度（未做加速度限制）。"""
        if dist < p["tol_in"]:
            return 0.0, 0.0
        speed = p["kp"] * dist
        if dist <= p["approach_dist"]:
            speed = max(p["vmin"], speed)
        speed = min(p["vmax"], speed)
        return speed * ex / dist, speed * ey / dist

    def tick(self, now, car, p):
        """执行一个控制周期；返回 (要下发的指令或 None)。"""
        dt = (now - self.last_tick) if self.last_tick else 1.0 / p["rate_hz"]
        self.last_tick = now

        age = (now - car.last_update) if car else None
        if car is None or not car.connected or age > p["state_timeout"]:
            if self.hold_since is None:
                self.hold_since = now
                self.phase_before_hold = self.phase
                send_stop(self.car_id, repeat=1)
            self.phase = "hold"
            self.message = "状态数据超时，已停车等待"
            self.cmd_global = (0.0, 0.0)
            self.cmd_body = (0.0, 0.0)
            self._write_trace(now, car, self.last_dist, age)
            if now - self.hold_since > STATE_LOST_ABORT_S:
                self.finish("error", f"状态数据丢失超过 {STATE_LOST_ABORT_S:.0f} s")
            return None

        if self.hold_since is not None:
            self.hold_since = None
            self.phase = self.phase_before_hold
            self.message = ""

        x, y = car.position["x"], car.position["y"]
        if not field.in_work_area(x, y, BOUNDARY_TOL):
            self.last_dist = None
            self._write_trace(now, car, None, age)
            self.finish("error", f"车中心 ({x:.2f}, {y:.2f}) 超出可运动区域")
            return None

        tx, ty = self.target
        ex, ey = tx - x, ty - y
        dist = math.hypot(ex, ey)
        self.last_dist = dist
        yaw_err = abs(_wrap_deg(self.target_yaw - car.heading))

        # ---- 单点超时（仅多点测试） ----
        if self.multi and not self.arrived_once and now - self.point_start > p["point_timeout"]:
            self._add_record(now, car, timed_out=True)
            self._write_trace(now, car, dist, age)
            self._next_point(now)
            return self._command(0.0, 0.0, car) if self.active else None

        # ---- 阶段与期望速度 ----
        if self.phase in ("arrived", "dwell"):
            if dist > p["tol_out"]:
                self.phase = "approach"
                self.settle_start = None
                self.message = "偏离目标，重新校正"
            vgx, vgy = (0.0, 0.0) if self.phase in ("arrived", "dwell") else \
                self._velocity(ex, ey, dist, p)
        else:
            vgx, vgy = self._velocity(ex, ey, dist, p)
            if dist < p["tol_in"]:
                stable = car.speed < p["settle_speed"] and yaw_err < p["yaw_tol"]
                if yaw_err >= p["yaw_tol"]:
                    self.message = f"等待航向对齐（误差 {yaw_err:.1f}°）"
                elif car.speed >= p["settle_speed"]:
                    self.message = "等待车辆停稳"
                else:
                    self.message = ""
                if self.phase != "settling":
                    self.phase = "settling"
                    self.settle_start = now if stable else None
                elif not stable:
                    self.settle_start = None
                elif self.settle_start is None:
                    self.settle_start = now
                if self.settle_start is not None and now - self.settle_start >= p["settle_time"]:
                    self.message = ""
                    if not self.arrived_once:
                        self.arrived_once = True
                        self._add_record(now, car, timed_out=False)
                    if self.multi:
                        # 停留计时从首次到达开始；中途被推开再回来不重新计时
                        self.phase = "dwell"
                        if self.dwell_start is None:
                            self.dwell_start = now
                    else:
                        self.phase = "arrived"
            else:
                self.settle_start = None
                self.phase = "approach" if dist <= p["approach_dist"] else "travel"

        # ---- 多点停留结束 -> 下一点 ----
        if self.phase == "dwell" and now - self.dwell_start >= self.dwell_time:
            self._write_trace(now, car, dist, age)
            self._next_point(now)
            return self._command(0.0, 0.0, car) if self.active else None

        if dist < p["tol_in"] or self.phase in ("arrived", "dwell"):
            # 进入容差或已到达: 直接置零（此时指令速度已不超过 vmin，无需减速斜坡）
            vgx, vgy = 0.0, 0.0
        else:
            # ---- 加速度限制 ----
            max_dv = p["accel_max"] * max(dt, 1e-3)
            dvx, dvy = vgx - self.cmd_global[0], vgy - self.cmd_global[1]
            dv = math.hypot(dvx, dvy)
            if dv > max_dv:
                vgx = self.cmd_global[0] + dvx * max_dv / dv
                vgy = self.cmd_global[1] + dvy * max_dv / dv

        cmd = self._command(vgx, vgy, car)
        self._write_trace(now, car, dist, age)
        return cmd

    def _command(self, vgx, vgy, car):
        """全局速度 -> 车体速度，生成 [T] 指令。"""
        yaw = math.radians(car.heading)
        c, s = math.cos(yaw), math.sin(yaw)
        vbx = c * vgx + s * vgy
        vby = -s * vgx + c * vgy
        self.cmd_global = (vgx, vgy)
        self.cmd_body = (vbx, vby)
        return f"[T,{self.car_id},{vbx:.3f},{vby:.3f},{self.target_yaw:.1f}]"


def _fmt(v, nd=4):
    if v is None or v == "":
        return ""
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return ""


# ==============================================================================
# 控制线程
# ==============================================================================
def _loop():
    next_t = time.monotonic()
    while not _thread_stop.is_set():
        p = dict(config.GOTO_PARAMS)
        period = 1.0 / p["rate_hz"]
        now = time.time()
        commands = []
        with _lock:
            active = [t for t in _tasks.values() if t.active]
        for task in active:
            with state.car_lock:
                car = state.cars.get(task.car_id)
                snapshot = _snapshot(car) if car else None
            with _lock:
                if not task.active:
                    continue
                try:
                    cmd = task.tick(now, snapshot, p)
                except Exception as e:
                    print(f" 到点控制异常 {task.car_id}: {e}")
                    task.finish("error", f"控制异常: {e}")
                    cmd = None
            if cmd:
                commands.append((task.car_id, cmd))
        for car_id, cmd in commands:
            state.udp_server.send_to_car(car_id, cmd, quiet=True)

        next_t += period
        delay = next_t - time.monotonic()
        if delay < 0:
            next_t = time.monotonic()
            delay = 0
        _thread_stop.wait(delay)


class _CarSnapshot:
    __slots__ = ("raw_position", "position", "heading", "speed", "last_update", "connected")


def _snapshot(car):
    s = _CarSnapshot()
    s.raw_position = dict(car.raw_position)
    s.position = dict(car.position)
    s.heading = car.heading
    s.speed = car.speed
    s.last_update = car.last_update
    s.connected = car.connected
    return s


def start_controller():
    """启动到点控制线程（幂等）。"""
    global _thread
    if _thread and _thread.is_alive():
        return
    _thread_stop.clear()
    _thread = threading.Thread(target=_loop, daemon=True, name="goto-controller")
    _thread.start()
    print(f" 到点控制线程已启动 ({config.GOTO_PARAMS['rate_hz']:.0f} Hz)")


# ==============================================================================
# 对外接口
# ==============================================================================
def start_task(car_id, points, target_yaw=0.0, repeats=1, dwell_time=None):
    """启动到点任务；返回 (ok, message)。points 为 [(x, y), ...]。"""
    if car_id not in config.CAR_INDEX_MAP:
        return False, f"未知车辆 {car_id}"
    if not points:
        return False, "缺少目标点"
    for i, (x, y) in enumerate(points):
        if not field.in_work_area(x, y):
            xmin, xmax, ymin, ymax = field.work_area()
            return False, (f"第 {i + 1} 个点 ({x:.2f}, {y:.2f}) 不在可运动区域 "
                           f"x[{xmin:.2f},{xmax:.2f}] y[{ymin:.2f},{ymax:.2f}] 内")
    if repeats < 1:
        return False, "重复次数至少为 1"

    with state.car_lock:
        car = state.cars.get(car_id)
        if car is None or not car.connected:
            return False, f"小车 {car_id} 未连接"
        age = time.time() - car.last_update
        cx, cy = car.position["x"], car.position["y"]
    if age > config.GOTO_PARAMS["state_timeout"]:
        return False, f"小车 {car_id} 状态数据过期（{age:.1f} s 未更新）"
    if not field.in_work_area(cx, cy, BOUNDARY_TOL):
        return False, f"小车当前位置 ({cx:.2f}, {cy:.2f}) 不在可运动区域内，请先把车放进场地"

    dwell = config.GOTO_PARAMS["dwell_time"] if dwell_time is None else dwell_time
    with _lock:
        old = _tasks.get(car_id)
        if old and old.active:
            old.finish("stopped", "被新任务替换")
        task = GotoTask(car_id, [(float(x), float(y)) for x, y in points],
                        float(target_yaw), int(repeats), float(dwell))
        _tasks[car_id] = task
    start_controller()
    kind = "多点测试" if task.multi else "单点到点"
    print(f" 到点任务启动 {car_id}: {kind}, {len(points)} 个点 x {repeats} 轮, 航向 {target_yaw:.1f}°")
    return True, f"{car_id} {kind}已启动"


def stop_task(car_id):
    with _lock:
        task = _tasks.get(car_id)
        if task and task.active:
            task.finish("stopped", "手动停止")
            return True
    send_stop(car_id)
    return False


def stop_all(reason="全部急停"):
    """终止全部任务并向所有在线车辆发送停车指令。"""
    with _lock:
        for task in _tasks.values():
            if task.active:
                task.finish("stopped", reason)
    with state.car_lock:
        car_ids = [cid for cid, car in state.cars.items() if car.connected]
    for car_id in car_ids:
        send_stop(car_id)
    return car_ids


def is_active(car_id):
    with _lock:
        task = _tasks.get(car_id)
        return bool(task and task.active)


def get_status():
    with _lock:
        return {cid: t.to_dict() for cid, t in _tasks.items()}


def set_measurement(car_id, index, meas_x=None, meas_y=None, skip=False):
    """为第 index 次到达记录填入卷尺实测的车中心坐标（或标记跳过）。"""
    with _lock:
        task = _tasks.get(car_id)
        if not task:
            return False, f"{car_id} 没有到点任务记录"
        rec = next((r for r in task.records if r["index"] == index), None)
        if rec is None:
            return False, f"找不到第 {index} 条记录"
        if skip:
            rec["skipped"] = True
        else:
            rec["meas_x"] = float(meas_x)
            rec["meas_y"] = float(meas_y)
            rec["meas_err"] = math.hypot(rec["target_x"] - rec["meas_x"], rec["target_y"] - rec["meas_y"])
            rec["skipped"] = False
        task.write_summary()
        return True, rec


def update_params(new_params):
    """校验并更新到点控制参数；返回 (ok, 当前参数或错误信息)。"""
    staged = dict(config.GOTO_PARAMS)
    for key, value in (new_params or {}).items():
        if key not in config.GOTO_PARAMS:
            return False, f"未知参数 {key}"
        try:
            value = float(value)
        except (TypeError, ValueError):
            return False, f"参数 {key} 不是数字"
        lo, hi = config.GOTO_PARAM_LIMITS[key]
        if not lo <= value <= hi:
            return False, f"参数 {key} 超出范围 [{lo}, {hi}]"
        staged[key] = value
    if staged["tol_out"] <= staged["tol_in"]:
        return False, "退出容差必须大于进入容差"
    if staged["vmin"] > staged["vmax"]:
        return False, "最低速度不能大于最大速度"
    config.GOTO_PARAMS.update(staged)
    return True, dict(config.GOTO_PARAMS)


@atexit.register
def _shutdown():
    """服务器退出时停止所有到点任务（连发 0 速度）。"""
    _thread_stop.set()
    try:
        if state.udp_server and state.udp_server.socket:
            stop_all("服务器退出")
    except Exception:
        pass
