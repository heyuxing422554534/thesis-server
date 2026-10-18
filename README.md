# 多车协同服务器端（到点实验版）

服务器负责接收小车遥测、计算到点控制速度并下发，同时提供 Flask Web 控制台。
当前阶段目标：单车到点（实测误差 ≤ 5 cm），为后续 8 车 B→J→T→U 图案变换打基础。

王浩版本中的视觉对接（reconstruct / vision / guide）、领航跟随编队（formation_controller）
和矩形轨迹实验已删除，可在 git 历史中找回。

## 启动

```
.venv-local\Scripts\python.exe web_car_server.py
```

浏览器打开控制台输出的 `http://<本机IP>:5000`。依赖见 `requirements.txt`
（视觉功能已删，opencv / numpy 不再必需）。

## 架构

```
STM32 (UWB+编码器+IMU, ESKF) --UART--> ESP32 --UDP 8080--> 服务器
                                                           |  车中心坐标 = UWB 坐标 - R(ψ)·标签偏移
                                                           |  到点控制 10 Hz: 全局速度 -> 车体速度
STM32 轮速闭环 <--UART-- ESP32 (透传 vx,vy + 航向闭环) <--UDP 8081-- [T,CARn,vx,vy,yaw]
```

| 模块 | 职责 |
|---|---|
| `server_app/config.py` | 端口、车辆编号（8 车）、到点控制参数 `GOTO_PARAMS` |
| `server_app/state.py` | 共享状态：小车表、广播开关、通信拓扑 |
| `server_app/field.py` | 读取 `field_config.json`，可运动区域判定，UWB 标签偏移补偿 |
| `server_app/net.py` | UDP 遥测接收、状态广播、离线检测、单播/广播发送 |
| `server_app/goto.py` | 到点控制器、多点测试、到达判定、实测记录、CSV 日志、安全保护 |
| `server_app/topology.py` | 8×8 通信拓扑的缓存与下发 |
| `server_app/routes/` | HTTP 接口（见下表） |
| `templates/index.html` | Web 控制台 |

## 场地配置 `field_config.json`

坐标系：A0 为原点（左下），+x 指向 A3，+y 指向 A1；航向 0° = +x，逆时针为正。

| 字段 | 含义 |
|---|---|
| `anchors` | 四个基站坐标 (m) |
| `margin` | 可运动区域相对基站边框的内缩量 (m) |
| `car` | 车体长、宽 (m)，用于绘图 |
| `tag_offset_default` | UWB 标签相对车中心的偏移，车体系 (前, 左) (m) |
| `tag_offset_per_car` | 个别车辆的标签偏移，例如 `"CAR3": [0.05, 0.02]` |

基站移动后：先在 UWB 配置软件中修改基站坐标，再改此文件并重启服务器。

## 到点控制

e = 目标 − 车中心，d = |e|，每 0.1 s 计算一次：

| 阶段 | 条件 | 速度大小 |
|---|---|---|
| 行进 | d > approach_dist (0.20 m) | min(vmax, kp·d) |
| 末端逼近 | tol_in ≤ d ≤ approach_dist | clamp(kp·d, vmin, vmax) |
| 稳定判定 | d < tol_in (0.03 m) | 0，连续 settle_time (1 s) 满足"停稳且航向误差 < yaw_tol"即到达 |
| 已到达 | — | 0；只有 d > tol_out (0.06 m) 才重新校正 |

速度矢量变化率限制在 accel_max 以内。所有参数可在网页"控制参数"卡片中实时修改。

安全：
- 状态超过 `state_timeout` (0.3 s) 未更新：停车等待，持续 2 s 则终止任务。
- 车中心超出可运动区域（容差 5 cm）：停车并终止任务；目标点越界直接拒绝。
- 停止 / 急停 / 服务器退出：连发 3 次 `[M,CARn,0,0,0]`。ESP32 自身 500 ms 收不到 `[T]` 也会停车。
- 本阶段没有车间避碰，多车同时到点需人工注意。

## 日志 `logs/goto/`

每个任务两个 CSV（UTF-8 BOM，可直接用 Excel 打开）：
- `goto_CARn_<时间>_trace.csv`：每个控制周期一行（原始 UWB、车中心、目标、距离、阶段、下发速度、状态年龄）。
- `goto_CARn_<时间>_summary.csv`：每次到达一行（估计误差、卷尺实测坐标与误差、用时、是否超时）。

## HTTP 接口

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/cars` | 小车列表（`position` 为车中心，`raw_position` 为上报原值） |
| GET | `/api/field` | 场地配置与可运动区域 |
| POST | `/api/goto/start` | `{car_id, points:[{x,y}], yaw, repeats, dwell_time}` |
| POST | `/api/goto/stop` | `{car_id}` |
| POST | `/api/estop` | 全部急停 |
| GET | `/api/goto/status` | 各车任务状态、到达记录与统计 |
| POST | `/api/goto/measure` | `{car_id, index, x, y}` 或 `{car_id, index, skip:true}` |
| GET/POST | `/api/goto/params` | 查询 / 修改控制参数 |
| GET | `/api/goto/logs/<文件名>` | 下载日志 CSV |
| POST | `/api/control_velocity` | 调试遥控 `[M,CARn,vx,vy,vz]`（到点任务中被锁定） |
| POST | `/api/broadcast` | 开关状态广播 |
| POST | `/api/topology`、`/api/topology/toggle` | 设置 / 启停 8×8 通信拓扑 |

所有接口均无鉴权，只应在实验局域网内使用。

## UDP 协议

上行（小车 → 服务器 8080）：`CARn:x,y,yaw,voltage,vx,vy,vz`

下行（服务器 → 小车 8081）：

| 指令 | 含义 |
|---|---|
| `[T,CARn,vx,vy,yaw]` | 车体系速度 (m/s) + 目标航向 (°)，到点控制使用 |
| `[M,CARn,vx,vy,vz]` | 手动速度，停车时发全 0 |
| `[T,E,0/1]`、`[T,M,64 个值]` | 拓扑启停 / 8×8 矩阵 |
| `[N C1 x y yaw vx vy vz ...]` | 状态广播（车中心坐标） |
