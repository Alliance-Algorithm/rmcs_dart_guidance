# 飞镖控制重构：第一阶段框架方案

> 状态：按本文档实现框架（接口 + stub 状态机），**不做真实控制律**  
> 前置：旧 actions/tasks 已清空；`DartManager` 已从全量 IO 瘦身为骨架  
> 关联：`rmcs_core/src/controller/dart/docs/refactor.md`

---

## 1. 目标与非目标

### 1.1 第一阶段目标

1. `rmcs_core` 三机构控制器可 plugin 加载，消费命名命令、发布 `MechanismStatus`
2. `rmcs_dart_guidance` 通过 Resource 写命令、等 status，用现有 Task/Action 运行时推进
3. stub：命令 → `BUSY` → 固定 tick → `SUCCEEDED` → Action 成功 → Task 推进
4. guidance **不再**直接读写电机 angle/velocity/torque

### 1.2 非目标

- 真实堵转/行程/闭环与 YAML 整定
- 视觉、manual、force
- 旧 `/dart_manager/*` 兼容层
- `carriage_init` 完整标定（可预留 `SLIDE_STALL_CALIB`）

---

## 2. 架构

```
[RemoteCommandBridge] ──string──► /dart/manager/command
                                       │
                                       ▼
                             [DartManager]
                        任务队列 + Action 运行时
                                       │
            ┌──────────────────────────┼──────────────────────────┐
            ▼                          ▼                          ▼
      BeltResource              TriggerResource            FillingResource
       cmd + status             cmd+setpoint+status         cmd + status
            │                          │                          │
            ▼                          ▼                          ▼
    /dart/belt/*               /dart/trigger/*             /dart/filling/*
            │                          │                          │
            ▼                          ▼                          ▼
    BeltController            TriggerController          FillingController
```

| 层 | 职责 |
|----|------|
| guidance Task | 业务顺序、fire_count 分支 |
| guidance Action | 写命名命令，等 status |
| guidance Resource | command/status 接口引用包装 |
| core Controller | 命名动作 FSM + 电机（二阶段填逻辑） |

---

## 3. 接口契约

### 3.1 Topic

| 方向 | Topic | 类型 |
|------|--------|------|
| G→C | `/dart/belt/command` | `BeltCommand` |
| C→G | `/dart/belt/status` | `MechanismStatus` |
| G→C | `/dart/trigger/command` | `TriggerCommand` |
| G→C | `/dart/trigger/setpoint` | `double`（仅 `SLIDE_GOTO`，否则 NaN） |
| C→G | `/dart/trigger/status` | `MechanismStatus` |
| G→C | `/dart/filling/command` | `FillingCommand` |
| C→G | `/dart/filling/status` | `MechanismStatus` |
| 外部→G | `/dart/manager/command` | `string` |
| G debug | `/dart/manager/debug/*`、`fire_count` | 精简保留 |

### 3.2 枚举（`rmcs_dart_guidance` C++ 头，非 rosidl）

路径：`include/rmcs_dart_guidance/msg/`

```
mechanism_status.hpp
belt_command.hpp
trigger_command.hpp
filling_command.hpp
```

命名空间：`rmcs_dart_guidance::msg`

```cpp
enum class MechanismStatus : uint8_t {
    IDLE = 0, BUSY = 1, SUCCEEDED = 2, FAILED = 3, ABORTED = 4,
};

enum class BeltCommand : uint8_t {
    IDLE = 0, ABORT,
    DOWN_SLOW, DOWN_FAST, DOWN_HOLD,
    UP_SOFT, UP_HARD, UP_STALL, BRAKE,
};

enum class TriggerCommand : uint8_t {
    IDLE = 0, ABORT,
    SERVO_LOCK, SERVO_FREE,
    SLIDE_UP, SLIDE_DOWN, SLIDE_STALL_CALIB, SLIDE_GOTO,
};

enum class FillingCommand : uint8_t {
    IDLE = 0, ABORT,
    LIFT_UP, LIFT_DOWN,
    LIMIT_FREE, LIMIT_LOCK, LIMIT_PULSE_FILL,
};
```

- CMake export include，供 `rmcs_core` 使用  
- **`rmcs_core` depend `rmcs_dart_guidance`**（层级倒置，第一阶段接受）

### 3.3 命令语义

- **电平保持**：直到 Action 结束写 `IDLE`
- **抢占**：BUSY 中不同非 IDLE 命令 → 重开
- **IDLE → status 立即 IDLE**
- **SUCCEEDED/FAILED/ABORTED**：保持到 guidance 写 `IDLE` 或新有效命令
- **边沿**：`cmd 变化` 或 `从终态/IDLE 进入有效命令`
- **cancel / 任务失败**：写 **ABORT 并保持**，直到外部 `recover` 再 idle（正常成功只 idle，不发 ABORT）
- **机构 Action 超时**：Task 注册时逐步写死 `timeout_ticks`（须大于 core `stub_complete_ticks`）。超时 → `TIMEOUT` → 与失败相同（ABORT 保持 + ERROR）。`update` 判定：failed → succeeded → timeout

### 3.4 Core FSM（写死）

```text
if cmd == ABORT:
    stop; status = ABORTED; return
if cmd == IDLE:
    stop; status = IDLE; return
if 边沿(有效命令):
    active_cmd = cmd; tick = 0; status = BUSY
if BUSY:
    tick++; run_busy_stub()  # 第一阶段空
    if tick >= stub_complete_ticks: status = SUCCEEDED
*status_out = status
```

参数：`stub_complete_ticks`（默认 50）。

---

## 4. Core

```
rmcs_core/src/controller/dart/
  mechanism_fsm.hpp
  belt_controller.cpp
  trigger_controller.cpp
  filling_controller.cpp
```

- `rmcs_core::controller::dart::{Belt,Trigger,Filling}Controller`
- 注册 `plugins.xml`
- Trigger：舵机 + 滑台合并

---

## 5. Guidance

```
manager/
  core/
    dart_manager.cpp
    runtime/          # action 运行时 + manager_types
    action/           # 具体动作
    task/             # 具体任务 + task_factory
  resources/          # belt/trigger/filling resource（机构 IO）
  external_control/
```

### 5.0 Resource IO 注册（对齐 hardware/device）

机构 command/status **不在** `DartManager` 上 register，由 Resource 构造时注册：

```text
DartManager (主 Component = status 侧)
  └─ create_partner_component<CommandComponent>  // 嵌套私有类，对齐 InfantryCommand
  └─ BeltResource(*this, *command_, "/dart/belt")
  └─ TriggerResource(*this, *command_, "/dart/trigger")
  └─ FillingResource(*this, *command_, "/dart/filling")

BeltResource(status_component, command_component, prefix):
  command_component.register_output(prefix+"/command", ...)
  status_component.register_input (prefix+"/status", ...)
  // trigger 额外: command 侧 setpoint
```

与 `DjiMotor(status, command, prefix)` / `OmniInfantry::InfantryCommand` 同构：  
partner 为 **主类内嵌** `CommandComponent`，`update` 可空（命令在主组件 task 循环中写入）。  
guidance 为控制使用方，故 command 侧为 **output**、status 侧为 **input**（电机 device 极性相反）。

`DartManager` 仅保留：`/dart/manager/command`、debug、fire_count、任务队列。

### 5.1 Task 序列

**dart-init**

```text
all-success(INIT, LIFT_UP, TRIGGER_FREE)
```

**dart-launch-prepare**

```text
fire_count == 0:
DOWN_SLOW → TRIGGER_LOCK → UP_SOFT

fire_count > 0:
DOWN_FAST → LIFT_DOWN → UP_SOFT_PART → LIFT_UP
→ DOWN_SLOW_PART → TRIGGER_LOCK
→ all-success(UP_SOFT, LIMIT_PULSE_FILL)
```

**dart-launch-cancel**

```text
DOWN_FAST → TRIGGER_FREE → UP_HARD
```

**dart-fire**

```text
TRIGGER_FREE → FireCount++
```

**dart-carriage-calibrate**

```text
CARRIAGE_CALIBRATE
```

### 5.2 外部命令

| 字符串 | 处理 |
|--------|------|
| `dart-init` / `dart_init` | DartInitTask |
| `dart-launch-prepare` / `dart_launch_prepare` / `launch_prepare` / `launch-prepare` | DartLaunchPrepareTask |
| `dart-launch-cancel` / `dart_launch_cancel` / `launch_cancel` / `cancel_launch` / `unload` | DartLaunchCancelTask |
| `dart-fire` / `dart_fire` / `fire_preload` / `fire` | DartFireTask |
| `dart-carriage-calibrate` / `dart_carriage_calibrate` / `carriage_init` / `carriage-init` | DartCarriageCalibrateTask |
| `cancel` | 清空队列 + Action cancel + 机构 ABORT 保持 + lifecycle=ERROR |
| `recover` | ERROR→IDLE，机构 idle，清 fire_count |

---

## 6. 时序

**成功：** `request(cmd)` → core BUSY → SUCCEEDED → Action SUCCESS → `idle()` → core IDLE  

**取消 / 失败：** `ABORT` 保持 → core ABORTED → lifecycle ERROR → 待 `recover` → `idle()` → core IDLE  

---

## 7. 实施顺序

| Step | 内容 |
|------|------|
| A | guidance：四枚举头 + CMake export |
| B | core depend + BeltController stub + plugin |
| C | Trigger + Filling stub |
| D | Resource + MechanismCommandAction |
| E | 三 Task + factory + DartManager 机构 IO |
| F | 编译联调 |

---

## 8. 已确认决策

| 项 | 决策 |
|----|------|
| 枚举 | guidance 包内 C++ 头，非 rosidl / 非 rmcs_msgs |
| 类型共享 | core depend dart_guidance |
| IDLE | 立即 status=IDLE |
| cancel / 失败 | ABORT 保持至 recover；成功只 idle |
| trigger | 舵机+滑台合并 |
| 动作粒度 | 命名原子，参数二阶段进 core |
| fire_count | guidance runtime |
| 视觉/底盘 | 本阶段不做 |

---

## 9. 风险

1. 成功帧内必须读到 SUCCEEDED，再 on_exit idle  
2. 段间 1 帧 IDLE 可接受  
3. core 依赖 guidance 倒置，日后可抽独立 msgs 包  
