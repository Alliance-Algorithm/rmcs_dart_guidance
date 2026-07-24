# 飞镖控制重构：第一阶段框架方案

> 状态：文档冻结，**尚未开始具体实现**  
> 前置：`rmcs_dart_guidance` 旧 actions/tasks/resources 已清空；`DartManager` 为可编译骨架  
> 关联：`rmcs_core/src/controller/dart/docs/refactor.md`

---

## 1. 目标与非目标

### 1.1 第一阶段目标

只搭 **接口契约 + 逐帧状态机骨架**，使：

1. `rmcs_core` 三机构控制器可 plugin 加载，消费命名命令、发布 `MechanismStatus`
2. `rmcs_dart_guidance` 通过 Resource 写命令、等 status，用现有 Task/Action 运行时推进流程
3. stub 完成：命令进入 `BUSY` → 固定 tick 后 `SUCCEEDED` → Action 成功 → Task 推进
4. guidance **不再**直接读写电机 angle/velocity/torque

### 1.2 非目标（本阶段不做）

- 真实堵转/行程/闭环控制律与 YAML 参数整定
- 视觉、manual、force 链路
- 底盘 4dof / 4z
- 与旧 `/dart_manager/*` 细粒度接口的兼容迁移层
- `carriage_init` 完整标定算法（接口可预留 `SLIDE_STALL_CALIB`）

---

## 2. 现状（清理后）

### 2.1 已删除

```
manager/resources/actions/*          # 全部旧动作
manager/resources/tasks/*            # 全部旧任务
manager/resources/task_factory.*
manager/resources/vision_aim_profile_provider.hpp
```

### 2.2 保留

```
manager/core/runtime/
  action.hpp
  action_sequence.hpp
  action_set.hpp
  task.hpp
manager/core/dart_manager.cpp        # 骨架：队列/生命周期/debug，无机构 IO
manager/manager_types.hpp            # 仅 lifecycle / runtime / debug 类型
manager/external_control/remote_command_bridge.cpp
vision/*                             # 暂不加载即可，代码保留
```

### 2.3 当前 DartManager 行为

- 输入：`/dart/manager/command`
- 输出：`fire_count` + debug lifecycle/task/action/queue/last_error
- 支持：`cancel` / `recover`
- 其它命令：日志忽略（待 Resource/Task 重建）

---

## 3. 架构

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
         cmd + status               cmd+setpoint+status         cmd + status
              │                          │                          │
              ▼                          ▼                          ▼
      /dart/belt/*               /dart/trigger/*             /dart/filling/*
              │                          │                          │
              ▼                          ▼                          ▼
      BeltController            TriggerController          FillingController
         (core FSM)              舵机+滑台 (core)              升降+限位 (core)
              │                          │                          │
              ▼                          ▼                          ▼
            电机/舵机                  电机/舵机                    电机/舵机
```

原则：

| 层 | 职责 |
|----|------|
| guidance Task | 业务顺序、fire_count 分支 |
| guidance Action | 写命名命令，等 status |
| guidance Resource | 对一组 command/status 接口的引用包装 |
| core Controller | 命名动作状态机 + 电机控制（二阶段填逻辑） |

---

## 4. 接口契约

### 4.1 Topic

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

Core 电机侧 topic 与 `4z-dart-launcher` 对齐时再定；框架期可 `register_input` optional，**不参与完成判定**。

### 4.2 枚举（建议 `rmcs_msgs`）

```cpp
enum class MechanismStatus : uint8_t {
    IDLE = 0,
    BUSY = 1,
    SUCCEEDED = 2,
    FAILED = 3,
    ABORTED = 4,
};

enum class BeltCommand : uint8_t {
    IDLE = 0,
    ABORT,
    DOWN_SLOW,   // 首次下行（原 fire_count==0 段）
    DOWN_FAST,   // 后续下行
    DOWN_HOLD,   // 第二段下行 + hold
    UP_SOFT,
    UP_HARD,
    UP_STALL,    // 末端堵转
    BRAKE,
};

enum class TriggerCommand : uint8_t {
    IDLE = 0,
    ABORT,
    SERVO_LOCK,
    SERVO_FREE,
    SLIDE_UP,
    SLIDE_DOWN,
    SLIDE_STALL_CALIB,
    SLIDE_GOTO,  // 配合 setpoint
};

enum class FillingCommand : uint8_t {
    IDLE = 0,
    ABORT,
    LIFT_UP,
    LIFT_DOWN,
    LIMIT_FREE,
    LIMIT_LOCK,
    LIMIT_PULSE_FILL,
};
```

### 4.3 命令语义

- **电平保持**：guidance 每帧维持当前 command，直到 Action 结束写 `IDLE`
- **抢占**：BUSY 中收到不同非 IDLE 命令 → 重开动作
- **终态保持**：`SUCCEEDED/FAILED/ABORTED` 保持到 guidance 写 `IDLE` 或新命令边沿
- **边沿检测**：`cmd 变化` 或 `从终态/IDLE 进入有效命令` 均可启动
- **ABORT**：停止输出 → `ABORTED`；cancel 任务时 Action `on_cancel` 发 ABORT

### 4.4 旧接口

不保留 `/dart_manager/belt/command|target_velocity|exit_mode|...` 等细粒度输出；不保留 guidance 对电机反馈的直接订阅。

---

## 5. Core 框架设计

### 5.1 文件

```
rmcs_core/src/controller/dart/
  mechanism_fsm.hpp          # 可选共享 stub FSM
  belt_controller.cpp
  trigger_controller.cpp
  filling_controller.cpp
```

插件类名：

- `rmcs_core::controller::dart::BeltController`
- `rmcs_core::controller::dart::TriggerController`
- `rmcs_core::controller::dart::FillingController`

注册到 `rmcs_core/plugins.xml`。CMake 已 `GLOB_RECURSE`，无需改构建。

### 5.2 通用状态机（stub）

```text
每帧:
  cmd = *command_in
  if cmd == ABORT:
      stop_outputs_stub()
      status = ABORTED
      return
  if cmd == IDLE:
      status = IDLE   # 或保持终态一帧后清 IDLE，实现时二选一并写死
      stop_outputs_stub()
      return
  if 新命令边沿:
      active_cmd = cmd
      tick = 0
      status = BUSY
  if status == BUSY:
      tick++
      run_busy_stub()           # 第一阶段空
      if tick >= stub_complete_ticks:
          status = SUCCEEDED
  *status_out = status
```

参数：`stub_complete_ticks`（默认 50）。

电机 `control_*` 输出框架期固定 0 / NaN / 安全值。

### 5.3 Trigger 合并范围

`trigger-controller` 同时管理：

- 扳机舵机：`SERVO_LOCK` / `SERVO_FREE`
- 丝杆滑台：`SLIDE_*` / `SLIDE_GOTO` + `setpoint`

---

## 6. Guidance 框架设计

### 6.1 待新建

```
manager/resources/
  belt_resource.hpp
  trigger_resource.hpp
  filling_resource.hpp
  actions/
    mechanism_command_action.hpp
    delay_action.hpp
    fire_count_increment_action.hpp
  tasks/
    launch_preparation_task.hpp
    fire_and_preload_task.hpp
    cancel_launch_task.hpp
  task_factory.hpp
  task_factory.cpp
```

### 6.2 Resource

由 `DartManager` 持有 `OutputInterface`/`InputInterface`，构造 Resource 时注入引用（非独立 Component）：

```cpp
struct BeltResource {
    rmcs_msgs::BeltCommand& command;
    const rmcs_msgs::MechanismStatus& status;

    void request(rmcs_msgs::BeltCommand c) { command = c; }
    void idle() { command = rmcs_msgs::BeltCommand::IDLE; }
    bool succeeded() const { return status == MechanismStatus::SUCCEEDED; }
    bool failed() const {
        return status == MechanismStatus::FAILED
            || status == MechanismStatus::ABORTED;
    }
};
```

`TriggerResource` 额外 `double& setpoint`。

### 6.3 MechanismCommandAction

```text
on_enter:  resource.request(cmd); [写 setpoint]
update:
  failed     → FAILURE
  succeeded  → SUCCESS
  else       → RUNNING
on_exit:     resource.idle()          # 推荐
on_cancel:   resource.request(ABORT)  # 或 idle，实现时统一
```

可选 `timeout_ticks`。

### 6.4 Task 骨架序列

**launch_prepare**

```text
(fire_count==0 ? DOWN_SLOW : DOWN_FAST) → DOWN_HOLD
→ SERVO_LOCK
→ [fire_count>0] LIFT_DOWN
→ UP_SOFT → UP_HARD → UP_STALL
```

**fire_preload**

```text
Delay → SERVO_FREE
→ [fire_count>0] LIFT_UP → LIMIT_PULSE_FILL
→ FireCountIncrement
```

**cancel_launch**

```text
DOWN_FAST → DOWN_HOLD → SERVO_FREE
→ UP_SOFT → UP_HARD → UP_STALL → LIFT_UP
```

第一阶段序列仅验证推进；core stub 一律 `SUCCEEDED`。

### 6.5 DartManager 恢复后 IO

```text
register_output  /dart/belt/command
register_input   /dart/belt/status
register_output  /dart/trigger/command
register_output  /dart/trigger/setpoint   (NaN default)
register_input   /dart/trigger/status
register_output  /dart/filling/command
register_input   /dart/filling/status
register_input   /dart/manager/command
register_output  fire_count + debug
```

删除：`ManagerInputContext` / `ManagerOutputContext` / `ManagerSettings`（已在清理中移除）。

任务工厂：`make_task(cmd, resources, runtime_state)`。

`cancel`：清空队列 + 当前 Action cancel + 三机构 ABORT/IDLE。  
`recover`：ERROR→IDLE，清 fire_count，可选再挂初始化 task。

### 6.6 外部命令字符串（factory）

| 字符串 | Task |
|--------|------|
| `launch_prepare` / `launch-prepare` | LaunchPreparationTask |
| `fire_preload` / `fire` | FireAndPreloadTask |
| `launch_cancel` / `cancel_launch` / `unload` | CancelLaunchTask |
| `cancel` | Manager 特殊：取消全部 |
| `recover` | Manager 特殊：恢复 |

`RemoteCommandBridge` 可继续发上述字符串；`manual_control` / `carriage_init` 等本阶段不实现。

---

## 7. 时序

### 7.1 正常完成

```text
Action.on_enter: command = UP_STALL
Core: 边沿 → BUSY
... stub_complete_ticks ...
Core: SUCCEEDED
Action.update: SUCCESS
Action.on_exit: command = IDLE
Core: IDLE
下一 Action...
```

### 7.2 取消

```text
command "cancel"
→ cancel 当前 Task/Action
→ Action.on_cancel: ABORT
→ Core: ABORTED，输出清零
→ Manager: 机构 IDLE，lifecycle = IDLE
```

---

## 8. 实施顺序（待开工）

| Step | 内容 | 依赖 |
|------|------|------|
| A | `rmcs_msgs` 增加 `MechanismStatus` + 三 Command 枚举 | 无 |
| B | core `BeltController` stub + plugins | A |
| C | core `TriggerController` + `FillingController` stub | A |
| D | guidance Resource + `MechanismCommandAction` | A |
| E | 重写三 Task + `task_factory` + 恢复 DartManager 机构 IO | D |
| F | 本地 executor 配置联调冒烟 | B–E |
| G | 同步更新 `controller/dart/docs/refactor.md` 接口表 | 契约稳定后 |

每步可独立编译；**在明确开工前不写业务控制逻辑**。

---

## 9. 验收标准（第一阶段完成时）

1. `rmcs_core` / `rmcs_dart_guidance` 编译通过  
2. 三控制器 + DartManager 可 plugin 加载  
3. 发 `launch_prepare` → 多段 Action 依次 SUCCESS（stub）  
4. `cancel` 不卡死，status 离开 BUSY  
5. guidance 无电机反馈 `register_input`  

---

## 10. 第二阶段预留

- core `run_busy`：堵转、行程角、舵机 settle、滑台闭环  
- 命名动作参数进 core YAML  
- `SLIDE_STALL_CALIB` / 可选标定 task  
- 视觉 Resource 与 manual  
- `KEEP` 式段间无 IDLE 间隙（若台架需要再优化）  

---

## 11. 已确认设计决策

| 项 | 决策 |
|----|------|
| trigger 范围 | 舵机 + 滑台合并进 trigger-controller |
| 动作粒度 | 命名原子动作，参数固化 core |
| 完成语义 | `MechanismStatus` 枚举，非单纯 bool done |
| fire_count | 留在 guidance `ManagerRuntimeState` |
| 视觉 | 本阶段断开 |
| 底盘 | 暂不接入 |

---

## 12. 风险备忘

1. 成功后必须 `idle()`，否则同命令可能无边沿；FSM 用双重边沿检测兜底  
2. 连续两段同机构动作中间 1 帧 IDLE 可接受（第一阶段不做 KEEP）  
3. 旧配置 yaml 中大量 `/dart_manager/*` 与电机阈值将失效，联调时需新配置  
