# DartManager

RMCS 框架插件组件，负责：
1. 维护飞镖全局状态机 (State Machine)
2. 管理任务队列 (Task Queue)，每帧驱动当前任务向前推进
3. 通过 RMCS Interface 黑板与底层硬件组件通信
4. 对任务失败提供统一的异常处理入口

## 当前目录结构

```text
src/manager/
├── core/
│   ├── components/   # 组件入口：DartManager / RemoteCommandBridge
│   └── runtime/      # 通用运行时：Action / ActionSequence / ActionSet / Task
└── resources/
    ├── actions/      # 具体机械动作
    ├── tasks/        # 由多个 action 组装出的业务 task
    └── task_factory.*# 命令到 task 的统一装配入口
```

重构后的依赖方向为：`core/components -> resources/task_factory -> resources/tasks|actions -> core/runtime`。
这样 `manager` 的核心调度逻辑与自定义动作资源分离，新增动作/任务时不需要再把实现塞进组件入口文件。

## 状态机转换图

```text
  ┌─────────────────────────────────────────────────────────────────┐
  │                                                                 │
  │   IDLE ──[提交Task]──► RUNNING ──[SUCCESS]──► IDLE              |
  │     ▲                     │                                     │
  │     │                     ├──[FAILURE]──► ERROR                 │
  │     │                     │                                     │
  │  [recover]            [cancel()]                                │
  │     │                     │                                     │
  │   ERROR ◄────────────────-┘                                     │
  └─────────────────────────────────────────────────────────────────┘
```

## 简单开发手册

### 1. 组件初始化 (`DartManager`)
`DartManager` 继承自 `rmcs_executor::Component` 和 `rclcpp::Node`，在初始化时会注册一系列输入输出接口，用于与底层硬件和外部组件通信。

### 2. 状态机管理
系统内部运行态由 `ManagerLifecycleState` 定义：
- `IDLE`：空闲，无任务运行。
- `RUNNING`：有任务正在执行。
- `ERROR`：任务失败，等待恢复。

每帧通过 `update()` 函数轮询命令并驱动任务执行：
- **空闲 (`IDLE`)**：从队列中取出下一个任务并开始执行。
- **运行中 (`RUNNING`)**：调用 `tick_current_task()` 推进当前任务。根据任务返回的状态 (`SUCCESS`, `FAILURE`, 或 `RUNNING`) 进行状态转换。
- **错误 (`ERROR`)**：暂停调度，等待外部发送 `recover` 命令。

### 3. 命令处理
当前阶段 `DartManager` 只接收离散命令，命令由 `RemoteCommandBridge` 统一写入 `/dart/manager/command`。

遥控器映射如下：
- 双下：`cancel`
- 左拨杆 `DOWN -> MIDDLE`：`recover`
- 左拨杆进入 `UP`：`manual_control`
- 左拨杆保持 `MIDDLE` 且右拨杆 `MIDDLE -> DOWN`：`launch_prepare` / `launch_prepare_with_vision` / `launch_cancel`
- 左拨杆保持 `MIDDLE` 且右拨杆 `MIDDLE -> UP`：`fire_preload`

内置保留命令包括：
- `cancel`：取消当前任务并清空任务队列。
- `recover`：从 `ERROR` 状态恢复到 `IDLE`。

其他任务命令（如 `launch_prepare`, `fire_preload`）会在 `poll_command()` 中解析并生成对应的 `Task` 加入队列。
当 `vision_enable=true` 时，遥控发射准备入口会发出 `launch_prepare_with_vision`；否则发出 `launch_prepare`。

`carriage_init` / `carriage-init` 仍可通过 `/dart/manager/command` 触发，但语义已经是完整标定流程：
- 连续 3 次低速逼近机械限位并记录编码器原点样本
- 对 3 次样本求平均，写回新的参考零点
- 按 `UP` 方向运行到 `carriage_calibration_parking_angle` 定义的校准停靠位

ROS 话题 `/carriage_position/calibrate` 收到非 0 时，会先由 `RemoteCommandBridge` 转发为
`carriage_init` 命令，再进入同一个完整标定 task。

`launch_prepare_with_vision` 当前的瞄准语义为：
- yaw：继续使用视觉识别目标的像素 x 误差
- pitch：使用 `vision_aim.shot_profiles.N.pitch_angle` 与 `/imu/catapult_pitch_angle` 的角度误差
- carriage：仍由 `trigger_carriage_position_aim` 独立控制，不参与上述修改

### 4. 任务调度 (`Task` & `Action`)
开发者可以通过派生 `Action` 或组装现有的 Action 创建 `Task`。组件层会先组装 `ManagerInputContext`、`ManagerOutputContext`、`ManagerSettings` 和 `ManagerRuntimeState`，再由 `task_factory` 统一创建具体任务，避免 `DartManager` 直接依赖所有自定义资源实现。

`ActionSet` 仍然作为通用 runtime 能力保留，但当前 manager 主流程主要使用顺序 `Task` 和 `ActionSequence`。

当前 motor 控制有三类退出语义：
- `WAIT_ZERO_VELOCITY`：退出后进入零速闭环等待。
- `WAIT_HOLD_TORQUE`：退出后进入 `WAIT + hold torque`，保持同步带对滑块的压紧状态，直到下一条 belt 指令覆盖。
- `KEEP`：退出后保持当前输出不变，直到下一条命令覆盖。

当前 manager 还会通过 `/dart/manager/fire_count` 输出成功完成 `fire_preload` 的次数。
任务分支使用 `fire_count` 判断是否为首发：
- `fire_count == 0`：首发流程
- `fire_count > 0`：后续流程
- `recover` 会将 `fire_count` 清零

任务执行失败时会触发 `on_task_failure()`，该函数会将输出置零，保证系统安全，并将状态机置为 `ERROR`。
