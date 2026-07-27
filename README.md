# DartManager

RMCS 框架插件组件，负责：
1. 维护飞镖全局状态机 (State Machine)
2. 管理任务队列 (Task Queue)，每帧驱动当前任务向前推进
3. 通过 RMCS Interface 黑板与底层硬件组件通信
4. 对任务失败提供统一的异常处理入口

## 当前目录结构

```text
include/rmcs_dart_guidance/
├── msg/          # 命令/状态枚举
├── resource/     # 机构 IO 包装
├── action/       # 具体动作
└── task/         # 业务任务

src/manager/
├── runtime/                  # Action 运行时 + task_factory
├── dart_manager.cpp          # 组件入口
└── remote_command_bridge.cpp # 命令桥接
```

依赖方向：`dart_manager -> runtime/task_factory -> task/action -> resource -> msg`。

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
- 左拨杆保持 `MIDDLE` 且右拨杆 `MIDDLE -> DOWN`：`dart-launch-prepare`；若上一次已发布发射准备且尚未发射/取消/recover，则发布 `dart-launch-cancel`
- 左拨杆保持 `MIDDLE` 且右拨杆 `MIDDLE -> UP`：`dart-fire`

ROS 命令来源：
- `/chassis/calibrate` (`std_msgs/msg/Int32`)：收到消息后发布一次 `dart-chassis-zero-calibrate`
- `/chassis/leveling` (`std_msgs/msg/Int32`)：收到消息后发布一次 `dart-chassis-level`
- `/carriage/calibrate` (`std_msgs/msg/Int32`)：收到消息后发布一次 `dart-carriage-calibrate`

内置保留命令包括：
- `cancel`：取消当前任务并清空任务队列。
- `recover`：从 `ERROR` 状态恢复到 `IDLE`。

任务命令会在 `poll_command()` 中解析并生成对应的 `Task` 加入队列。
正式预设任务命令为 `dart-init`、`dart-launch-prepare`、`dart-launch-cancel`、
`dart-fire`、`dart-carriage-calibrate`、`dart-chassis-zero-calibrate`、
`dart-chassis-level`。兼容别名仍保留：`launch_prepare` / `launch-prepare`、
`launch_cancel` / `cancel_launch` / `unload`、`fire_preload` / `fire`、
`carriage_init` / `carriage-init`、`chassis-zero-calibrate` /
`chassis_zero_calibrate` / `zero_calibrate`、`chassis-level` / `chassis_level` /
`level`。

### 4. 任务调度 (`Task` & `Action`)
开发者可以通过派生 `Action` 或组装现有的 Action 创建 `Task`。组件层会先组装 `ManagerInputContext`、`ManagerOutputContext`、`ManagerSettings` 和 `ManagerRuntimeState`，再由 `task_factory` 统一创建具体任务，避免 `DartManager` 直接依赖所有自定义资源实现。

基础预设任务使用顺序 `Task` / `ActionSequence`，需要并行 all-success 时嵌套
`ActionSet`。

当前 motor 控制有三类退出语义：
- `WAIT_ZERO_VELOCITY`：退出后进入零速闭环等待。
- `WAIT_HOLD_TORQUE`：退出后进入 `WAIT + hold torque`，保持同步带对滑块的压紧状态，直到下一条 belt 指令覆盖。
- `KEEP`：退出后保持当前输出不变，直到下一条命令覆盖。

当前 manager 还会通过 `/dart/manager/fire_count` 输出成功完成 `dart-fire` 的次数。
任务分支使用 `fire_count` 判断是否为首发：
- `fire_count == 0`：首发流程
- `fire_count > 0`：后续流程
- `recover` 会将 `fire_count` 清零

滑台发射位置由 `dart_manager` 参数 `launch_carriage_position_1..4` 配置，单位为相对滑台标定零点的 encoder 值。`dart-carriage-calibrate` 在标定完成后 goto 位置 1；`dart-fire` 在释放扳机后 goto 下一发位置，`fire_count=0/1/>=2` 分别对应位置 2/3/4，第 4 发及之后重复位置 4，goto 成功后才递增 `fire_count`。

任务执行失败时会触发 `on_task_failure()`，该函数会将输出置零，保证系统安全，并将状态机置为 `ERROR`。
