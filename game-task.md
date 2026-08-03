# 比赛控制任务 game control task

新增比赛控制任务：

- 触发条件：当/referee/game/stage为STARTED时，且/referee/dart/remaining_time大于15时，发布一次任务，需要去重，即remainingtime为30时接到了这个任务，后续正在执行任务时无需再次安排这个任务

- 比赛时任务可以无视遥控器的双下停止，即无论遥控器在什么状态都能执行

- 当遥控器双中，且连续3s有/remote/rotary_knob_switch == up，也可以触发这个任务

## 流程

1. 连续执行四个任务：launch-prepare，dart-fire，launch-prepare，dart-fire
2. 任务结束后，发射计数不清零，即如果一场比赛有两触发了这个任务，则第一次用1，2的参数，第二次用3，4的参数