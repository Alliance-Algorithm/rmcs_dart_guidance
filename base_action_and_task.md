# 基础预设任务

- dart-init
    1. 并行动作,all-success{
        - BeltCommand::INIT,
        - FillingCommand::LIFT_UP,
        - TriggerCommand::TRIGGER_FREE
    }
    

- dart-launch-prepare
    - fire-count = 0，即首次发射：
    1. BeltCommand::DOWN_SLOW
    2. TriggerCommand::TRIGGER_LOCK
    3. BeltCommand::UP_SOFT

    - fire-count >0，即后续发射：
    1. BeltCommand::DOWN_FAST
    2. FillingCommand::LIFT_DOWN
    3. BeltCommand::UP_SOFT_PART
    4. FillingCommand::LIFT_UP
    5. BeltCommand::DOWN_SLOW_PART
    6. TriggerCommand::TRIGGER_LOCK
    7. 并行任务{
        - BeltCommand::UP_SOFT
        - FillingCommand::LIMIT_PULSE_FILL
    } 

- dart-launch-cancel
    1. BeltCommand::DOWN_FAST
    2. TriggerCommand::TRIGGER_FREE
    3. BeltCommand::UP_HARD

- dart-fire
    1. TriggerCommand::TRIGGER_FREE

- dart-carriage-calibrate
    1. TriggerCommand::CARRIAGE_CALIBRATE
