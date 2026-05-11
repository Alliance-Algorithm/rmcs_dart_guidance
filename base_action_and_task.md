#　Task and Action　列表

## 基础自动动作

- belt_init
    - belt_up

- carriage_init
    - carriage_calibration_prepare
    - 3 x (carriage_init -> carriage_origin_record -> carriage_travel)
    - carriage_origin_average_finalize
    - carriage_calibration_park

- launch_prepare
    - (fire_count == 0) belt_down(wait) -> trigger_lock -> belt_up
    - (fire_count > 0) belt_down(wait) -> trigger_lock -> filling_lift_down -> belt_up

- launch_cancel
    - belt_down(wait) -> trigger_free -> belt_up(zero)

- fire_preload
    - (fire_count == 0) trigger_free
    - (fire_count > 0) trigger_free -> filling_lift_up -> filling_limit_servo

## 传感器闭环自动动作
- force_auto_control
    
    描述：传入一个设定的力度，发布setpoing给下层让其闭环（下层实现这里不需要管），但是上层需要获取力传感器数据判断任务是否成功

- angle_auto_control
    
    描述： 视觉那边会传来当前识别到的目标的坐标，发布与设定值的误差（打包成vector2d），通过误差判断任务是否成功

## 调试动作

### 单步动作组成的任务

- trigger_free/trigger_lock
- belt_up（退出时零速闭环）/belt_down（退出时保留力矩）
- filling_lift_up/filling_lift_down
- 

### 手动控制任务

- 手动控制角度（以恒定速度）
- 手动控制力度（以恒定速度）
- 手动控制同步带上下（退出时零速闭环）
- 手动控制填装机构升降（以恒定速度）
