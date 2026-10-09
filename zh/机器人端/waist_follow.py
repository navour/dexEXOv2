#!/usr/bin/env python3
"""armsdk 腰部跟随的纯逻辑, 与 DDS 无关, 便于在没有机器人时测试。

robot_arm_receiver.py 依赖 unitree_sdk2py, 在 PC 上导不进来; 腰部是直接
改变上半身重心的安全相关决策, 所以把判据单独放在这里, 由
PC端/test_waist_follow.py 按路径加载后覆盖。
"""


# armsdk 一并接管的腰部关节 (与官方 g1_arm5_sdk_dds_example 一致)。
WAIST_JOINTS = (12, 13, 14)
# 只驱动 waist_yaw。roll/pitch 限位只有 ±0.52 rad 且直接改变上半身重心在
# 矢状/冠状面的投影, 对运控服务是比 yaw 强得多的扰动, 恒定保持回中。
WAIST_YAW_JOINT = 12

# G1 URDF waist_yaw 限位是 ±2.618, 但整个上半身连着两条手臂, 转到极限会
# 把重心甩出脚掌。这里按 PC 端的发送上限 (±1.0) 再收一档作为独立防线 ——
# 发送端已经夹过一次, 接收端不信任发送端。
WAIST_YAW_LIMITS = (-1.0, 1.0)
# 腰带动整个上半身, 比单条手臂重得多, 限速单列, 明显低于 MAX_FOLLOW_SPEED。
WAIST_FOLLOW_SPEED = 1.5


def _clamp(value, limits):
    return max(limits[0], min(limits[1], value))


def _step_toward(current, target, max_step):
    diff = target - current
    if abs(diff) <= max_step:
        return target
    return current + max_step * (1.0 if diff > 0 else -1.0)


def waist_yaw_command(current, target_yaw, allow_waist, arm_following, dt,
                      return_speed):
    """算下一帧的 waist_yaw 指令角, 返回 (角度, 是否在跟随)。

    三个跟随条件缺一不可:
      allow_waist    命令行门控 (--waist), 默认关闭
      arm_following  至少一条手臂在跟随; mode=0 (暂停/超时) 时腰必须跟着
                     双臂一起回中, 不能留在扭着的姿态
      target_yaw     本帧确实收到有效尾块; 老发送端或坏尾块给 None

    不跟随时按 return_speed 回中, 与双臂回零同一套语义。
    输出无条件夹在 WAIST_YAW_LIMITS 内。
    """
    following = bool(
        allow_waist and arm_following and target_yaw is not None)
    if following:
        nxt = _step_toward(
            current,
            _clamp(float(target_yaw), WAIST_YAW_LIMITS),
            WAIST_FOLLOW_SPEED * dt)
    else:
        nxt = _step_toward(current, 0.0, return_speed * dt)
    return _clamp(nxt, WAIST_YAW_LIMITS), following
