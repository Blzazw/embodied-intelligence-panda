"""
具身智能项目 - 抓取演示
功能：实现"抓取-移动-释放"完整流水线
      1. 移动到物体上方（准备抓取）
      2. 下降抓取（闭合夹爪）
      3. 抬升并移动（搬运）
      4. 释放物体（打开夹爪）
"""

import pybullet as p
import pybullet_data
import time
import numpy as np


def setup_simulation():
    """初始化仿真"""
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.8,
        cameraYaw=60,
        cameraPitch=-40,
        cameraTargetPosition=[0.5, 0, 0.1]
    )
    return physics_client


def load_environment():
    """加载环境：地面 + 机械臂 + 被抓取物体"""
    # 地面
    plane_id = p.loadURDF("plane.urdf")
    
    # Franka Panda 机械臂
    robot_id = p.loadURDF(
        "franka_panda/panda.urdf",
        useFixedBase=True
    )
    
    # 可控关节和末端
    controllable_joints = [0, 1, 2, 3, 4, 5, 6]
    finger_joints = [9, 10]  # Panda 夹爪关节
    end_effector_index = 11  # panda_hand
    
    # 创建一个绿色方块作为抓取目标
    cube_half_extents = [0.03, 0.03, 0.03]
    cube_id = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=cube_half_extents
    )
    cube_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=cube_half_extents,
        rgbaColor=[0.2, 0.8, 0.2, 1]
    )
    cube_pos = [0.5, 0.0, 0.03]  # 放在地面上
    cube_body_id = p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=cube_id,
        baseVisualShapeIndex=cube_visual,
        basePosition=cube_pos
    )
    
    return robot_id, controllable_joints, finger_joints, end_effector_index, cube_body_id


def set_finger_positions(robot_id, finger_joints, open_fingers=True):
    """控制夹爪开合"""
    target = 0.04 if open_fingers else 0.0
    for joint_idx in finger_joints:
        p.setJointMotorControl2(
            robot_id, joint_idx,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=100.0
        )


def move_to_position(robot_id, controllable_joints, end_effector_index,
                     target_position, steps=120, verbose=False):
    """移动到指定位置"""
    joint_positions = p.calculateInverseKinematics(
        robot_id, end_effector_index, target_position,
        maxNumIterations=100, residualThreshold=1e-4
    )
    joint_angles = joint_positions[:7]
    
    for step in range(steps):
        for j, joint_idx in enumerate(controllable_joints):
            p.setJointMotorControl2(
                robot_id, joint_idx,
                p.POSITION_CONTROL,
                targetPosition=joint_angles[j],
                force=500.0
            )
        p.stepSimulation()
        if verbose and step % 30 == 0:
            ls = p.getLinkState(robot_id, end_effector_index)
            pos = ls[0]
            print(f"  步数 {step:3d}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        time.sleep(1/240.)
    
    return joint_angles


def get_end_effector_position(robot_id, end_effector_index):
    """获取末端当前坐标"""
    link_state = p.getLinkState(robot_id, end_effector_index)
    return link_state[0]


def main():
    print("=" * 60)
    print("  具身智能项目：抓取-移动-释放 完整流水线")
    print("=" * 60)
    
    # 初始化
    physics_client = setup_simulation()
    (robot_id, controllable_joints, finger_joints,
     end_effector_index, cube_body_id) = load_environment()
    
    time.sleep(1)  # 等待 GUI 加载
    
    print("\n初始位置：夹爪张开")
    set_finger_positions(robot_id, finger_joints, open_fingers=True)
    for _ in range(60):
        p.stepSimulation()
        time.sleep(1/240.)
    
    # === 第一阶段：移动到物体上方 ===
    print("\n【阶段 1/4】移动到物体上方准备抓取...")
    pre_grasp_pos = [0.5, 0.0, 0.15]
    move_to_position(robot_id, controllable_joints, end_effector_index,
                     pre_grasp_pos, steps=120)
    
    # === 第二阶段：下降抓取 ===
    print("\n【阶段 2/4】下降抓取...")
    grasp_pos = [0.5, 0.0, 0.06]
    move_to_position(robot_id, controllable_joints, end_effector_index,
                     grasp_pos, steps=80)
    
    # 闭合夹爪
    print("闭合夹爪...")
    set_finger_positions(robot_id, finger_joints, open_fingers=False)
    for _ in range(60):
        p.stepSimulation()
        time.sleep(1/240.)
    
    # === 第三阶段：抬升并搬运 ===
    print("\n【阶段 3/4】抬升并搬运到目标位置...")
    
    # 先抬升
    lift_pos = [0.5, 0.0, 0.25]
    move_to_position(robot_id, controllable_joints, end_effector_index,
                     lift_pos, steps=80)
    
    # 移动到目标位置
    target_pos = [0.5, 0.4, 0.25]
    move_to_position(robot_id, controllable_joints, end_effector_index,
                     target_pos, steps=120)
    
    # 下降到放置高度
    place_pos = [0.5, 0.4, 0.08]
    move_to_position(robot_id, controllable_joints, end_effector_index,
                     place_pos, steps=60)
    
    # === 第四阶段：释放物体 ===
    print("\n【阶段 4/4】释放物体...")
    set_finger_positions(robot_id, finger_joints, open_fingers=True)
    for _ in range(60):
        p.stepSimulation()
        time.sleep(1/240.)
    
    # 抬升离开
    print("抬升离开...")
    retreat_pos = [0.5, 0.4, 0.30]
    move_to_position(robot_id, controllable_joints, end_effector_index,
                     retreat_pos, steps=60)
    
    print("\n" + "=" * 60)
    print("  ✅ 抓取-移动-释放 完整流水线执行完毕！")
    print("=" * 60)
    print("  过程总结：")
    print("    [1] 移动到物体上方 → [2] 下降 → 闭合夹爪")
    print("    [3] 抬升 → 水平搬运 → 下降到放置位置")
    print("    [4] 打开夹爪 → 抬升离开")
    print("\n  观察提示：")
    print("    - 红色球体标记了关键路径点")
    print("    - 绿色方块是被抓取的物体")
    print("    - 观察夹爪开合和物体的移动轨迹")
    print("\n仿真运行中，按 Ctrl+C 退出...")
    
    try:
        while True:
            p.stepSimulation()
            time.sleep(1/240.)
    except KeyboardInterrupt:
        p.disconnect()
        print("仿真已停止。")


if __name__ == "__main__":
    main()
