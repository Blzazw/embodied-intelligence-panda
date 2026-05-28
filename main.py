"""
具身智能项目 - 主程序
功能：使用 PyBullet 仿真环境，通过逆运动学控制 Franka Panda 机械臂
      使末端执行器精确移动到指定的三维空间目标点。
"""

import pybullet as p
import pybullet_data
import time
import numpy as np


def setup_simulation():
    """初始化 PyBullet 仿真环境"""
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=50,
        cameraPitch=-35,
        cameraTargetPosition=[0.5, 0, 0.2]
    )
    return physics_client


def load_environment():
    """加载地面和机械臂"""
    # 加载地面
    plane_id = p.loadURDF("plane.urdf")
    
    # 加载 Franka Panda 机械臂（固定基座）
    robot_id = p.loadURDF(
        "franka_panda/panda.urdf",
        useFixedBase=True
    )
    
    # 获取关节信息
    num_joints = p.getNumJoints(robot_id)
    joint_info = []
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_info.append({
            'index': i,
            'name': info[1].decode('utf-8'),
            'type': info[2],
        })
    
    # Panda 的可控关节（前7个为位置控制关节）
    controllable_joints = [0, 1, 2, 3, 4, 5, 6]
    # 末端执行器 link 索引（panda_link7 之后的 link8 是末端）
    end_effector_index = 11  # panda_hand
    
    return robot_id, controllable_joints, end_effector_index


def move_to_target(robot_id, controllable_joints, end_effector_index, 
                   target_position, max_steps=240):
    """
    通过逆运动学将机械臂末端移动到目标位置
    
    参数:
        robot_id: 机械臂 ID
        controllable_joints: 可控关节索引列表
        end_effector_index: 末端执行器 Link 索引
        target_position: 目标位置 [x, y, z]
        max_steps: 最大仿真步数
    
    返回:
        joint_positions: 逆运动学求解的关节角度
        actual_position: 实际到达的位置
    """
    print(f"\n=== 移动到目标位置: {target_position} ===")
    
    # 逆运动学求解
    joint_positions = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        target_position,
        maxNumIterations=100,
        residualThreshold=1e-4
    )
    
    # 截取前7个关节角度
    joint_angles = joint_positions[:7]
    
    print(f"计算得到的关节角度: {[round(angle, 4) for angle in joint_angles]}")
    
    # 逐步控制机械臂移动到目标位置
    for step in range(max_steps):
        for j, joint_idx in enumerate(controllable_joints):
            p.setJointMotorControl2(
                robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=joint_angles[j],
                force=500.0
            )
        p.stepSimulation()
        
        # 每30步获取并输出末端位置
        if step % 30 == 0:
            link_state = p.getLinkState(robot_id, end_effector_index)
            pos = link_state[0]
            print(f"  步数 {step:3d}: 末端位置 [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
        
        time.sleep(1 / 240.)
    
    # 获取最终末端位置
    final_state = p.getLinkState(robot_id, end_effector_index)
    actual_position = final_state[0]
    
    return joint_angles, actual_position


def add_visual_marker(position, color=[1, 0, 0], size=0.03):
    """在目标位置添加可视化标记"""
    marker_id = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=size,
        rgbaColor=color + [0.8]
    )
    body_id = p.createMultiBody(
        baseVisualShapeIndex=marker_id,
        basePosition=position
    )
    return body_id


def main():
    """主程序"""
    print("=" * 60)
    print("  具身智能项目：PyBullet 机械臂逆运动学控制")
    print("=" * 60)
    
    # 初始化仿真
    physics_client = setup_simulation()
    robot_id, controllable_joints, end_effector_index = load_environment()
    
    # 等待 GUI 加载
    time.sleep(1)
    
    # 定义多个目标位置进行测试
    target_positions = [
        [0.5, 0.2, 0.3],   # 右前方高处
        [0.5, -0.2, 0.3],  # 左前方高处
        [0.3, 0.0, 0.1],   # 正前方低处
        [0.6, 0.1, 0.4],   # 更远更高
        [0.4, -0.3, 0.2],  # 左前方中处
    ]
    
    results = []
    
    for i, target in enumerate(target_positions):
        print(f"\n{'='*40}")
        print(f"  测试 {i+1}/{len(target_positions)}")
        print(f"{'='*40}")
        
        # 添加目标点标记
        add_visual_marker(target, color=[1, 0, 0])
        
        # 移动到目标位置
        joint_angles, actual_pos = move_to_target(
            robot_id, controllable_joints, end_effector_index, target
        )
        
        # 计算误差
        error = np.linalg.norm(np.array(actual_pos) - np.array(target))
        print(f"\n目标位置: [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]")
        print(f"实际位置: [{actual_pos[0]:.4f}, {actual_pos[1]:.4f}, {actual_pos[2]:.4f}]")
        print(f"定位误差: {error:.6f} 米")
        
        results.append({
            'target': target,
            'actual': actual_pos,
            'error': error,
            'joint_angles': joint_angles
        })
        
        # 在目标之间停留，便于观察
        time.sleep(0.5)
    
    # 输出结果汇总
    print("\n\n" + "=" * 60)
    print("  测试结果汇总")
    print("=" * 60)
    print(f"{'目标位置':<30} {'定位误差':<12} {'可达性'}")
    print("-" * 60)
    for r in results:
        target_str = f"[{r['target'][0]:.3f}, {r['target'][1]:.3f}, {r['target'][2]:.3f}]"
        error_mm = r['error'] * 1000  # 转换为毫米
        reachable = "✅ 可达" if r['error'] < 0.01 else "⚠️ 误差较大"
        print(f"{target_str:<30} {error_mm:.2f} mm{'':<6} {reachable}")
    
    print("\n仿真结束。按 Ctrl+C 退出，或关闭窗口。")
    
    # 保持仿真窗口打开
    try:
        while True:
            p.stepSimulation()
            time.sleep(1/240.)
    except KeyboardInterrupt:
        p.disconnect()
        print("仿真已停止。")


if __name__ == "__main__":
    main()
