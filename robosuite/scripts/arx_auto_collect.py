from dataclasses import dataclass
import time
import random
import numpy as np
import robosuite as suite

@dataclass
class AutoCollectConfig:
    robot: str = "Arx5"
    env_name: str = "Lift"
    has_renderer: bool = True
    ignore_done: bool = True
    use_camera_obs: bool = False
    control_freq: int = 20
    gripper_type: str = "ArxGripper"

class ArxRobotController:
    """ARX5机器人数据收集控制器"""
    
    def __init__(self, env):
        self.env = env
        self.target_reached = False
        self.current_phase = "approach"  # approach -> grasp -> lift
        self.phases = ["approach", "grasp", "lift"]
        self.phase_index = 0
        
        # 控制参数
        self.movement_speed = 0.05
        self.rotation_speed = 0.05
        self.position_tolerance = 0.04  # 放宽位置容差，避免卡死
        self.orientation_tolerance = 0.2  # 放宽姿态容差
        self.grasp_height_offset = 0.3  # 抓取高度偏移
        self.lift_height = 0.4  # 提升高度
        
        # 路径规划
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # 状态跟踪
        self.gripper_closed = False
        
        print("✅ ARX5控制器初始化完成")
    
    def get_ee_position(self):
        """获取末端执行器位置"""
        robot = self.env.robots[0]
        eef_site_id = robot.eef_site_id["right"]
        return self.env.sim.data.site_xpos[eef_site_id].copy()
    
    def get_ee_orientation(self):
        """获取末端执行器姿态（四元数）"""
        robot = self.env.robots[0]
        eef_site_id = robot.eef_site_id["right"]
        # 获取旋转矩阵
        rotation_matrix = self.env.sim.data.site_xmat[eef_site_id].reshape(3, 3)
        # 简化：直接返回旋转矩阵的第一行作为方向向量
        return rotation_matrix[2, :]  # Z轴方向（末端执行器朝向）
    
    def get_cube_position(self):
        """获取方块位置"""
        return self.env.sim.data.body_xpos[self.env.cube_body_id].copy()
    
    def plan_trajectory(self):
        """规划抓取轨迹"""
        cube_pos = self.get_cube_position()
        ee_pos = self.get_ee_position()
        
        print(f"🎯 开始规划轨迹:")
        print(f"   当前末端位置: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
        print(f"   方块位置: [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")
        
        # 清空之前的路径点
        self.waypoints = []
        
        # 定义抓取姿态：夹爪朝下
        # 使用方向向量 [0, 0, -1] 表示Z轴朝下
        grasp_orientation = np.array([0.0, 0.0, -1.0])  # 夹爪朝下
        
        # 阶段1: 移动到方块上方，调整到正确姿态
        approach_pos = cube_pos.copy()
        approach_pos[2] += self.grasp_height_offset
        self.waypoints.append({
            'position': approach_pos,
            'orientation': grasp_orientation,
            'gripper': 1.0,  # 打开夹爪（正值表示完全打开）
            'phase': 'approach'
        })
        
        # 阶段2: 下降到抓取位置
        grasp_pos = cube_pos.copy()
        grasp_pos[2] += 0.05  # 稍微高于方块表面（增加到5cm，避免太低）
        self.waypoints.append({
            'position': grasp_pos,
            'orientation': grasp_orientation,
            'gripper': 1.0,  # 保持打开
            'phase': 'grasp'
        })
        
        # 阶段3: 闭合夹爪
        grasp_pos = cube_pos.copy()
        grasp_pos[2] += 0.01  # 稍微高于方块表面（增加到1cm，避免太低）
        self.waypoints.append({
            'position': grasp_pos,
            'orientation': grasp_orientation,
            'gripper': -1.0,  # 闭合夹爪（负值表示完全闭合）
            'phase': 'grasp'
        })
        
        # 阶段4: 提升方块
        lift_pos = grasp_pos.copy()
        lift_pos[2] += self.lift_height
        self.waypoints.append({
            'position': lift_pos,
            'orientation': grasp_orientation,
            'gripper': -1.0,  # 保持闭合
            'phase': 'lift'
        })
        
        self.current_waypoint_index = 0
        
        print(f"📋 规划了 {len(self.waypoints)} 个路径点:")
        for i, wp in enumerate(self.waypoints):
            pos = wp['position']
            ori = wp['orientation']
            print(f"   {i+1}. 位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] "
                  f"姿态: [{ori[0]:.2f}, {ori[1]:.2f}, {ori[2]:.2f}] "
                  f"夹爪: {wp['gripper']:.1f} 阶段: {wp['phase']}")
        
        return True
    
    def quaternion_distance(self, v1, v2):
        """计算两个方向向量之间的角度距离"""
        # 归一化向量
        v1_norm = v1 / (np.linalg.norm(v1) + 1e-8)
        v2_norm = v2 / (np.linalg.norm(v2) + 1e-8)
        # 计算点积
        dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        # 返回角度差
        return np.arccos(np.abs(dot_product))
    
    def get_action_to_waypoint(self, target_waypoint):
        """计算到达目标路径点的动作"""
        current_ee_pos = self.get_ee_position()
        current_ee_ori = self.get_ee_orientation()
        target_pos = target_waypoint['position']
        target_ori = target_waypoint['orientation']
        target_gripper = target_waypoint['gripper']
        
        # 计算位置误差
        pos_error = target_pos - current_ee_pos
        pos_distance = np.linalg.norm(pos_error)
        
        # 计算姿态误差
        ori_distance = self.quaternion_distance(current_ee_ori, target_ori)
        
        # 检查是否到达目标（位置和姿态都要满足）
        position_reached = pos_distance < self.position_tolerance
        orientation_reached = ori_distance < self.orientation_tolerance
        
        if position_reached and orientation_reached:
            return None, True  # 返回None表示已到达
        
        # 计算位置运动
        if pos_distance > 0:
            pos_direction = pos_error / pos_distance
            # 使用更激进的运动策略
            if pos_distance > 0.1:  # 如果距离较远，使用最大速度
                pos_movement = pos_direction * self.movement_speed
            else:
                pos_movement = pos_direction * max(0.05, pos_distance * 3)  # 近距离时减速
        else:
            pos_movement = np.zeros(3)
        
        # 计算姿态运动（简化方法）
        if ori_distance > 0:
            # 计算目标方向与当前方向的叉积，得到旋转轴
            current_ori_norm = current_ee_ori / (np.linalg.norm(current_ee_ori) + 1e-8)
            target_ori_norm = target_ori / (np.linalg.norm(target_ori) + 1e-8)
            
            rotation_axis = np.cross(current_ori_norm, target_ori_norm)
            rotation_magnitude = np.linalg.norm(rotation_axis)
            
            if rotation_magnitude > 1e-6:
                # 归一化旋转轴并限制旋转速度
                rotation_axis = rotation_axis / rotation_magnitude
                rotation_speed = min(self.rotation_speed, ori_distance)
                ori_movement = rotation_axis * rotation_speed
            else:
                ori_movement = np.zeros(3)
        else:
            ori_movement = np.zeros(3)
        
        # 构造动作向量
        action_dim = self.env.action_dim
        action = np.zeros(action_dim)
        
        # 位置和姿态控制
        if action_dim >= 6:
            action[:3] = pos_movement  # 位置增量
            action[3:6] = ori_movement  # 姿态增量
        
        # 夹爪控制（ARX5夹爪只需要一个维度）
        if action_dim >= 7:
            action[6] = target_gripper  # 夹爪控制（单一维度控制两个手指）
        
        return action, False
    
    def update(self):
        """更新控制器状态并返回动作"""
        if self.current_waypoint_index >= len(self.waypoints):
            print("🎉 所有路径点执行完成！机器人已到达方块上方")
            return None
        
        current_waypoint = self.waypoints[self.current_waypoint_index]
        action, reached = self.get_action_to_waypoint(current_waypoint)
        
        # 添加超时检测，避免卡死
        if hasattr(self, 'waypoint_start_time'):
            if time.time() - self.waypoint_start_time > 10.0:  # 10秒超时
                print(f"⚠️  路径点 {self.current_waypoint_index + 1} 超时，强制跳过")
                self.current_waypoint_index += 1
                self.waypoint_start_time = time.time()
                return self.update()
        else:
            self.waypoint_start_time = time.time()
        
        if reached:
            print(f"✅ 到达路径点 {self.current_waypoint_index + 1}/{len(self.waypoints)} "
                  f"({current_waypoint['phase']})")
            self.current_waypoint_index += 1
            self.waypoint_start_time = time.time()  # 重置计时器
            
            # 如果是夹爪动作，需要等待夹爪完全执行
            if abs(current_waypoint['gripper']) > 0.5:
                if current_waypoint['gripper'] > 0.5:
                    print("🤏 夹爪闭合中...")
                else:
                    print("👐 夹爪打开中...")
                # 夹爪需要约80步才能完全动作（@20Hz = 4秒）
                time.sleep(0.5)  # 等待夹爪完成动作
            
            # 递归调用获取下一个动作
            return self.update()
        
        return action
    
    def is_complete(self):
        """检查是否完成所有任务"""
        return self.current_waypoint_index >= len(self.waypoints)

def create_arx_environment():
    """创建ARX5机器人环境 - 将机器人安装在桌子上"""
    
    # 创建环境 - 使用默认的OSC_POSE控制器
    env = suite.make(
        env_name="Lift",
        robots="Arx5",
        gripper_types="ArxGripper",
        has_renderer=True,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        use_object_obs=True,
        control_freq=20,
        horizon=2000,  # 增加时间限制
        reward_shaping=True,
        ignore_done=True,  # 忽略done信号
        # 自定义机器人放置
        placement_initializer=None,  # 使用默认放置
    )
    
    print("✅ 环境创建成功")
    return env

def collect_demonstration():
    """收集演示数据"""
    # 创建环境
    env = create_arx_environment()
    
    # 重置环境
    obs = env.reset()
    print("🔄 环境重置完成")
    
    # 创建控制器
    controller = ArxRobotController(env)
    
    # 手动调整机器人初始位置 - 让它更接近桌子
    print("🔧 调整机器人初始位置...")
    
    # 设置更好的初始关节角度，让机器人手臂朝向桌子
    robot = env.robots[0]
    joint_angles = [0.0, 0, 0, 0, 0.0, 0.0]  # 让机器人手臂更向前伸展
    
    # 找到机器人关节的qpos索引
    joint_indices = []
    for joint_name in robot.robot_joints:
        joint_id = env.sim.model.joint_name2id(joint_name)
        qpos_addr = env.sim.model.jnt_qposadr[joint_id]
        joint_indices.append(qpos_addr)
    
    # 应用新的关节角度
    for i, angle in enumerate(joint_angles):
        if i < len(joint_indices):
            env.sim.data.qpos[joint_indices[i]] = angle
    
    # 执行前向动力学更新位置
    env.sim.forward()
    
    print("✅ 机器人位置调整完成")
    
    # 等待环境稳定
    print("⏳ 等待环境稳定...")
    for _ in range(100):
        env.step(np.zeros(env.action_dim))
    
    # 规划轨迹
    if not controller.plan_trajectory():
        print("❌ 轨迹规划失败")
        return
    
    print("\n🚀 开始执行演示...")
    
    step_count = 0
    max_steps = 20000
    
    while not controller.is_complete() and step_count < max_steps:
        # 获取控制动作
        action = controller.update()
        
        if action is None:
            # 如果控制器返回None，使用零动作
            action = np.zeros(env.action_dim)
        
        # 执行动作
        obs, reward, done, info = env.step(action)
        
        # 检查是否达到终止条件
        if done and not env.ignore_done:
            print(f"🏁 环境终止：done={done}")
            break
        
        # 渲染
        env.render()
        
        step_count += 1
        
        # 打印状态信息
        if step_count % 50 == 0:  # 更频繁地打印状态
            robot = env.robots[0]
            eef_site_id = robot.eef_site_id["right"]
            ee_pos = env.sim.data.site_xpos[eef_site_id]
            cube_pos = env.sim.data.body_xpos[env.cube_body_id]
            
            # 当前路径点信息
            if controller.current_waypoint_index < len(controller.waypoints):
                current_wp = controller.waypoints[controller.current_waypoint_index]
                target_pos = current_wp['position']
                distance = np.linalg.norm(ee_pos - target_pos)
                print(f"步骤 {step_count}: EE位置 [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] "
                      f"目标 [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] "
                      f"距离: {distance:.3f}m")
            else:
                print(f"步骤 {step_count}: EE位置 [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] "
                      f"方块位置 [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")
        
        # 小延时以便观察
        time.sleep(0.005)  # 减少延时
    
    if controller.is_complete():
        print("🎉 演示收集完成！")
    else:
        print("⏰ 达到最大步数限制")
    
    env.close()

if __name__ == "__main__":
    collect_demonstration()