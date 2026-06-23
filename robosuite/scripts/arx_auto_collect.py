from dataclasses import dataclass
import time
import random
import numpy as np
import robosuite as suite
import h5py
import cv2  # cv2 仍然用于图像处理 (flipud)
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R
# import moviepy.editor as mpy # 将在需要时导入

@dataclass
class AutoCollectConfig:
    robot: str = "Arx5"
    env_name: str = "Lift"
    has_renderer: bool = True
    ignore_done: bool = True
    use_camera_obs: bool = True  # 启用相机观测
    control_freq: int = 20
    gripper_type: str = "ArxGripper"
    record_freq: int = 10  # 数据记录频率 10Hz

class DataRecorder:
    """
    数据记录器,记录演示数据到HDF5文件
    (已修改为使用 moviepy 写入视频)
    """

    def __init__(self, save_dir="demonstrations"):
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

        self.demo_counter = self._get_next_demo_number()

        self.current_demo_data = {
            'external_cam': [],
            'robot0_right_eye_in_hand': [],
            'joint_states': [],
            'gripper_states': [],
            'actions': [],
            '_timestamps': []
        }

        # --- MoviePy 修改 ---
        # 移除 self.video_writer
        # 添加帧缓存列表
        self.video_frames = []
        # ---------------------

        self.video_path = None
        self.hdf5_path = None
        self.record_interval = 1.0 / 10.0  # 10Hz
        self.last_record_time = 0

        print(f"✅ 数据记录器初始化完成 (使用 moviepy)，保存目录: {save_dir}")
        print(f"📊 下一个演示序号: {self.demo_counter}")

    def _get_next_demo_number(self):
        existing_numbers = []
        if os.path.exists(self.save_dir):
            for filename in os.listdir(self.save_dir):
                if filename.startswith('demo_') and filename.endswith('.hdf5'):
                    try:
                        num_str = filename[5:-5]
                        if num_str.isdigit():
                            existing_numbers.append(int(num_str))
                    except ValueError:
                        continue
        if existing_numbers:
            return max(existing_numbers) + 1
        else:
            return 0

    def start_new_demo(self):
        for key in self.current_demo_data:
            self.current_demo_data[key] = []

        # HDF5 文件名使用规范的 "demo_X.hdf5" 格式
        self.hdf5_path = os.path.join(self.save_dir, f"demo_{self.demo_counter}.hdf5")
        self.video_path = os.path.join(self.save_dir, f"demo_{self.demo_counter}_preview.mp4")

        # --- MoviePy 修改 ---
        # 清空视频帧缓存
        self.video_frames = []
        # ---------------------

        self.last_record_time = 0

        print(f"🎬 开始演示 {self.demo_counter} 记录 (HDF5: {self.hdf5_path})")

    def should_record(self, current_time):
        return (current_time - self.last_record_time) >= self.record_interval

    def record_frame(self, env, obs, current_time, action=None):
        """
        记录一帧数据

        Args:
            env: robosuite 环境实例
            obs: 环境返回的观测字典
            current_time: 当前时间 (秒)
            action: 控制器输出的 *相对* 动作 (delta)，如果控制器完成则为 None
        """
        if not self.should_record(current_time):
            return

        try:
            # 1. 获取相机图像 (H, W, C)
            external_cam_img = obs.get('external_cam_image', None)
            external_cam_img = np.flipud(external_cam_img)
            eye_in_hand_img = obs.get('robot0_right_eye_in_hand_image', None)
            eye_in_hand_img = np.flipud(eye_in_hand_img)

            if external_cam_img is None or eye_in_hand_img is None:
                print("⚠️ 相机图像未找到，跳过记录")
                return

            self.current_demo_data['external_cam'].append(external_cam_img)
            self.current_demo_data['robot0_right_eye_in_hand'].append(eye_in_hand_img)

            # 2. 获取机器人状态
            robot = env.robots[0]

            # 2a. 关节位置 (joint_states)
            joint_positions = []
            for joint_name in robot.robot_joints:
                joint_id = env.sim.model.joint_name2id(joint_name)
                qpos_addr = env.sim.model.jnt_qposadr[joint_id]
                joint_positions.append(env.sim.data.qpos[qpos_addr])
            current_joint_positions = np.array(joint_positions)
            self.current_demo_data['joint_states'].append(current_joint_positions)

            # 2b. 夹爪状态 (gripper_states)
            gripper_joint_name = robot.gripper["right"].joints[0]
            gripper_joint_id = env.sim.model.joint_name2id(gripper_joint_name)
            gripper_qpos_addr = env.sim.model.jnt_qposadr[gripper_joint_id]
            gripper_qpos = env.sim.data.qpos[gripper_qpos_addr]
            # 训练格式需要 (T, 2)，我们将单个值复制
            self.current_demo_data['gripper_states'].append(np.array([gripper_qpos, gripper_qpos]))

            # ==================
            # 3. 记录action (目标绝对关节位置)
            # ==================
            if action is not None:
                # action 是相对增量, 我们计算目标绝对位置
                # (假设 action 是 7D 的: 6D 增量 + 1D 夹爪)

                # 简化处理：直接使用action的前6维作为关节增量
                joint_increments = action[:6] * 0.1
                target_joint_positions = current_joint_positions + joint_increments

                # 夹爪目标位置
                gripper_target = action[6]
                target_action = np.append(target_joint_positions, gripper_target)

                self.current_demo_data['actions'].append(target_action)
            else:
                # 如果没有action (控制器返回None, 任务完成)
                # 目标 = 保持当前位置 (即目标绝对位置 = 当前绝对位置)
                gripper_target = gripper_qpos # 保持当前夹爪状态
                target_action = np.append(current_joint_positions, gripper_target)
                self.current_demo_data['actions'].append(target_action)

            # 4. 内部时间戳
            self.current_demo_data['_timestamps'].append(current_time)

            # --- MoviePy 修改 ---
            # 5. 缓存预览视频帧
            # 帧 (external_cam_img) 已经是 (H, W, C) RGB 格式
            # moviepy 喜欢 RGB 格式，这正好
            self.video_frames.append(external_cam_img)
            # ---------------------

            self.last_record_time = current_time

        except Exception as e:
            print(f"❌ 记录数据时出错: {e}")
            import traceback
            traceback.print_exc()

    def save_success_demo(self):
        """保存成功的演示数据到HDF5文件（已修改为训练所需格式）"""
        try:
            if not self.current_demo_data['_timestamps']:
                print("⚠️ 没有数据可保存")
                return False

            # 确保所有数据长度一致
            data_lengths = {k: len(v) for k, v in self.current_demo_data.items()}
            if len(set(data_lengths.values())) > 1:
                print(f"❌ 数据长度不一致: {data_lengths}")
                return False

            T = len(self.current_demo_data['_timestamps'])

            with h5py.File(self.hdf5_path, 'w') as f:
                # 创建根组
                root = f.create_group('root')

                # 1. 保存 Actions: (T, ActionDim)
                actions_data = np.array(self.current_demo_data['actions'])
                root.create_dataset('actions', data=actions_data)

                # 2. 保存 Extra States (低维状态)
                extra_states_group = root.create_group('extra_states')

                # 2a. joint_states: (T, 6)
                joint_data = np.array(self.current_demo_data['joint_states'])
                extra_states_group.create_dataset('joint_states', data=joint_data)

                # 2b. gripper_states: (T, 2)
                gripper_data = np.array(self.current_demo_data['gripper_states'])
                extra_states_group.create_dataset('gripper_states', data=gripper_data)

                # 3. 保存相机数据
                # (视图名称必须与 dataloader 查找的一致)
                # 'external_cam' 对应 'agentview' (通常)
                # 'robot0_right_eye_in_hand' 对应 'eye_in_hand' (通常)
                # 训练脚本 会自动排序, 假设为:
                # view 0: 'external_cam'
                # view 1: 'robot0_right_eye_in_hand'

                # (为了安全起见，我们使用训练脚本期望的键名)
                view_map = {
                    'external_cam': 'agentview',
                    'robot0_right_eye_in_hand': 'eye_in_hand'
                }

                for original_view_name, target_view_name in view_map.items():
                    image_list = self.current_demo_data[original_view_name]
                    if not image_list:
                        print(f"⚠️ 警告: 视图 {original_view_name} 没有图像数据，跳过。")
                        continue

                    images_np = np.array(image_list)

                    # 转换为 (T, C, H, W)
                    images_np_t_c_h_w = np.transpose(images_np, (0, 3, 1, 2))

                    # 转换为 (1, T, C, H, W)
                    images_np_final = np.expand_dims(images_np_t_c_h_w, axis=0)

                    # 使用目标视图名称创建组
                    view_group = root.create_group(target_view_name)
                    view_group.create_dataset('video', data=images_np_final, dtype='u1')

                    # 注意：'tracks' 和 'vis' 缺失。
                    # 您需要稍后运行光流 (e.g. CoTracker) 来填充这些字段。
                    print(f"   (注意: 视图 '{target_view_name}' 缺少 'tracks' 和 'vis' 数据)")


            # --- MoviePy 修改 ---
            # 释放 video_writer (替换为 moviepy 写入)
            try:
                import moviepy.editor as mpy

                if self.video_frames:
                    print(f"  正在使用 moviepy 写入预览视频: {self.video_path}")
                    # 帧率 (fps) 匹配 HDF5 记录频率
                    record_fps = 1.0 / self.record_interval

                    # moviepy 期望 (T, H, W, C) 格式, 且为 RGB
                    # self.video_frames 已经是 [(H, W, C), ...] 的 RGB 图像列表
                    clip = mpy.ImageSequenceClip(self.video_frames, fps=record_fps)

                    # 写入视频文件
                    clip.write_videofile(
                        self.video_path,
                        codec='libx264',  # H.264 编码器
                        audio=False,      # 无音频
                        logger=None,      # 关闭日志 (减少控制台输出)
                        threads=4         # 使用多线程加速
                    )
                    clip.close()
                else:
                    print(f"  (没有视频帧可写入: {self.video_path})")

            except ImportError:
                print("⚠️ moviepy 未安装。跳过视频写入。")
                print("  请运行: pip install moviepy")
            except Exception as e_vid:
                print(f"❌ 使用 moviepy 写入视频时出错: {e_vid}")

            # 清空帧缓存
            self.video_frames = []
            # ---------------------


            demo_start_time = self.current_demo_data['_timestamps'][0]
            demo_end_time = self.current_demo_data['_timestamps'][-1]
            print(f"💾 成功保存演示数据 (已适配训练格式):")
            print(f"   HDF5: {self.hdf5_path}")
            print(f"   视频 (预览用): {self.video_path}")
            print(f"   帧数: {T}")
            print(f"   时长: {demo_end_time - demo_start_time:.2f}秒")
            print(f"   HDF5 结构: root/actions, root/extra_states/..., root/<view_name>/video")

            self.demo_counter += 1
            return True

        except Exception as e:
            print(f"❌ 保存数据时出错: {e}")
            import traceback
            traceback.print_exc()
            return False

    def discard_demo(self):
        # --- MoviePy 修改 ---
        # 清空内存中的视频帧
        self.video_frames = []
        # ---------------------

        for file_path in [self.video_path, self.hdf5_path]:
            if file_path and os.path.exists(file_path):
                try:
                    os.remove(file_path)
                except:
                    pass

        print("🗑️ 已丢弃失败的演示数据")

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
        self.rotation_speed = 0.08  # 增加旋转速度
        self.position_tolerance = 0.01  # 位置容差
        self.orientation_tolerance = 0.25  # 放宽姿态容差，因为姿态控制较慢
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
        initial_ee_ori = self.get_ee_orientation()  # 获取初始姿态

        print(f"🎯 开始规划轨迹:")
        print(f"   当前末端位置: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
        print(f"   当前末端姿态: [{initial_ee_ori[0]:.3f}, {initial_ee_ori[1]:.3f}, {initial_ee_ori[2]:.3f}]")
        print(f"   方块位置: [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")

        # 清空之前的路径点
        self.waypoints = []

        # 定义抓取姿态：夹爪朝下
        # 使用方向向量 [0, 0, -1] 表示Z轴朝下
        grasp_orientation = np.array([0.0, 0.0, -1.0])  # 夹爪朝下

        # 计算目标位置（方块正上方）
        approach_pos = cube_pos.copy()
        approach_pos[2] += self.grasp_height_offset

        # 阶段1a: 先移动到方块正上方，但保持初始姿态（不旋转）
        self.waypoints.append({
            'position': approach_pos,
            'orientation': initial_ee_ori,  # 保持初始姿态
            'gripper': 1.0,  # 打开夹爪
            'phase': 'approach'
        })

        # 阶段1b: 在方块正上方调整姿态为朝下
        self.waypoints.append({
            'position': approach_pos,  # 位置不变，停留在方块上方
            'orientation': grasp_orientation,  # 调整为朝下
            'gripper': 1.0,  # 保持打开
            'phase': 'approach'
        })

        # 阶段2: 下降到抓取位置
        grasp_pos = cube_pos.copy()
        grasp_pos[2] += 0.15  # 稍微高于方块表面
        self.waypoints.append({
            'position': grasp_pos,
            'orientation': grasp_orientation,
            'gripper': 1.0,  # 保持打开
            'phase': 'grasp'
        })

        # 阶段3: 闭合夹爪
        grasp_pos = cube_pos.copy()
        grasp_pos[2] += 0.15
        self.waypoints.append({
            'position': grasp_pos,
            'orientation': grasp_orientation,
            'gripper': -1.0,  # 闭合夹爪
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

        print(f"🗺️ 规划了 {len(self.waypoints)} 个路径点:")
        for i, wp in enumerate(self.waypoints):
            pos = wp['position']
            ori = wp['orientation']
            print(f"   {i+1}. 位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] "
                  f"姿态: [{ori[0]:.2f}, {ori[1]:.2f}, {ori[2]:.2f}] "
                  f"夹爪: {wp['gripper']:.1f} 阶段: {wp['phase']}")

        return True

    def _interpolate_orientation(self, ori_start, ori_end, t):
        """
        在两个方向向量之间进行球面线性插值

        Args:
            ori_start: 起始方向向量
            ori_end: 结束方向向量
            t: 插值参数 [0, 1]

        Returns:
            插值后的方向向量
        """
        # 归一化输入向量
        ori_start_norm = ori_start / (np.linalg.norm(ori_start) + 1e-8)
        ori_end_norm = ori_end / (np.linalg.norm(ori_end) + 1e-8)

        # 计算夹角
        dot = np.clip(np.dot(ori_start_norm, ori_end_norm), -1.0, 1.0)
        theta = np.arccos(dot)

        # 如果夹角很小，使用线性插值
        if theta < 1e-6:
            result = (1 - t) * ori_start_norm + t * ori_end_norm
            return result / (np.linalg.norm(result) + 1e-8)

        # 球面线性插值 (Slerp)
        sin_theta = np.sin(theta)
        w1 = np.sin((1 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta

        result = w1 * ori_start_norm + w2 * ori_end_norm
        return result / (np.linalg.norm(result) + 1e-8)

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
            print("🏁 所有路径点执行完成！机器人已到达方块上方")
            return None

        current_waypoint = self.waypoints[self.current_waypoint_index]
        action, reached = self.get_action_to_waypoint(current_waypoint)

        # 添加超时检测，避免卡死
        if hasattr(self, 'waypoint_start_time'):
            if time.time() - self.waypoint_start_time > 15.0:  # 15秒超时（增加超时时间，因为姿态调整需要更长时间）
                print(f"⚠️  路径点 {self.current_waypoint_index + 1} 超时，强制跳过")
                self.current_waypoint_index += 1
                self.waypoint_start_time = time.time()
                return self.update()
        else:
            self.waypoint_start_time = time.time()

        if reached:
            print(f"✅ 到达路径点 {self.current_waypoint_index + 1}/{len(self.waypoints)} "
                  f"({current_waypoint['phase']})")

            # 检查夹爪状态是否改变
            if self.current_waypoint_index > 0:
                prev_gripper = self.waypoints[self.current_waypoint_index - 1]['gripper']
                curr_gripper = current_waypoint['gripper']
                if prev_gripper != curr_gripper and curr_gripper < 0:
                    # 夹爪即将闭合，标记需要等待
                    print("🤖 开始夹爪闭合...")
                    self.gripper_wait_time = time.time()
                    self.waiting_for_gripper = True

            self.current_waypoint_index += 1
            self.waypoint_start_time = time.time()  # 重置计时器

            # 递归调用获取下一个动作
            return self.update()

        # 如果正在等待夹爪闭合
        if hasattr(self, 'waiting_for_gripper') and self.waiting_for_gripper:
            elapsed = time.time() - self.gripper_wait_time
            if elapsed < 1.0:  # 等待1秒
                # 继续发送当前动作（保持夹爪闭合命令）
                action_dim = self.env.action_dim
                action = np.zeros(action_dim)
                if action_dim >= 7:
                    action[6] = -1.0  # 持续发送闭合命令
                return action
            else:
                # 等待完成
                print("✅ 夹爪闭合完成")
                self.waiting_for_gripper = False

        return action

    def is_complete(self):
        """检查是否完成所有任务"""
        return self.current_waypoint_index >= len(self.waypoints)

def create_arx_environment(headless=False):
    """
    创建ARX5机器人环境

    Args:
        headless (bool): 如果为True, 则在无头模式下运行 (无可视化窗口)
    """

    print(f"🌍 正在创建环境 (Headless: {headless})...")

    # 启用离屏渲染 (has_offscreen_renderer=True)
    # 无论是否为 headless 模式，我们都需要它来获取相机观测数据

    # 启用屏幕渲染 (has_renderer=True) 仅在非 headless 模式下

    env = suite.make(
        env_name="Lift",
        robots="Arx5",
        gripper_types="ArxGripper",
        has_renderer=(not headless),       # <-- 修改点: 仅在非无头时显示窗口
        has_offscreen_renderer=True,       # <-- 保持 True 以获取图像
        use_camera_obs=True,
        camera_names=["external_cam", "robot0_right_eye_in_hand"],
        camera_heights=480,
        camera_widths=640,
        use_object_obs=True,
        control_freq=20,
        horizon=2000,
        reward_shaping=True,
        ignore_done=True,
        hard_reset=True,
        placement_initializer=None,
    )

    print("✅ 环境创建成功")
    print(f"📷 可用相机: {env.camera_names}")
    return env

def collect_demonstration(headless=False):
    """
    收集演示数据

    Args:
        headless (bool): 如果为True, 则在无头模式下运行
    """
    # 创建环境
    env = create_arx_environment(headless=headless) # <-- 修改点: 传入开关

    # 创建数据记录器
    recorder = DataRecorder()

    # 主循环：持续收集演示
    episode_count = 0
    successful_demos = 0

    while True:
        # 重置环境
        obs = env.reset()
        episode_count += 1
        print(f"\n{'='*60}")
        print(f"▶️ 第 {episode_count} 次演示开始")
        print(f"{'='*60}")

        # ... (start_new_demo, controller, 调整机器人位置等... ) ...
        # (这部分逻辑保持不变)

        recorder.start_new_demo()
        demo_start_time = time.time()

        controller = ArxRobotController(env)

        print("🔧 调整机器人初始位置...")
        robot = env.robots[0]
        joint_angles = [0.0, 0, 0, 0, 0.0, 0.0]
        joint_indices = []
        for joint_name in robot.robot_joints:
            joint_id = env.sim.model.joint_name2id(joint_name)
            qpos_addr = env.sim.model.jnt_qposadr[joint_id]
            joint_indices.append(qpos_addr)
        for i, angle in enumerate(joint_angles):
            if i < len(joint_indices):
                env.sim.data.qpos[joint_indices[i]] = angle
        env.sim.forward()
        print("✅ 机器人位置调整完成")

        print("⏳ 等待环境稳定...")
        for _ in range(100):
            env.step(np.zeros(env.action_dim))

        if not controller.plan_trajectory():
            print("❌ 轨迹规划失败")
            recorder.discard_demo()
            continue

        print("\n🚀 开始执行演示...")

        step_count = 0
        max_steps_per_episode = 1500
        success_achieved = False

        while step_count < max_steps_per_episode:
            action = controller.update()

            # (在原始代码中，这里 action 是 None 时被转换了两次,
            #  为保持逻辑一致，我们只在 env.step 中处理 None)

            # 执行动作
            action_to_step = action if action is not None else np.zeros(env.action_dim)
            obs, reward, done, info = env.step(action_to_step)

            # 记录数据（10Hz频率）
            # 传入原始 action (可能为 None)，以便 record_frame 正确处理
            current_time = time.time() - demo_start_time
            recorder.record_frame(env, obs, current_time, action)

            success = env._check_success()
            if success and not success_achieved:
                print(f"🎉 步骤 {step_count}: 任务成功！")
                success_achieved = True
                time.sleep(0.5)
                break

            # --- 修改点: 仅在非无头模式下渲染 ---
            if not headless:
                env.render()
            # ------------------------------------

            step_count += 1

            if step_count % 50 == 0:
                robot = env.robots[0]
                eef_site_id = robot.eef_site_id["right"]
                ee_pos = env.sim.data.site_xpos[eef_site_id]
                cube_pos = env.sim.data.body_xpos[env.cube_body_id]

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

            # --- 修改点: 无头模式下不需要休眠 ---
            if not headless:
                time.sleep(0.005)
            # ------------------------------------

        # ... (演示结束，处理数据...) ...
        # (这部分逻辑保持不变)
        if success_achieved:
            print(f"✅ 第 {episode_count} 次演示成功完成！（{step_count} 步）")
            if recorder.save_success_demo():
                successful_demos += 1
                print(f"📊 已成功收集 {successful_demos} 个演示")
        else:
            print(f"❌ 第 {episode_count} 次演示失败（超过 {max_steps_per_episode} 步）")
            recorder.discard_demo()

        time.sleep(1.0)

if __name__ == "__main__":
    # --- 修改点: 添加 argparse 来解析 --headless 参数 ---
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--headless",
        action="store_true",
        help="运行 robosuite 在无头模式下 (无可视化窗口)"
    )
    args = parser.parse_args()

    collect_demonstration(headless=args.headless)