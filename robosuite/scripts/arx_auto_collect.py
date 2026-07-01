import time
import random
import numpy as np
import h5py
import cv2
import os
import logging
import argparse
import multiprocessing as mp
import sys
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from dataclasses import dataclass

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
LOCAL_ROBOSUITE_DIR = os.path.join(REPO_ROOT, "robosuite")
if LOCAL_ROBOSUITE_DIR not in sys.path:
    sys.path.insert(0, LOCAL_ROBOSUITE_DIR)

import robosuite as suite
from robosuite.controllers import load_composite_controller_config

# 屏蔽 moviepy 的冗余日志
logging.getLogger('moviepy').setLevel(logging.ERROR)

# ========== 固定初始关节角度 ==========
# 前 6 维为机械臂关节，最后 1 维为夹爪初值；link6_initial 只由前 6 维决定。
FIXED_INITIAL_FULL_QPOS = np.array([
    -0.00057220458984375,
    -0.00133514404296875,
    0.00133514404296875,
    -0.01049041748046875,
    0.00019073486328125,
    -0.01697540283203125,
    0.061989784240722656,
])
FIXED_INITIAL_QPOS = FIXED_INITIAL_FULL_QPOS[:6].copy()
FIXED_INITIAL_GRIPPER_QPOS = float(FIXED_INITIAL_FULL_QPOS[6])
LINK6_REFERENCE_BODY = "link6"
LINK6_EEF_BODY = "link6"


def make_pose(pos, rot):
    pose = np.eye(4)
    pose[:3, :3] = rot
    pose[:3, 3] = pos
    return pose


def resolve_body_id(env, short_name):
    prefix = env.robots[0].robot_model.naming_prefix
    for name in (prefix + short_name, short_name):
        try:
            return name, env.sim.model.body_name2id(name)
        except ValueError:
            pass
    raise ValueError(f"Could not find body '{short_name}'")


def get_body_pose(env, short_name):
    name, body_id = resolve_body_id(env, short_name)
    pos = env.sim.data.body_xpos[body_id].copy()
    rot = env.sim.data.body_xmat[body_id].reshape(3, 3).copy()
    return name, make_pose(pos, rot)


def set_initial_gripper_qpos(env):
    robot = env.robots[0]
    applied_values = []
    try:
        gripper = robot.gripper["right"]
    except Exception:
        return np.array(applied_values)

    for joint_name in gripper.joints:
        joint_id = env.sim.model.joint_name2id(joint_name)
        qpos_addr = env.sim.model.jnt_qposadr[joint_id]
        value = FIXED_INITIAL_GRIPPER_QPOS
        if env.sim.model.jnt_limited[joint_id]:
            low, high = env.sim.model.jnt_range[joint_id]
            value = float(np.clip(value, low, high))
        env.sim.data.qpos[qpos_addr] = value
        applied_values.append(value)
    return np.array(applied_values)


def sync_arm_controller_to_current_state(env):
    robot = env.robots[0]
    robot.composite_controller.update_state()
    for arm in robot.arms:
        controller = robot.part_controllers.get(arm)
        if controller is None:
            continue
        if hasattr(controller, "update_initial_joints"):
            controller.update_initial_joints(controller.joint_pos.copy())
        elif hasattr(controller, "reset_goal"):
            controller.reset_goal()

@dataclass
class AutoCollectConfig:
    robot: str = "Arx5"
    env_name: str = "Lift"
    has_renderer: bool = True
    ignore_done: bool = True
    use_camera_obs: bool = True
    control_freq: int = 20
    record_freq: int = 20
    gripper_type: str = "ArxGripper"
    save_dir: str = "demonstrations_test"
    img_size: tuple = (640, 480)
    save_size: tuple = (350, 350)

    # ========== 数据增强噪声配置 ==========
    # Action 噪声 (归一化空间，范围 [-1, 1])
    action_noise_std: float = 0.02  # 位置/旋转噪声标准差
    gripper_noise_prob: float = 0.0  # 夹爪指令翻转概率 (设为 0 禁用)

    # Joint State 噪声 (弧度)
    joint_noise_std: float = 0.01  # 关节角度噪声标准差

    # 摄像头位姿噪声 (很小的值)
    # camera_pos_noise_std: float = 0.01  # 位置噪声标准差 (米)
    # camera_ori_noise_std: float = 0.01  # 姿态噪声标准差 (弧度)

    # 关闭摄像头噪声
    camera_pos_noise_std: float = 0  # 位置噪声标准差 (米)
    camera_ori_noise_std: float = 0  # 姿态噪声标准差 (弧度)

Config = AutoCollectConfig()

class MinJerkTrajectory:
    """
    最小加加速度轨迹规划 (Minimum Jerk Trajectory)
    位置使用5次多项式插值，姿态使用 SLERP 球面插值。
    """
    def __init__(self, start_pos, start_quat, end_pos, end_quat, duration):
        self.start_pos = np.array(start_pos)
        self.end_pos = np.array(end_pos)
        self.start_quat = start_quat
        self.end_quat = end_quat
        self.duration = max(duration, 0.1) # 防止除零

        # 准备 SLERP 插值器
        self.times = [0, self.duration]
        self.key_rots = R.from_quat([start_quat, end_quat])
        self.slerp = Slerp(self.times, self.key_rots)

    def get_pose(self, t):
        if t < 0: t = 0
        if t > self.duration: t = self.duration

        # 位置插值 (5th order polynomial)
        # s(t) = 10(tau)^3 - 15(tau)^4 + 6(tau)^5
        tau = t / self.duration
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5

        current_pos = self.start_pos + (self.end_pos - self.start_pos) * s

        # 姿态插值 (SLERP)
        current_quat = self.slerp([t]).as_quat()[0]

        return current_pos, current_quat

class DataRecorder:
    """
    数据记录器：记录图像、状态和 Action。
    Action label: A_t = S_{t+1}, S = [ee_pos(3), ee_rotvec(3), gripper_qpos(1)].
    ee_positions / ee_orientations 保存为 link6_initial 坐标系下的 link6 位姿。
    """
    def __init__(self):
        self.save_dir = Config.save_dir
        os.makedirs(self.save_dir, exist_ok=True)

        self.current_demo_data = {
            'external_cam': [],
            'robot0_right_eye_in_hand': [],
            'joint_states': [],
            'gripper_states': [],
            'osc_controller_inputs': [],
            'ee_positions': [],
            'ee_orientations': [],
            '_timestamps': []
        }

        self.video_frames = []
        self.record_interval = 1.0 / Config.record_freq
        self.last_record_time = -1.0

        # 用于计算 Delta Action 的上一帧状态
        self.prev_ee_position = None
        self.prev_ee_rotation = None

        # 记录方块初始位置
        self.initial_cube_pos = None
        self.link6_initial_body_name = None
        self.ee_body_name = None
        self.world_t_link6_initial = None
        self.link6_initial_t_world = None
        self.applied_initial_gripper_qpos = None

    def set_initial_cube_pos(self, pos):
        """设置方块初始位置（用于保存到 txt）"""
        self.initial_cube_pos = pos.copy()

    def start_new_demo(self):
        for key in self.current_demo_data:
            self.current_demo_data[key] = []
        self.video_frames = []
        self.last_record_time = -1.0
        self.prev_ee_position = None
        self.prev_ee_rotation = None
        self.initial_cube_pos = None
        self.link6_initial_body_name = None
        self.ee_body_name = None
        self.world_t_link6_initial = None
        self.link6_initial_t_world = None
        self.applied_initial_gripper_qpos = None

    def should_record(self, current_time):
        if self.last_record_time < 0: return True
        return (current_time - self.last_record_time) >= self.record_interval

    def set_link6_initial_reference(self, env):
        self.link6_initial_body_name, self.world_t_link6_initial = get_body_pose(env, LINK6_REFERENCE_BODY)
        self.link6_initial_t_world = np.linalg.inv(self.world_t_link6_initial)

    def get_ee_pose(self, env):
        """Return link6 pose expressed in the link6_initial frame."""
        if self.link6_initial_t_world is None:
            self.set_link6_initial_reference(env)

        self.ee_body_name, world_t_eef = get_body_pose(env, LINK6_EEF_BODY)
        link6_initial_t_eef = self.link6_initial_t_world @ world_t_eef

        return link6_initial_t_eef[:3, 3], link6_initial_t_eef[:3, :3]

    def record_frame(self, env, obs, current_time, action=None):
        """
        记录一帧数据。

        参数:
            action: 控制器发送给 env.step() 的 OSC 输入 (7维)，只保存到
                    /root/extra_states/osc_controller_inputs 便于回放调试。
                    /root/actions 会在保存时由下一帧修复后的状态 S 生成。
        """
        if not self.should_record(current_time):
            return

        try:
            # --- 1. 处理图像 ---
            raw_ext_img = obs.get('external_cam_image', None)
            raw_hand_img = obs.get('robot0_right_eye_in_hand_image', None)

            if raw_ext_img is None or raw_hand_img is None:
                return

            # 翻转图像 (Robosuite 渲染特性)
            raw_ext_img = np.flipud(raw_ext_img)
            raw_hand_img = np.flipud(raw_hand_img)

            target_h, target_w = Config.save_size
            orig_h, orig_w, _ = raw_ext_img.shape

            # 外部相机裁剪逻辑
            right_margin = 30
            if orig_h >= target_h and orig_w >= (target_w + right_margin):
                crop_y_start = 0
                crop_y_end = target_h
                crop_x_end = orig_w - right_margin
                crop_x_start = crop_x_end - target_w
                ext_img_processed = raw_ext_img[crop_y_start:crop_y_end, crop_x_start:crop_x_end]
            else:
                ext_img_processed = cv2.resize(raw_ext_img, (target_w, target_h), interpolation=cv2.INTER_AREA)

            hand_img_processed = cv2.resize(raw_hand_img, (target_w, target_h), interpolation=cv2.INTER_AREA)

            self.current_demo_data['external_cam'].append(ext_img_processed)
            self.current_demo_data['robot0_right_eye_in_hand'].append(hand_img_processed)
            self.video_frames.append(ext_img_processed)

            # --- 2. 机器人状态 ---
            robot = env.robots[0]
            joint_positions = []
            for joint_name in robot.robot_joints:
                joint_id = env.sim.model.joint_name2id(joint_name)
                qpos_addr = env.sim.model.jnt_qposadr[joint_id]
                joint_positions.append(env.sim.data.qpos[qpos_addr])

            # 添加 Joint State 噪声 (数据增强)
            joint_positions = np.array(joint_positions)
            if Config.joint_noise_std > 0:
                joint_noise = np.random.normal(0, Config.joint_noise_std, len(joint_positions))
                joint_positions = joint_positions + joint_noise

            self.current_demo_data['joint_states'].append(joint_positions)

            # --- 3. 夹爪状态 ---
            try:
                gripper_joint_name = robot.gripper["right"].joints[0]
                gripper_joint_id = env.sim.model.joint_name2id(gripper_joint_name)
                gripper_qpos_addr = env.sim.model.jnt_qposadr[gripper_joint_id]
                gripper_qpos = env.sim.data.qpos[gripper_qpos_addr]
            except:
                gripper_qpos = 0.0

            self.current_demo_data['gripper_states'].append(np.array([gripper_qpos, gripper_qpos]))

            # --- 4. 末端执行器位姿 ---
            ee_position, ee_rotation = self.get_ee_pose(env)
            self.current_demo_data['ee_positions'].append(ee_position)
            self.current_demo_data['ee_orientations'].append(ee_rotation.flatten())

            # --- 5. 保存实际送入环境的 OSC 输入；训练 action label 在保存时生成 ---
            if action is not None:
                self.current_demo_data['osc_controller_inputs'].append(action.copy())
            else:
                self.current_demo_data['osc_controller_inputs'].append(np.zeros(7))

            self.current_demo_data['_timestamps'].append(current_time)
            self.last_record_time = current_time

        except Exception as e:
            print(f"Error in record_frame: {e}")

    def build_next_state_actions(self):
        """
        Build action labels from the corrected robot state.

        A_t = S_{t+1}, where S_t = [ee_xyz(3), ee_rotvec(3), gripper_qpos(1)].
        The final action repeats the penultimate action, so for T frames:
            actions[:-1] = states[1:]
            actions[-1] = actions[-2]
        """
        ee_positions = np.array(self.current_demo_data['ee_positions'])
        ee_orientations = np.array(self.current_demo_data['ee_orientations']).reshape(-1, 3, 3)
        gripper_states = np.array(self.current_demo_data['gripper_states'])

        if len(ee_positions) == 0:
            return np.zeros((0, 7))

        ee_rotvecs = R.from_matrix(ee_orientations).as_rotvec()
        gripper_qpos = gripper_states[:, :1] if gripper_states.ndim == 2 else gripper_states.reshape(-1, 1)
        states = np.concatenate([ee_positions, ee_rotvecs, gripper_qpos], axis=1)

        actions = np.empty_like(states)
        if len(states) == 1:
            actions[0] = states[0]
        else:
            actions[:-1] = states[1:]
            actions[-1] = actions[-2]
        return actions

    def save_success_demo(self, demo_index):
        """保存数据到 HDF5"""
        if not self.current_demo_data['_timestamps']:
            print("No data to save.")
            return False

        hdf5_path = os.path.join(self.save_dir, f"demo_{demo_index}.hdf5")
        video_path = os.path.join(self.save_dir, f"demo_{demo_index}.mp4")

        try:
            with h5py.File(hdf5_path, 'w') as f:
                root = f.create_group('root')
                if self.initial_cube_pos is not None:
                    root.attrs["cube_init_pos"] = self.initial_cube_pos
                root.attrs["fixed_initial_arm_qpos"] = FIXED_INITIAL_QPOS
                root.attrs["fixed_initial_full_qpos"] = FIXED_INITIAL_FULL_QPOS
                root.attrs["requested_initial_gripper_qpos"] = FIXED_INITIAL_GRIPPER_QPOS
                if self.applied_initial_gripper_qpos is not None:
                    root.attrs["applied_initial_gripper_qpos"] = self.applied_initial_gripper_qpos
                root.attrs["action_label_type"] = "next_state"
                root.attrs["action_label_convention"] = "A_t = S_{t+1}; final action repeats penultimate action"
                root.attrs["action_state_layout"] = "ee_x,ee_y,ee_z,ee_rotvec_x,ee_rotvec_y,ee_rotvec_z,gripper_qpos"
                root.attrs["action_state_frame"] = "link6_initial"
                if self.link6_initial_body_name is not None:
                    root.attrs["action_state_reference_body_name"] = self.link6_initial_body_name
                if self.ee_body_name is not None:
                    root.attrs["action_state_eef_body_name"] = self.ee_body_name

                # 1. 保存 Action Label: A_t = S_{t+1}
                actions_data = self.build_next_state_actions()
                root.create_dataset('actions', data=actions_data)

                # 2. 保存其他状态
                extra_group = root.create_group('extra_states')
                extra_group.attrs["ee_frame"] = "link6_initial"
                extra_group.attrs["ee_pose_convention"] = "T_link6_initial_link6"
                extra_group.attrs["reference_body"] = LINK6_REFERENCE_BODY
                extra_group.attrs["eef_body"] = LINK6_EEF_BODY
                if self.link6_initial_body_name is not None:
                    extra_group.attrs["reference_body_name"] = self.link6_initial_body_name
                if self.ee_body_name is not None:
                    extra_group.attrs["eef_body_name"] = self.ee_body_name
                if self.world_t_link6_initial is not None:
                    extra_group.attrs["T_world_link6_initial"] = self.world_t_link6_initial
                extra_group.create_dataset('joint_states', data=np.array(self.current_demo_data['joint_states']))
                extra_group.create_dataset('gripper_states', data=np.array(self.current_demo_data['gripper_states']))
                extra_group.create_dataset('ee_positions', data=np.array(self.current_demo_data['ee_positions']))
                extra_group.create_dataset('ee_orientations', data=np.array(self.current_demo_data['ee_orientations']))
                extra_group.create_dataset(
                    'osc_controller_inputs',
                    data=np.array(self.current_demo_data['osc_controller_inputs'])
                )

                # 3. 保存图像 (格式调整为 N, C, H, W 以兼容常用训练库)
                view_map = {
                    'external_cam': 'agentview',
                    'robot0_right_eye_in_hand': 'eye_in_hand'
                }
                for k, v in view_map.items():
                    imgs = np.array(self.current_demo_data[k])
                    if len(imgs) > 0:
                        # (N, H, W, C) -> (1, N, C, H, W)
                        imgs_t = np.transpose(imgs, (0, 3, 1, 2))
                        imgs_final = np.expand_dims(imgs_t, axis=0)
                        view_group = root.create_group(v)
                        view_group.create_dataset('video', data=imgs_final, dtype='u1')

            # 4. 保存视频预览
            if self.video_frames:
                try:
                    import moviepy.editor as mpy
                    clip = mpy.ImageSequenceClip(self.video_frames, fps=Config.record_freq)
                    clip.write_videofile(video_path, codec='libx264', audio=False, verbose=False, logger=None)
                except Exception as e:
                    print(f"Video save error (ignored): {e}")

            # 5. 保存方块初始位置到 txt 文件
            if self.initial_cube_pos is not None:
                cube_pos_path = os.path.join(self.save_dir, f"demo_{demo_index}_cube_pos.txt")
                np.savetxt(cube_pos_path, self.initial_cube_pos, fmt='%.6f')
                print(f"   Cube position saved to: {cube_pos_path}")

            return True
        except Exception as e:
            print(f"HDF5 Save failed: {e}")
            return False

class ArxRobotController:
    """
    机器人控制器：负责规划路径并计算控制指令。
    使用零位姿态（Unit Quaternion）作为向下抓取的姿态。
    支持失败重试机制。
    """
    def __init__(self, env, max_retries=3):
        self.env = env
        self.trajectories = []
        self.current_traj_idx = 0
        self.traj_start_time = 0.0
        self.grasp_ori_quat = np.array([0.0, 0.0, 0.0, 1.0]) # 默认零位 (朝下)

        # 重试机制
        self.max_retries = max_retries
        self.retry_count = 0
        self.is_lifting = False  # 标记是否在抬起阶段
        self.lift_start_time = 0.0

    def get_ee_pose(self):
        robot = self.env.robots[0]
        eef_id = robot.eef_site_id["right"]
        pos = self.env.sim.data.site_xpos[eef_id].copy()
        mat = self.env.sim.data.site_xmat[eef_id].reshape(3,3).copy()
        quat = R.from_matrix(mat).as_quat()
        return pos, quat

    def get_cube_pose(self):
        # 稳健地获取方块 ID
        try:
            cid = self.env.cube_body_id
        except:
            cid = self.env.sim.model.body_name2id("cube_main")

        pos = self.env.sim.data.body_xpos[cid].copy()
        quat = self.grasp_ori_quat
        return pos, quat

    def plan_task(self):
        """规划 Approach -> Descend -> Grasp -> Lift"""
        self.trajectories = []
        ee_pos, ee_quat = self.get_ee_pose()
        cube_pos, _ = self.get_cube_pose()

        # 始终保持机器人当前的自然姿态 (即零位朝下)
        task_quat = self.grasp_ori_quat

        # 1. Approach: 移动到方块上方 20cm
        hover_pos = cube_pos.copy()
        hover_pos[2] += 0.20

        dist = np.linalg.norm(hover_pos - ee_pos)
        duration = max(dist / 0.3, 2.0)

        traj_approach = MinJerkTrajectory(ee_pos, ee_quat, hover_pos, task_quat, duration)
        self.trajectories.append({'traj': traj_approach, 'gripper': 1.0, 'is_pause': False})

        # 2. Descend: 下降到抓取位置
        grasp_pos = cube_pos.copy()

        # [修改] 之前是 +0.03，现在改为 +0.06
        # 原因：防止夹爪手指太长导致碰撞桌面
        # 你可以根据实际情况微调：0.05 ~ 0.08
        grasp_pos[2] = cube_pos[2] + 0.15

        duration = 1.0
        traj_descend = MinJerkTrajectory(hover_pos, task_quat, grasp_pos, task_quat, duration)
        self.trajectories.append({'traj': traj_descend, 'gripper': 1.0, 'is_pause': False})

        # 3. Grasp: 保持位置，闭合夹爪
        self.trajectories.append({
            'traj': None,
            'fixed_pos': grasp_pos,
            'fixed_quat': task_quat,
            'gripper': -1.0,
            'is_pause': True,
            'duration': 0.8
        })

        # 4. Lift: 抬起
        lift_pos = grasp_pos.copy()
        lift_pos[2] += 0.30
        duration = 1.5
        traj_lift = MinJerkTrajectory(grasp_pos, task_quat, lift_pos, task_quat, duration)
        self.trajectories.append({'traj': traj_lift, 'gripper': -1.0, 'is_pause': False})

        self.current_traj_idx = 0
        self.traj_start_time = 0.0
        self.lift_traj_idx = len(self.trajectories) - 1  # 记录抬起阶段的索引
        self.is_lifting = False
        return True

    def check_grasp_success(self):
        """
        检查是否成功抓取了方块。
        通过比较方块位置与未抬起时的高度差异来判断。
        """
        cube_pos, _ = self.get_cube_pose()
        # 如果方块的 Z 坐标比初始高度高出 5cm，认为抓取成功
        initial_cube_z = 0.8225  # 桥面高度 + 方块半高
        return cube_pos[2] > initial_cube_z + 0.05

    def plan_retry(self, current_sim_time):
        """
        规划重试轨迹：松开夹爪 -> 抬起 -> 重新规划抓取
        返回 True 表示成功规划重试，False 表示已达到最大重试次数
        """
        if self.retry_count >= self.max_retries:
            print(f"   ⚠️ 已达到最大重试次数 ({self.max_retries})")
            return False

        self.retry_count += 1
        print(f"   🔄 第 {self.retry_count} 次重试...")

        self.trajectories = []
        ee_pos, ee_quat = self.get_ee_pose()
        cube_pos, _ = self.get_cube_pose()
        task_quat = self.grasp_ori_quat

        # 1. Release: 松开夹爪 (保持当前位置)
        self.trajectories.append({
            'traj': None,
            'fixed_pos': ee_pos,
            'fixed_quat': ee_quat,
            'gripper': 1.0,  # 打开夹爪
            'is_pause': True,
            'duration': 0.5
        })

        # 2. Retreat: 稍微抬起以避免碰撞
        retreat_pos = ee_pos.copy()
        retreat_pos[2] += 0.10
        duration = 0.8
        traj_retreat = MinJerkTrajectory(ee_pos, ee_quat, retreat_pos, task_quat, duration)
        self.trajectories.append({'traj': traj_retreat, 'gripper': 1.0, 'is_pause': False})

        # 3. Re-approach: 移动到方块上方 (重新获取方块位置，因为可能已经移动)
        hover_pos = cube_pos.copy()
        hover_pos[2] += 0.15
        duration = 1.0
        traj_approach = MinJerkTrajectory(retreat_pos, task_quat, hover_pos, task_quat, duration)
        self.trajectories.append({'traj': traj_approach, 'gripper': 1.0, 'is_pause': False})

        # 4. Re-descend: 下降到抓取位置
        grasp_pos = cube_pos.copy()
        grasp_pos[2] = cube_pos[2] + 0.15
        duration = 0.8
        traj_descend = MinJerkTrajectory(hover_pos, task_quat, grasp_pos, task_quat, duration)
        self.trajectories.append({'traj': traj_descend, 'gripper': 1.0, 'is_pause': False})

        # 5. Re-grasp: 闭合夹爪
        self.trajectories.append({
            'traj': None,
            'fixed_pos': grasp_pos,
            'fixed_quat': task_quat,
            'gripper': -1.0,
            'is_pause': True,
            'duration': 0.8
        })

        # 6. Re-lift: 抬起
        lift_pos = grasp_pos.copy()
        lift_pos[2] += 0.30
        duration = 1.5
        traj_lift = MinJerkTrajectory(grasp_pos, task_quat, lift_pos, task_quat, duration)
        self.trajectories.append({'traj': traj_lift, 'gripper': -1.0, 'is_pause': False})

        self.current_traj_idx = 0
        self.traj_start_time = current_sim_time
        self.lift_traj_idx = len(self.trajectories) - 1
        self.is_lifting = False
        return True

    def get_action(self, current_sim_time):
        if self.current_traj_idx >= len(self.trajectories):
            return None # 结束

        step_data = self.trajectories[self.current_traj_idx]
        elapsed = current_sim_time - self.traj_start_time

        # 确定当前段的持续时间
        if step_data['is_pause']:
            seg_duration = step_data['duration']
        else:
            seg_duration = step_data['traj'].duration

        # 检查是否切换下一段
        if elapsed >= seg_duration:
            self.current_traj_idx += 1
            self.traj_start_time = current_sim_time
            return self.get_action(current_sim_time)

        # 获取目标位姿
        if step_data['is_pause']:
            target_pos = step_data['fixed_pos']
            target_quat = step_data['fixed_quat']
        else:
            target_pos, target_quat = step_data['traj'].get_pose(elapsed)

        # 计算误差
        current_pos, current_quat = self.get_ee_pose()
        pos_err = target_pos - current_pos

        r_curr = R.from_quat(current_quat)
        r_targ = R.from_quat(target_quat)
        r_diff = r_targ * r_curr.inv()
        rot_err = r_diff.as_rotvec()

        # 简单的 P 控制器增益
        kp_pos = 50.0
        kp_rot = 15.0

        # [关键修改] 夹爪闭合时，机械臂保持静止
        # 在 is_pause 阶段，位置和旋转输出为零，只发送夹爪指令
        if step_data['is_pause']:
            d_pos = np.zeros(3)
            d_rot = np.zeros(3)
        else:
            d_pos = np.clip(pos_err * kp_pos, -1.0, 1.0)
            d_rot = np.clip(rot_err * kp_rot, -1.0, 1.0)

        # 添加 Action 噪声 (数据增强)
        if Config.action_noise_std > 0 and not step_data['is_pause']:
            pos_noise = np.random.normal(0, Config.action_noise_std, 3)
            rot_noise = np.random.normal(0, Config.action_noise_std, 3)
            d_pos = np.clip(d_pos + pos_noise, -1.0, 1.0)
            d_rot = np.clip(d_rot + rot_noise, -1.0, 1.0)

        # 返回用于 env.step 的 action
        action = np.concatenate([d_pos, d_rot, [step_data['gripper']]])
        return action


def create_env(headless=True):
    config = load_composite_controller_config(robot="arx5")
    if "body_parts" in config:
        for name, part_config in config["body_parts"].items():
            if "gripper" in name:
                part_config["type"] = "JOINT_POSITION"
                part_config["input_type"] = "binary"

    env = suite.make(
        env_name="Lift",
        robots="Arx5",
        gripper_types="ArxGripper",
        controller_configs=config,
        has_renderer=(not headless),
        has_offscreen_renderer=True,
        use_camera_obs=True,
        camera_names=["external_cam", "robot0_right_eye_in_hand"],
        camera_heights=480,
        camera_widths=640,
        control_freq=Config.control_freq,
        horizon=2000,
        ignore_done=True,
        hard_reset=True
    )
    return env

def randomize_camera_pose(env):
    """
    为摄像头位姿添加微量噪声。
    在原始 XML 定义的位置附近随机初始化。
    """
    # 获取摄像头 ID
    for cam_name in ["external_cam", "robot0_right_eye_in_hand"]:
        try:
            cam_id = env.sim.model.camera_name2id(cam_name)

            # 添加位置噪声 (xyz)
            pos_noise = np.random.normal(0, Config.camera_pos_noise_std, 3)
            env.sim.model.cam_pos[cam_id] += pos_noise

            # 添加姿态噪声 (四元数微调)
            # 将小角度噪声转换为四元数扰动
            angle_noise = np.random.normal(0, Config.camera_ori_noise_std, 3)
            # 使用旋转向量转换为四元数
            rot_noise = R.from_rotvec(angle_noise)
            current_quat = env.sim.model.cam_quat[cam_id].copy()
            # MuJoCo 使用 (w, x, y, z) 格式
            current_rot = R.from_quat([current_quat[1], current_quat[2], current_quat[3], current_quat[0]])
            new_rot = rot_noise * current_rot
            new_quat_xyzw = new_rot.as_quat()
            # 转换回 MuJoCo 格式 (w, x, y, z)
            env.sim.model.cam_quat[cam_id] = [new_quat_xyzw[3], new_quat_xyzw[0], new_quat_xyzw[1], new_quat_xyzw[2]]

        except Exception as e:
            # 如果找不到摄像头，跳过
            pass

def worker_collect(worker_id, shared_counter, lock, target_demos, headless):
    env = create_env(headless)
    recorder = DataRecorder()
    controller = ArxRobotController(env)

    print(f"[Worker {worker_id}] Started.")
    dt = 1.0 / Config.control_freq

    while True:
        with lock:
            if shared_counter.value >= target_demos: break

        obs = env.reset()
        recorder.start_new_demo()

        # 为摄像头位姿添加微量噪声 (每个 episode 随机初始化)
        randomize_camera_pose(env)

        # 强制设置机器人到固定初始姿态
        robot = env.robots[0]
        j_start = robot.joint_indexes[0]
        j_end = robot.joint_indexes[-1] + 1
        env.sim.data.qpos[j_start:j_end] = FIXED_INITIAL_QPOS
        recorder.applied_initial_gripper_qpos = set_initial_gripper_qpos(env)
        env.sim.forward()
        sync_arm_controller_to_current_state(env)
        recorder.set_link6_initial_reference(env)

        # 记录方块初始位置
        cube_pos = env.sim.data.body_xpos[env.cube_body_id].copy()
        recorder.set_initial_cube_pos(cube_pos)

        # 归位/稳定
        for _ in range(20): env.step(np.zeros(7))

        if not controller.plan_task():
            continue

        sim_time = 0.0
        controller.traj_start_time = sim_time
        controller.retry_count = 0  # 重置重试计数

        for i in range(2000):  # 增加最大步数以容纳重试
            action = controller.get_action(sim_time)

            if action is None:  # 轨迹结束
                # 检查是否成功
                if controller.check_grasp_success():
                    # 成功，但让 env._check_success() 来最终确认
                    break
                else:
                    # 失败，尝试重试
                    if controller.plan_retry(sim_time):
                        continue  # 继续执行重试轨迹
                    else:
                        # 达到最大重试次数，放弃这次 demo
                        break

            obs, reward, done_env, info = env.step(action)
            sim_time += dt

            # 记录数据 (传入控制指令)
            recorder.record_frame(env, obs, sim_time, action=action)

            # 在抬起阶段检查是否失败
            if controller.current_traj_idx == controller.lift_traj_idx:
                if not controller.is_lifting:
                    controller.is_lifting = True
                    controller.lift_start_time = sim_time

                # 抬起 0.5 秒后检查方块是否跟随
                if sim_time - controller.lift_start_time > 0.5:
                    if not controller.check_grasp_success():
                        print(f"[Worker {worker_id}] 抓取失败，方块未被抬起")
                        if controller.plan_retry(sim_time):
                            continue
                        else:
                            break

            # 检查成功 (Robosuite 内部判定)
            if env._check_success():
                with lock:
                    if shared_counter.value < target_demos:
                        idx = shared_counter.value
                        shared_counter.value += 1
                        if controller.retry_count > 0:
                            print(f"[Worker {worker_id}] SUCCESS after {controller.retry_count} retries! Saving demo {idx}...")
                        else:
                            print(f"[Worker {worker_id}] SUCCESS! Cube successfully lifted. Saving demo {idx}...")
                        recorder.save_success_demo(idx)
                    break

    env.close()

def run_main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--num_demos", type=int, default=50)
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument("--save_dir", type=str, default="demonstration")
    args = parser.parse_args()
    Config.save_dir = args.save_dir

    if args.workers > 1:
        mp.set_start_method('spawn', force=True)
        manager = mp.Manager()
        counter = manager.Value('i', 0)
        lock = manager.Lock()
        procs = []
        for i in range(args.workers):
            p = mp.Process(target=worker_collect, args=(i, counter, lock, args.num_demos, args.headless))
            p.start()
            procs.append(p)
        for p in procs: p.join()
    else:
        # 单进程模式
        class MockVal: value = 0
        worker_collect(0, MockVal(), mp.Lock(), args.num_demos, args.headless)

if __name__ == "__main__":
    run_main()
