import argparse
import logging
import multiprocessing as mp
import os
import sys
from dataclasses import dataclass, replace
from datetime import datetime

import cv2
import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PACKAGE_ROOT)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import robosuite as suite
from robosuite.controllers import load_composite_controller_config

logging.getLogger("moviepy").setLevel(logging.ERROR)


FIXED_INITIAL_FULL_QPOS = np.array(
    [
        -0.00057220458984375,
        -0.00133514404296875,
        0.00133514404296875,
        -0.01049041748046875,
        0.00019073486328125,
        -0.01697540283203125,
        0.061989784240722656,
    ]
)
FIXED_INITIAL_QPOS = FIXED_INITIAL_FULL_QPOS[:6].copy()
FIXED_INITIAL_GRIPPER_QPOS = float(FIXED_INITIAL_FULL_QPOS[6])

LINK6_REFERENCE_BODY = "link6"
LINK6_EEF_BODY = "link6"
GRASP_FRAME_OFFSET_POS = np.array([0.145, 0.0, 0.0])
GRASP_FRAME_OFFSET_ROT = np.eye(3)
DOWNWARD_GRASP_ROT = R.from_euler("y", np.pi / 2.0).as_matrix()
OPEN_GRIPPER = 1.0
CLOSE_GRIPPER = -1.0


@dataclass
class LemonEEFCollectConfig:
    robot: str = "Arx5"
    env_name: str = "Lemon"
    gripper_type: str = "ArxGripper"
    control_freq: int = 20
    record_freq: int = 20
    save_dir: str = "lemon_eef_pose_demonstrations"
    img_size: tuple = (640, 480)
    save_size: tuple = (350, 350)
    num_demos: int = 50
    workers: int = 1
    headless: bool = True
    max_steps: int = 700
    settle_steps: int = 20
    no_video: bool = False
    use_camera_obs: bool = True
    max_episodes: int = 0
    seed: int = -1
    eef_kp: float = 150.0
    motion_speed: float = 0.12
    grasp_yaw_offset: float = 0.0
    lift_yaw: float = 0.0
    lift_height: float = 0.16
    place_hover_height: float = 0.16
    success_stable_steps: int = 20
    lock_lemon_until_grasp: bool = True
    camera_pos_noise_std: float = 0.0
    camera_ori_noise_std: float = 0.0


def make_pose(pos, rot):
    pose = np.eye(4)
    pose[:3, :3] = rot
    pose[:3, 3] = pos
    return pose


def offset_pose(pos=GRASP_FRAME_OFFSET_POS, rot=GRASP_FRAME_OFFSET_ROT):
    return make_pose(np.asarray(pos, dtype=np.float64), np.asarray(rot, dtype=np.float64))


T_LINK6_GRASP = offset_pose()
T_GRASP_LINK6 = np.linalg.inv(T_LINK6_GRASP)


def yaw_from_pose(world_t_object):
    x_axis = world_t_object[:3, 0]
    return float(np.arctan2(x_axis[1], x_axis[0]))


def wrap_to_pi(angle):
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def fold_yaw_to_half_turn(angle):
    angle = wrap_to_pi(angle)
    if angle > np.pi / 2:
        angle -= np.pi
    elif angle < -np.pi / 2:
        angle += np.pi
    return float(angle)


def grasp_yaw_from_object(world_t_object, yaw_offset=0.0):
    return fold_yaw_to_half_turn(yaw_from_pose(world_t_object) + yaw_offset)


def downward_grasp_rotation(yaw):
    return R.from_euler("z", yaw).as_matrix() @ DOWNWARD_GRASP_ROT


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


def get_body_pose_by_id(env, body_id):
    pos = env.sim.data.body_xpos[body_id].copy()
    rot = env.sim.data.body_xmat[body_id].reshape(3, 3).copy()
    return make_pose(pos, rot)


def pose_to_action_in_reference(world_t_reference_initial, world_t_link6):
    reference_initial_t_world = np.linalg.inv(world_t_reference_initial)
    reference_t_link6 = reference_initial_t_world @ world_t_link6
    rotvec = R.from_matrix(reference_t_link6[:3, :3]).as_rotvec()
    return np.concatenate([reference_t_link6[:3, 3], rotvec]).astype(np.float64)


def action_to_world_pose(world_t_reference_initial, action6):
    reference_t_link6 = make_pose(action6[:3], R.from_rotvec(action6[3:6]).as_matrix())
    return world_t_reference_initial @ reference_t_link6


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
        if hasattr(controller, "reset_goal"):
            controller.reset_goal()


class MinJerkPoseTrajectory:
    def __init__(self, start_pose, end_pose, duration):
        self.start_pos = np.array(start_pose[:3, 3], dtype=np.float64)
        self.end_pos = np.array(end_pose[:3, 3], dtype=np.float64)
        self.duration = max(float(duration), 0.1)
        self.times = [0.0, self.duration]
        key_rots = R.from_matrix([start_pose[:3, :3], end_pose[:3, :3]])
        self.slerp = Slerp(self.times, key_rots)

    def get_pose(self, t):
        t = float(np.clip(t, 0.0, self.duration))
        tau = t / self.duration
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        pos = self.start_pos + (self.end_pos - self.start_pos) * s
        rot = self.slerp([t]).as_matrix()[0]
        return make_pose(pos, rot)


class LemonDataRecorder:
    def __init__(self, cfg):
        self.cfg = cfg
        os.makedirs(self.cfg.save_dir, exist_ok=True)
        self.record_interval = 1.0 / self.cfg.record_freq
        self.start_new_demo()

    def start_new_demo(self):
        self.data = {
            "external_cam": [],
            "robot0_right_eye_in_hand": [],
            "joint_states": [],
            "gripper_states": [],
            "ee_positions": [],
            "ee_orientations": [],
            "eef_pose_controller_inputs": [],
            "lemon_positions": [],
            "lemon_quats": [],
            "plate_positions": [],
            "plate_quats": [],
            "_timestamps": [],
        }
        self.video_frames = []
        self.last_record_time = -1.0
        self.world_t_link6_initial = None
        self.link6_initial_t_world = None
        self.link6_initial_body_name = None
        self.ee_body_name = None
        self.applied_initial_gripper_qpos = None
        self.initial_lemon_pose = None
        self.initial_plate_pose = None

    def should_record(self, sim_time):
        if self.last_record_time < 0:
            return True
        return sim_time - self.last_record_time >= self.record_interval

    def set_link6_initial_reference(self, env):
        self.link6_initial_body_name, self.world_t_link6_initial = get_body_pose(env, LINK6_REFERENCE_BODY)
        self.link6_initial_t_world = np.linalg.inv(self.world_t_link6_initial)

    def get_link6_pose_in_reference(self, env):
        if self.link6_initial_t_world is None:
            self.set_link6_initial_reference(env)
        self.ee_body_name, world_t_link6 = get_body_pose(env, LINK6_EEF_BODY)
        reference_t_link6 = self.link6_initial_t_world @ world_t_link6
        return reference_t_link6

    @staticmethod
    def _body_quat_xyzw(env, body_id):
        quat_wxyz = env.sim.data.body_xquat[body_id].copy()
        return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

    def record_frame(self, env, obs, sim_time, action):
        if not self.should_record(sim_time):
            return

        raw_ext_img = obs.get("external_cam_image", None)
        raw_hand_img = obs.get("robot0_right_eye_in_hand_image", None)
        if raw_ext_img is not None and raw_hand_img is not None:
            raw_ext_img = np.flipud(raw_ext_img)
            raw_hand_img = np.flipud(raw_hand_img)
            target_h, target_w = self.cfg.save_size
            orig_h, orig_w, _ = raw_ext_img.shape

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

            self.data["external_cam"].append(ext_img_processed)
            self.data["robot0_right_eye_in_hand"].append(hand_img_processed)
            self.video_frames.append(ext_img_processed)

        robot = env.robots[0]
        joint_positions = []
        for joint_name in robot.robot_joints:
            joint_id = env.sim.model.joint_name2id(joint_name)
            qpos_addr = env.sim.model.jnt_qposadr[joint_id]
            joint_positions.append(env.sim.data.qpos[qpos_addr])
        self.data["joint_states"].append(np.array(joint_positions))

        try:
            gripper_joint_name = robot.gripper["right"].joints[0]
            gripper_joint_id = env.sim.model.joint_name2id(gripper_joint_name)
            gripper_qpos_addr = env.sim.model.jnt_qposadr[gripper_joint_id]
            gripper_qpos = env.sim.data.qpos[gripper_qpos_addr]
        except Exception:
            gripper_qpos = 0.0
        self.data["gripper_states"].append(np.array([gripper_qpos, gripper_qpos]))

        reference_t_link6 = self.get_link6_pose_in_reference(env)
        self.data["ee_positions"].append(reference_t_link6[:3, 3])
        self.data["ee_orientations"].append(reference_t_link6[:3, :3].flatten())
        self.data["eef_pose_controller_inputs"].append(action.copy())

        self.data["lemon_positions"].append(env.sim.data.body_xpos[env.lemon_body_id].copy())
        self.data["lemon_quats"].append(self._body_quat_xyzw(env, env.lemon_body_id))
        self.data["plate_positions"].append(env.sim.data.body_xpos[env.plate_body_id].copy())
        self.data["plate_quats"].append(self._body_quat_xyzw(env, env.plate_body_id))

        self.data["_timestamps"].append(sim_time)
        self.last_record_time = sim_time

    def save_success_demo(self, demo_index):
        if not self.data["_timestamps"]:
            print("No data to save.")
            return False

        hdf5_path = os.path.join(self.cfg.save_dir, f"demo_{demo_index}.hdf5")
        video_path = os.path.join(self.cfg.save_dir, f"demo_{demo_index}.mp4")

        try:
            with h5py.File(hdf5_path, "w") as f:
                root = f.create_group("root")
                root.attrs["env_name"] = self.cfg.env_name
                root.attrs["controller"] = "EEF_POSE"
                root.attrs["controller_input_type"] = "absolute"
                root.attrs["action_label_type"] = "eef_pose_absolute_controller_input"
                root.attrs["action_layout"] = "link6_x,link6_y,link6_z,link6_rotvec_x,link6_rotvec_y,link6_rotvec_z,gripper"
                root.attrs["action_frame"] = "link6_initial"
                root.attrs["grasp_orientation_mode"] = "downward_link6_x_axis_yaw_from_lemon_plus_offset_folded_to_90deg"
                root.attrs["fixed_initial_arm_qpos"] = FIXED_INITIAL_QPOS
                root.attrs["fixed_initial_full_qpos"] = FIXED_INITIAL_FULL_QPOS
                root.attrs["requested_initial_gripper_qpos"] = FIXED_INITIAL_GRIPPER_QPOS
                root.attrs["grasp_frame_offset_pos_link6"] = GRASP_FRAME_OFFSET_POS
                root.attrs["grasp_frame_offset_rot_link6"] = GRASP_FRAME_OFFSET_ROT
                root.attrs["downward_grasp_rot_world_at_yaw0"] = DOWNWARD_GRASP_ROT
                root.attrs["grasp_yaw_offset_rad"] = self.cfg.grasp_yaw_offset
                root.attrs["grasp_yaw_offset_deg"] = np.rad2deg(self.cfg.grasp_yaw_offset)
                root.attrs["grasp_yaw_folded_to_half_turn"] = True
                root.attrs["lift_yaw_rad"] = self.cfg.lift_yaw
                root.attrs["lift_yaw_deg"] = np.rad2deg(self.cfg.lift_yaw)
                root.attrs["lift_height"] = self.cfg.lift_height
                root.attrs["place_hover_height"] = self.cfg.place_hover_height
                root.attrs["success_stable_steps"] = self.cfg.success_stable_steps
                root.attrs["lemon_locked_until_grasp"] = self.cfg.lock_lemon_until_grasp
                root.attrs["motion_speed"] = self.cfg.motion_speed
                if self.applied_initial_gripper_qpos is not None:
                    root.attrs["applied_initial_gripper_qpos"] = self.applied_initial_gripper_qpos
                if self.initial_lemon_pose is not None:
                    root.attrs["initial_lemon_pose"] = self.initial_lemon_pose
                if self.initial_plate_pose is not None:
                    root.attrs["initial_plate_pose"] = self.initial_plate_pose
                if self.link6_initial_body_name is not None:
                    root.attrs["action_state_reference_body_name"] = self.link6_initial_body_name
                if self.ee_body_name is not None:
                    root.attrs["action_state_eef_body_name"] = self.ee_body_name

                actions = np.array(self.data["eef_pose_controller_inputs"])
                root.create_dataset("actions", data=actions)

                extra_group = root.create_group("extra_states")
                extra_group.attrs["ee_frame"] = "link6_initial"
                extra_group.attrs["ee_pose_convention"] = "T_link6_initial_link6"
                extra_group.attrs["reference_body"] = LINK6_REFERENCE_BODY
                extra_group.attrs["eef_body"] = LINK6_EEF_BODY
                if self.world_t_link6_initial is not None:
                    extra_group.attrs["T_world_link6_initial"] = self.world_t_link6_initial
                extra_group.create_dataset("joint_states", data=np.array(self.data["joint_states"]))
                extra_group.create_dataset("gripper_states", data=np.array(self.data["gripper_states"]))
                extra_group.create_dataset("ee_positions", data=np.array(self.data["ee_positions"]))
                extra_group.create_dataset("ee_orientations", data=np.array(self.data["ee_orientations"]))
                extra_group.create_dataset("lemon_positions", data=np.array(self.data["lemon_positions"]))
                extra_group.create_dataset("lemon_quats", data=np.array(self.data["lemon_quats"]))
                extra_group.create_dataset("plate_positions", data=np.array(self.data["plate_positions"]))
                extra_group.create_dataset("plate_quats", data=np.array(self.data["plate_quats"]))
                extra_group.create_dataset("eef_pose_controller_inputs", data=actions)

                view_map = {
                    "external_cam": "agentview",
                    "robot0_right_eye_in_hand": "eye_in_hand",
                }
                for src_name, dst_name in view_map.items():
                    imgs = np.array(self.data[src_name])
                    if len(imgs) == 0:
                        continue
                    imgs_t = np.transpose(imgs, (0, 3, 1, 2))
                    imgs_final = np.expand_dims(imgs_t, axis=0)
                    view_group = root.create_group(dst_name)
                    view_group.create_dataset("video", data=imgs_final, dtype="u1")

            if self.video_frames and not self.cfg.no_video:
                try:
                    import moviepy.editor as mpy

                    clip = mpy.ImageSequenceClip(self.video_frames, fps=self.cfg.record_freq)
                    clip.write_videofile(video_path, codec="libx264", audio=False, verbose=False, logger=None)
                except Exception as e:
                    print(f"Video save error (ignored): {e}")

            return True
        except Exception as e:
            print(f"HDF5 save failed: {e}")
            return False


class LemonEEFPlanner:
    def __init__(self, env, recorder):
        self.env = env
        self.recorder = recorder
        self.cfg = recorder.cfg
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.release_segment_idx = None

    def _current_link6_world_pose(self):
        _, pose = get_body_pose(self.env, LINK6_EEF_BODY)
        return pose

    def _target_link6_pose_for_grasp_frame(self, world_t_grasp):
        return world_t_grasp @ T_GRASP_LINK6

    def _append_traj(self, start_pose, end_pose, duration, gripper, lock_lemon=False):
        self.segments.append(
            {
                "traj": MinJerkPoseTrajectory(start_pose, end_pose, duration),
                "duration": duration,
                "gripper": gripper,
                "pause": False,
                "lock_lemon": lock_lemon,
            }
        )

    def _append_pause(self, hold_pose, duration, gripper, lock_lemon=False):
        self.segments.append(
            {
                "hold_pose": hold_pose.copy(),
                "duration": duration,
                "gripper": gripper,
                "pause": True,
                "lock_lemon": lock_lemon,
            }
        )

    def plan_task(self):
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.release_segment_idx = None
        self.place_hover_planned = False
        self.release_planned = False

        world_t_link6 = self._current_link6_world_pose()
        world_t_lemon = get_body_pose_by_id(self.env, self.env.lemon_body_id)

        grasp_yaw = grasp_yaw_from_object(world_t_lemon, self.cfg.grasp_yaw_offset)
        grasp_rot = downward_grasp_rotation(grasp_yaw)
        lemon_grasp = make_pose(world_t_lemon[:3, 3].copy(), grasp_rot)
        lemon_hover = lemon_grasp.copy()
        lemon_hover[:3, 3] += np.array([0.0, 0.0, 0.18])

        lift_grasp = make_pose(world_t_lemon[:3, 3].copy(), downward_grasp_rotation(self.cfg.lift_yaw))
        lift_grasp[:3, 3] += np.array([0.0, 0.0, self.cfg.lift_height])

        link6_hover = self._target_link6_pose_for_grasp_frame(lemon_hover)
        link6_grasp = self._target_link6_pose_for_grasp_frame(lemon_grasp)
        link6_lift = self._target_link6_pose_for_grasp_frame(lift_grasp)

        speed = max(float(self.cfg.motion_speed), 0.02)
        dist = np.linalg.norm(link6_hover[:3, 3] - world_t_link6[:3, 3])
        self._append_traj(world_t_link6, link6_hover, max(dist / speed, 3.0), OPEN_GRIPPER, lock_lemon=True)
        self._append_traj(link6_hover, link6_grasp, 1.8, OPEN_GRIPPER, lock_lemon=True)
        self._append_pause(link6_grasp, 0.8, CLOSE_GRIPPER)
        self._append_traj(link6_grasp, link6_lift, 2.0, CLOSE_GRIPPER)
        return True

    def _plan_place_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        world_t_lemon = get_body_pose_by_id(self.env, self.env.lemon_body_id)
        world_t_plate = get_body_pose_by_id(self.env, self.env.plate_body_id)

        link6_t_lemon = np.linalg.inv(world_t_link6) @ world_t_lemon
        lemon_t_link6 = np.linalg.inv(link6_t_lemon)
        lemon_rot = world_t_lemon[:3, :3].copy()

        lemon_plate_hover = make_pose(world_t_plate[:3, 3].copy(), lemon_rot)
        lemon_plate_hover[:3, 3] += np.array([0.0, 0.0, self.cfg.place_hover_height])

        link6_plate_hover = lemon_plate_hover @ lemon_t_link6

        speed = max(float(self.cfg.motion_speed), 0.02)
        dist = np.linalg.norm(link6_plate_hover[:3, 3] - world_t_link6[:3, 3])
        self._append_traj(world_t_link6, link6_plate_hover, max(dist / speed, 3.0), CLOSE_GRIPPER)
        self.place_hover_planned = True
        self.segment_start_time = sim_time

    def _plan_release_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        world_t_lemon = get_body_pose_by_id(self.env, self.env.lemon_body_id)
        world_t_plate = get_body_pose_by_id(self.env, self.env.plate_body_id)

        link6_t_lemon = np.linalg.inv(world_t_link6) @ world_t_lemon
        lemon_t_link6 = np.linalg.inv(link6_t_lemon)
        lemon_rot = world_t_lemon[:3, :3].copy()

        lemon_plate_release = make_pose(world_t_plate[:3, 3].copy(), lemon_rot)
        lemon_plate_release[:3, 3] += np.array([0.0, 0.0, 0.07])

        link6_plate_release = lemon_plate_release @ lemon_t_link6
        link6_retreat = link6_plate_release.copy()
        link6_retreat[:3, 3] += np.array([0.0, 0.0, 0.16])

        self._append_traj(world_t_link6, link6_plate_release, 1.8, CLOSE_GRIPPER)
        self.release_segment_idx = len(self.segments)
        self._append_pause(link6_plate_release, 0.8, OPEN_GRIPPER)
        self._append_traj(link6_plate_release, link6_retreat, 1.4, OPEN_GRIPPER)
        self._append_pause(link6_retreat, 1.0, OPEN_GRIPPER)
        self.release_planned = True
        self.segment_start_time = sim_time

    def should_lock_lemon(self):
        if self.current_segment_idx >= len(self.segments):
            return False
        return bool(self.segments[self.current_segment_idx].get("lock_lemon", False))

    def get_action(self, sim_time):
        if self.current_segment_idx >= len(self.segments):
            if not self.place_hover_planned:
                self._plan_place_from_current_state(sim_time)
                return self.get_action(sim_time)
            if not self.release_planned:
                self._plan_release_from_current_state(sim_time)
                return self.get_action(sim_time)
            return None

        seg = self.segments[self.current_segment_idx]
        elapsed = sim_time - self.segment_start_time
        if elapsed >= seg["duration"]:
            self.current_segment_idx += 1
            self.segment_start_time = sim_time
            return self.get_action(sim_time)

        if seg["pause"]:
            world_t_link6_target = seg["hold_pose"]
        else:
            world_t_link6_target = seg["traj"].get_pose(elapsed)

        arm_action = pose_to_action_in_reference(self.recorder.world_t_link6_initial, world_t_link6_target)
        arm_action = np.clip(arm_action, [-1, -1, -1, -np.pi, -np.pi, -np.pi], [1, 1, 1, np.pi, np.pi, np.pi])
        return np.concatenate([arm_action, [seg["gripper"]]])


def create_eef_pose_controller_config(cfg):
    controller_path = os.path.join(PACKAGE_ROOT, "controllers", "config", "robots", "default_arx5_eef_pose.json")
    config = load_composite_controller_config(controller=controller_path)

    for part_config in config.get("body_parts", {}).values():
        if part_config.get("type") == "EEF_POSE":
            part_config["input_type"] = "absolute"
            part_config["input_ref_frame"] = "link6_initial"
            part_config["eef_body_name"] = LINK6_EEF_BODY
            part_config["reference_body_name"] = LINK6_REFERENCE_BODY
            part_config["kp"] = cfg.eef_kp
            part_config["damping_ratio"] = 1
            part_config["interpolation"] = None
        if "gripper" in part_config:
            part_config["gripper"] = {"type": "GRIP"}
    return config


def create_env(cfg, worker_id=0):
    controller_config = create_eef_pose_controller_config(cfg)
    env_seed = None if cfg.seed is None or cfg.seed < 0 else cfg.seed + worker_id
    env = suite.make(
        env_name=cfg.env_name,
        robots=cfg.robot,
        gripper_types=cfg.gripper_type,
        controller_configs=controller_config,
        has_renderer=(not cfg.headless),
        has_offscreen_renderer=cfg.use_camera_obs,
        use_camera_obs=cfg.use_camera_obs,
        use_object_obs=True,
        camera_names=["external_cam", "robot0_right_eye_in_hand"],
        camera_heights=cfg.img_size[1],
        camera_widths=cfg.img_size[0],
        control_freq=cfg.control_freq,
        horizon=2000,
        ignore_done=True,
        hard_reset=True,
        seed=env_seed,
    )
    return env


def randomize_camera_pose(env, cfg):
    for cam_name in ["external_cam", "robot0_right_eye_in_hand"]:
        try:
            cam_id = env.sim.model.camera_name2id(cam_name)
            env.sim.model.cam_pos[cam_id] += np.random.normal(0, cfg.camera_pos_noise_std, 3)
            angle_noise = np.random.normal(0, cfg.camera_ori_noise_std, 3)
            rot_noise = R.from_rotvec(angle_noise)
            current_quat = env.sim.model.cam_quat[cam_id].copy()
            current_rot = R.from_quat([current_quat[1], current_quat[2], current_quat[3], current_quat[0]])
            new_rot = rot_noise * current_rot
            new_quat_xyzw = new_rot.as_quat()
            env.sim.model.cam_quat[cam_id] = [
                new_quat_xyzw[3],
                new_quat_xyzw[0],
                new_quat_xyzw[1],
                new_quat_xyzw[2],
            ]
        except Exception:
            pass


def reset_to_fixed_initial(env, recorder):
    robot = env.robots[0]
    j_start = robot.joint_indexes[0]
    j_end = robot.joint_indexes[-1] + 1
    env.sim.data.qpos[j_start:j_end] = FIXED_INITIAL_QPOS
    recorder.applied_initial_gripper_qpos = set_initial_gripper_qpos(env)
    env.sim.forward()
    sync_arm_controller_to_current_state(env)
    recorder.set_link6_initial_reference(env)


def get_free_joint_qpos(env, joint_name):
    return env.sim.data.get_joint_qpos(joint_name).copy()


def hold_free_joint_pose(env, joint_name, joint_qpos):
    env.sim.data.set_joint_qpos(joint_name, joint_qpos)
    env.sim.data.set_joint_qvel(joint_name, np.zeros(6))
    env.sim.forward()


def current_link6_hold_action(env, recorder, gripper=OPEN_GRIPPER):
    _, world_t_link6 = get_body_pose(env, LINK6_EEF_BODY)
    arm_action = pose_to_action_in_reference(recorder.world_t_link6_initial, world_t_link6)
    arm_action = np.clip(arm_action, [-1, -1, -1, -np.pi, -np.pi, -np.pi], [1, 1, 1, np.pi, np.pi, np.pi])
    return np.concatenate([arm_action, [gripper]])


def worker_collect(worker_id, shared_counter, lock, cfg):
    env = create_env(cfg, worker_id=worker_id)
    recorder = LemonDataRecorder(cfg)
    planner = LemonEEFPlanner(env, recorder)
    dt = 1.0 / cfg.control_freq
    episode_count = 0

    print(f"[Worker {worker_id}] Started.", flush=True)
    while True:
        with lock:
            if shared_counter.value >= cfg.num_demos:
                break
        if cfg.max_episodes > 0 and episode_count >= cfg.max_episodes:
            print(f"[Worker {worker_id}] Reached max_episodes={cfg.max_episodes}.", flush=True)
            break
        episode_count += 1

        obs = env.reset()
        recorder.start_new_demo()
        randomize_camera_pose(env, cfg)
        reset_to_fixed_initial(env, recorder)

        lemon_joint = env.lemon.joints[0] if getattr(env.lemon, "joints", None) else None
        locked_lemon_qpos = get_free_joint_qpos(env, lemon_joint) if lemon_joint is not None else None

        obs = env._get_observations(force_update=True)
        for _ in range(cfg.settle_steps):
            if cfg.lock_lemon_until_grasp and lemon_joint is not None:
                hold_free_joint_pose(env, lemon_joint, locked_lemon_qpos)
            obs, _, _, _ = env.step(np.zeros(env.action_dim))
            if cfg.lock_lemon_until_grasp and lemon_joint is not None:
                hold_free_joint_pose(env, lemon_joint, locked_lemon_qpos)

        recorder.initial_lemon_pose = get_body_pose_by_id(env, env.lemon_body_id)
        recorder.initial_plate_pose = get_body_pose_by_id(env, env.plate_body_id)

        if not planner.plan_task():
            continue

        sim_time = 0.0
        planner.segment_start_time = sim_time
        success = False
        stable_success_steps = 0

        for _ in range(cfg.max_steps):
            action = planner.get_action(sim_time)
            if action is None:
                if stable_success_steps <= 0:
                    break
                action = current_link6_hold_action(env, recorder, OPEN_GRIPPER)

            if cfg.lock_lemon_until_grasp and lemon_joint is not None and planner.should_lock_lemon():
                hold_free_joint_pose(env, lemon_joint, locked_lemon_qpos)
            obs, reward, _, _ = env.step(action)
            if cfg.lock_lemon_until_grasp and lemon_joint is not None and planner.should_lock_lemon():
                hold_free_joint_pose(env, lemon_joint, locked_lemon_qpos)
                obs = env._get_observations(force_update=True)
            sim_time += dt
            recorder.record_frame(env, obs, sim_time, action)

            if env._check_success():
                stable_success_steps += 1
                if stable_success_steps >= cfg.success_stable_steps:
                    success = True
                    break
            else:
                stable_success_steps = 0

        if success:
            with lock:
                if shared_counter.value < cfg.num_demos:
                    demo_idx = shared_counter.value
                    shared_counter.value += 1
                    print(f"[Worker {worker_id}] SUCCESS lemon -> plate. Saving demo {demo_idx}...", flush=True)
                    recorder.save_success_demo(demo_idx)
        else:
            lemon_pos = env.sim.data.body_xpos[env.lemon_body_id].copy()
            plate_pos = env.sim.data.body_xpos[env.plate_body_id].copy()
            print(
                f"[Worker {worker_id}] Episode failed, discarded. "
                f"lemon_pos={np.round(lemon_pos, 4)}, plate_pos={np.round(plate_pos, 4)}",
                flush=True,
            )

    env.close()


def run_main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--render", action="store_true", help="Show MuJoCo viewer instead of running headless.")
    parser.add_argument("--num_demos", type=int, default=50)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--save_dir", type=str, default="lemon_eef_pose_demonstrations")
    parser.add_argument("--max_steps", type=int, default=700)
    parser.add_argument("--record_freq", type=int, default=20)
    parser.add_argument("--control_freq", type=int, default=20)
    parser.add_argument("--seed", type=int, default=-1, help="Use -1 for non-deterministic placement sampling.")
    parser.add_argument("--eef_kp", type=float, default=150.0)
    parser.add_argument("--motion_speed", type=float, default=0.12)
    parser.add_argument(
        "--grasp_yaw_offset_deg",
        type=float,
        default=0.0,
        help="Extra yaw around the downward grasp axis. 0 grips along lemon local-y; 90 grips along local-x.",
    )
    parser.add_argument(
        "--lift_yaw_deg",
        type=float,
        default=0.0,
        help="Fixed yaw around the downward link6 x-axis after grasp. 0 straightens the wrist during lift.",
    )
    parser.add_argument("--lift_height", type=float, default=0.16, help="Vertical lift height after grasp, in meters.")
    parser.add_argument(
        "--place_hover_height",
        type=float,
        default=0.16,
        help="Height above the plate center before descending to release, in meters.",
    )
    parser.add_argument(
        "--success_stable_steps",
        type=int,
        default=20,
        help="Require this many consecutive successful steps before saving a demo.",
    )
    parser.add_argument(
        "--no_lock_lemon_until_grasp",
        action="store_true",
        help="Do not hold the lemon fixed during settle / approach / descend.",
    )
    parser.add_argument("--no_video", action="store_true")
    parser.add_argument("--no_camera_obs", action="store_true", help="Disable camera observations for renderer-less tests.")
    parser.add_argument("--max_episodes", type=int, default=0, help="Debug limit per worker. 0 means unlimited.")
    args = parser.parse_args()

    cfg = replace(
        LemonEEFCollectConfig(),
        headless=(not args.render) if args.render else args.headless,
        num_demos=args.num_demos,
        workers=args.workers,
        save_dir=args.save_dir,
        max_steps=args.max_steps,
        record_freq=args.record_freq,
        control_freq=args.control_freq,
        seed=args.seed,
        eef_kp=args.eef_kp,
        motion_speed=args.motion_speed,
        grasp_yaw_offset=np.deg2rad(args.grasp_yaw_offset_deg),
        lift_yaw=np.deg2rad(args.lift_yaw_deg),
        lift_height=args.lift_height,
        place_hover_height=args.place_hover_height,
        success_stable_steps=args.success_stable_steps,
        lock_lemon_until_grasp=not args.no_lock_lemon_until_grasp,
        no_video=args.no_video,
        use_camera_obs=not args.no_camera_obs,
        max_episodes=args.max_episodes,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"Saving Lemon EEF_POSE demos to {cfg.save_dir} ({timestamp})", flush=True)

    if cfg.workers > 1:
        mp.set_start_method("spawn", force=True)
        manager = mp.Manager()
        counter = manager.Value("i", 0)
        lock = manager.Lock()
        procs = []
        for worker_id in range(cfg.workers):
            p = mp.Process(target=worker_collect, args=(worker_id, counter, lock, cfg))
            p.start()
            procs.append(p)
        for p in procs:
            p.join()
    else:
        class Counter:
            value = 0

        worker_collect(0, Counter(), mp.Lock(), cfg)


if __name__ == "__main__":
    run_main()
