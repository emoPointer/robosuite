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

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PACKAGE_ROOT)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import robosuite as suite
from robosuite.scripts.arx_lemon_eef_pose_collect import (
    FIXED_INITIAL_FULL_QPOS,
    FIXED_INITIAL_GRIPPER_QPOS,
    FIXED_INITIAL_QPOS,
    GRASP_FRAME_OFFSET_POS,
    GRASP_FRAME_OFFSET_ROT,
    LINK6_EEF_BODY,
    LINK6_REFERENCE_BODY,
    CLOSE_GRIPPER,
    OPEN_GRIPPER,
    T_GRASP_LINK6,
    MinJerkPoseTrajectory,
    create_eef_pose_controller_config,
    current_link6_hold_action,
    downward_grasp_rotation,
    fold_yaw_to_half_turn,
    get_body_pose,
    get_body_pose_by_id,
    make_pose,
    pose_to_action_in_reference,
    randomize_camera_pose,
    reset_to_fixed_initial,
    sync_arm_controller_to_current_state,
)
from robosuite.utils.transform_utils import convert_quat

logging.getLogger("moviepy").setLevel(logging.ERROR)

HOLD_GRIPPER = 0.0
REAL_COFFEE_POD_DIMENSIONS_MM = np.array([48.60, 48.60, 46.21])
REAL_DRAWER_COLOR = "yellow"
REAL_COFFEE_POD_COLOR = "green"


@dataclass
class DrawerEEFCollectConfig:
    robot: str = "Arx5"
    env_name: str = "Drawer"
    gripper_type: str = "ArxGripper"
    control_freq: int = 20
    record_freq: int = 20
    save_dir: str = "drawer_eef_pose_demonstrations"
    img_size: tuple = (640, 480)
    save_size: tuple = (350, 350)
    num_demos: int = 50
    workers: int = 1
    headless: bool = True
    max_steps: int = 1200
    settle_steps: int = 20
    no_video: bool = False
    use_camera_obs: bool = True
    max_episodes: int = 0
    seed: int = -1
    eef_kp: float = 150.0
    motion_speed: float = 0.18
    contact_local_x: float = 0.14
    contact_local_y: float = 0.0
    contact_local_z: float = 0.04
    pull_distance: float = 0.03
    drawer_qpos_start: float = 0.06
    drawer_qpos_end: float = 0.09
    drawer_qpos_closed: float = 0.025
    approach_height: float = 0.10
    post_open_lift_height: float = 0.12
    contact_pause: float = 0.25
    pull_duration: float = 1.0
    post_pull_pause: float = 0.4
    pod_grasp_z_offset: float = 0.0
    pod_lift_height: float = 0.12
    pod_place_front_local_x: float = 0.13
    pod_place_local_x: float = 0.145
    pod_place_local_y: float = 0.0
    pod_place_local_z: float = 0.035
    pod_place_hover_height: float = 0.12
    pod_post_release_lift_height: float = 0.06
    pod_release_pause: float = 0.55
    pod_retreat_local_x: float = 0.06
    pod_retreat_height: float = 0.02
    close_contact_local_x: float = 0.222
    close_approach_height: float = 0.07
    close_outside_offset: float = 0.035
    close_contact_pause: float = 0.2
    close_duration: float = 1.1
    success_stable_steps: int = 20
    post_task_wait_steps: int = 60
    drive_drawer_qpos: bool = True
    attach_pod_after_grasp: bool = True
    snap_pod_to_grasp_frame_on_attach: bool = True
    carry_pod_with_drawer_during_close: bool = True
    snap_pod_to_drawer_place_on_release: bool = True
    drawer_qpos_tolerance: float = 0.003
    camera_pos_noise_std: float = 0.0
    camera_ori_noise_std: float = 0.0

    @property
    def contact_local_pos(self):
        return np.array([self.contact_local_x, self.contact_local_y, self.contact_local_z], dtype=np.float64)

    @property
    def pull_end_local_pos(self):
        return self.contact_local_pos + np.array([self.pull_distance, 0.0, 0.0], dtype=np.float64)

    @property
    def close_end_local_pos(self):
        close_distance = self.drawer_qpos_end - self.drawer_qpos_closed
        return self.close_contact_local_pos - np.array([close_distance, 0.0, 0.0], dtype=np.float64)

    @property
    def close_contact_local_pos(self):
        return np.array(
            [self.close_contact_local_x, self.contact_local_y, self.contact_local_z], dtype=np.float64
        )

    @property
    def close_outside_local_pos(self):
        return self.close_contact_local_pos + np.array([self.close_outside_offset, 0.0, 0.0], dtype=np.float64)

    @property
    def pod_place_front_local_pos(self):
        return np.array(
            [self.pod_place_front_local_x, self.pod_place_local_y, self.pod_place_local_z], dtype=np.float64
        )

    @property
    def pod_place_local_pos(self):
        return np.array([self.pod_place_local_x, self.pod_place_local_y, self.pod_place_local_z], dtype=np.float64)


def min_jerk_alpha(elapsed, duration):
    tau = float(np.clip(elapsed / max(duration, 1e-6), 0.0, 1.0))
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def set_drawer_qpos(env, qpos):
    env.sim.data.qpos[env.drawer_qpos_addr] = float(qpos)
    env.sim.data.qvel[env.drawer_qvel_addr] = 0.0
    env.sim.forward()


def set_free_joint_pose_from_matrix(env, joint_name, pose):
    quat_xyzw = R.from_matrix(pose[:3, :3]).as_quat()
    quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
    env.sim.data.set_joint_qpos(joint_name, np.concatenate([pose[:3, 3], quat_wxyz]))
    env.sim.data.set_joint_qvel(joint_name, np.zeros(6))
    env.sim.forward()


def hold_pod_attached_to_link6(env, pod_joint, link6_t_pod):
    _, world_t_link6 = get_body_pose(env, LINK6_EEF_BODY)
    world_t_pod = world_t_link6 @ link6_t_pod
    set_free_joint_pose_from_matrix(env, pod_joint, world_t_pod)


def make_drawer_carried_pod_state(env, cfg=None):
    world_t_drawer = get_body_pose_by_id(env, env.drawer_body_id)
    world_t_pod = get_body_pose_by_id(env, env.pod_body_id)
    drawer_t_pod = np.linalg.inv(world_t_drawer) @ world_t_pod
    drawer_qpos = float(env.sim.data.qpos[env.drawer_qpos_addr])
    if cfg is not None and cfg.snap_pod_to_drawer_place_on_release:
        return {
            "x_offset_from_slide": float(cfg.pod_place_local_x - cfg.drawer_qpos_end),
            "local_yz": np.array([cfg.pod_place_local_y, cfg.pod_place_local_z], dtype=np.float64),
            "local_rot": drawer_t_pod[:3, :3].copy(),
        }
    return {
        "x_offset_from_slide": float(drawer_t_pod[0, 3] - drawer_qpos),
        "local_yz": drawer_t_pod[:3, 3].copy()[1:3],
        "local_rot": drawer_t_pod[:3, :3].copy(),
    }


def hold_pod_carried_by_drawer(env, pod_joint, carried_state, drawer_qpos):
    world_t_drawer = get_body_pose_by_id(env, env.drawer_body_id)
    local_pos = np.array(
        [
            float(drawer_qpos) + carried_state["x_offset_from_slide"],
            carried_state["local_yz"][0],
            carried_state["local_yz"][1],
        ],
        dtype=np.float64,
    )
    drawer_t_pod = make_pose(local_pos, carried_state["local_rot"])
    world_t_pod = world_t_drawer @ drawer_t_pod
    set_free_joint_pose_from_matrix(env, pod_joint, world_t_pod)


class DrawerDataRecorder:
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
            "drawer_positions": [],
            "drawer_quats": [],
            "drawer_qpos": [],
            "coffee_pod_positions": [],
            "coffee_pod_quats": [],
            "drawer_contact_positions": [],
            "drawer_pull_target_positions": [],
            "drawer_close_target_positions": [],
            "pod_place_target_positions": [],
            "_timestamps": [],
        }
        self.video_frames = []
        self.last_record_time = -1.0
        self.world_t_link6_initial = None
        self.link6_initial_t_world = None
        self.link6_initial_body_name = None
        self.ee_body_name = None
        self.applied_initial_gripper_qpos = None
        self.initial_drawer_pose = None
        self.initial_drawer_qpos = None
        self.initial_pod_pose = None

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
        return self.link6_initial_t_world @ world_t_link6

    @staticmethod
    def _body_quat_xyzw(env, body_id):
        return convert_quat(np.array(env.sim.data.body_xquat[body_id]), to="xyzw")

    def _drawer_local_point_world(self, env, local_pos):
        world_t_drawer = get_body_pose_by_id(env, env.drawer_body_id)
        return world_t_drawer[:3, 3] + world_t_drawer[:3, :3] @ np.asarray(local_pos, dtype=np.float64)

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

        self.data["drawer_positions"].append(env.sim.data.body_xpos[env.drawer_body_id].copy())
        self.data["drawer_quats"].append(self._body_quat_xyzw(env, env.drawer_body_id))
        self.data["drawer_qpos"].append(np.array([float(env.sim.data.qpos[env.drawer_qpos_addr])]))
        self.data["coffee_pod_positions"].append(env.sim.data.body_xpos[env.pod_body_id].copy())
        self.data["coffee_pod_quats"].append(self._body_quat_xyzw(env, env.pod_body_id))
        self.data["drawer_contact_positions"].append(self._drawer_local_point_world(env, self.cfg.contact_local_pos))
        self.data["drawer_pull_target_positions"].append(self._drawer_local_point_world(env, self.cfg.pull_end_local_pos))
        self.data["drawer_close_target_positions"].append(
            self._drawer_local_point_world(env, self.cfg.close_end_local_pos)
        )
        self.data["pod_place_target_positions"].append(self._drawer_local_point_world(env, self.cfg.pod_place_local_pos))

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
                root.attrs["task_object"] = "coffee_pod"
                root.attrs["target_object"] = "drawer"
                root.attrs["real_dimension_order"] = "x,y,z"
                root.attrs["real_task_object_dimensions_mm"] = REAL_COFFEE_POD_DIMENSIONS_MM
                root.attrs["real_task_object_color"] = REAL_COFFEE_POD_COLOR
                root.attrs["real_target_object_color"] = REAL_DRAWER_COLOR
                root.attrs["drawer_motion"] = "open_place_pod_close"
                root.attrs["task_sequence"] = "open_drawer,grasp_coffee_pod_center,place_pod_in_drawer,close_drawer"
                root.attrs["drawer_contact_local_pos"] = self.cfg.contact_local_pos
                root.attrs["drawer_close_contact_local_pos"] = self.cfg.close_contact_local_pos
                root.attrs["drawer_pull_end_local_pos"] = self.cfg.pull_end_local_pos
                root.attrs["drawer_close_outside_local_pos"] = self.cfg.close_outside_local_pos
                root.attrs["drawer_close_end_local_pos"] = self.cfg.close_end_local_pos
                root.attrs["drawer_qpos_start"] = self.cfg.drawer_qpos_start
                root.attrs["drawer_qpos_open"] = self.cfg.drawer_qpos_end
                root.attrs["drawer_qpos_closed"] = self.cfg.drawer_qpos_closed
                root.attrs["pull_distance"] = self.cfg.pull_distance
                root.attrs["pod_grasp_target"] = "coffee_pod_body_center"
                root.attrs["pod_place_local_pos"] = self.cfg.pod_place_local_pos
                root.attrs["pod_place_front_local_pos"] = self.cfg.pod_place_front_local_pos
                root.attrs["pod_grasp_z_offset"] = self.cfg.pod_grasp_z_offset
                root.attrs["pod_lift_height"] = self.cfg.pod_lift_height
                root.attrs["pod_place_hover_height"] = self.cfg.pod_place_hover_height
                root.attrs["pod_post_release_lift_height"] = self.cfg.pod_post_release_lift_height
                root.attrs["pod_release_pause"] = self.cfg.pod_release_pause
                root.attrs["pod_retreat_height"] = self.cfg.pod_retreat_height
                root.attrs["attach_pod_after_grasp"] = self.cfg.attach_pod_after_grasp
                root.attrs["snap_pod_to_grasp_frame_on_attach"] = self.cfg.snap_pod_to_grasp_frame_on_attach
                root.attrs["carry_pod_with_drawer_during_close"] = self.cfg.carry_pod_with_drawer_during_close
                root.attrs["snap_pod_to_drawer_place_on_release"] = self.cfg.snap_pod_to_drawer_place_on_release
                root.attrs["object_pose_override_mode"] = (
                    "coffee_pod_attached_to_link6_after_grasp_and_snapped_to_drawer_place_during_close"
                    if self.cfg.carry_pod_with_drawer_during_close and self.cfg.snap_pod_to_drawer_place_on_release
                    else "coffee_pod_attached_to_link6_after_grasp_and_carried_with_drawer_during_close"
                    if self.cfg.carry_pod_with_drawer_during_close
                    else "coffee_pod_attached_to_link6_only_after_grasp"
                )
                root.attrs["gripper_command_mode"] = "drawer_contact_hold_zero,pod_grasp_open_close"
                root.attrs["orientation_mode"] = "downward_link6_x_axis_with_zero_roll"
                root.attrs["drive_drawer_qpos"] = self.cfg.drive_drawer_qpos
                root.attrs["drawer_qpos_drive_mode"] = (
                    "scripted_linear_with_eef_pull" if self.cfg.drive_drawer_qpos else "physics_only"
                )
                root.attrs["fixed_initial_arm_qpos"] = FIXED_INITIAL_QPOS
                root.attrs["fixed_initial_full_qpos"] = FIXED_INITIAL_FULL_QPOS
                root.attrs["requested_initial_gripper_qpos"] = FIXED_INITIAL_GRIPPER_QPOS
                root.attrs["grasp_frame_offset_pos_link6"] = GRASP_FRAME_OFFSET_POS
                root.attrs["grasp_frame_offset_rot_link6"] = GRASP_FRAME_OFFSET_ROT
                root.attrs["approach_height"] = self.cfg.approach_height
                root.attrs["post_open_lift_height"] = self.cfg.post_open_lift_height
                root.attrs["contact_pause"] = self.cfg.contact_pause
                root.attrs["pull_duration"] = self.cfg.pull_duration
                root.attrs["post_pull_pause"] = self.cfg.post_pull_pause
                root.attrs["close_outside_offset"] = self.cfg.close_outside_offset
                root.attrs["close_contact_pause"] = self.cfg.close_contact_pause
                root.attrs["close_duration"] = self.cfg.close_duration
                root.attrs["success_stable_steps"] = self.cfg.success_stable_steps
                root.attrs["motion_speed"] = self.cfg.motion_speed
                if self.applied_initial_gripper_qpos is not None:
                    root.attrs["applied_initial_gripper_qpos"] = self.applied_initial_gripper_qpos
                if self.initial_drawer_pose is not None:
                    root.attrs["initial_drawer_pose"] = self.initial_drawer_pose
                if self.initial_drawer_qpos is not None:
                    root.attrs["initial_drawer_qpos"] = self.initial_drawer_qpos
                if self.initial_pod_pose is not None:
                    root.attrs["initial_pod_pose"] = self.initial_pod_pose
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
                extra_group.create_dataset("drawer_positions", data=np.array(self.data["drawer_positions"]))
                extra_group.create_dataset("drawer_quats", data=np.array(self.data["drawer_quats"]))
                extra_group.create_dataset("drawer_qpos", data=np.array(self.data["drawer_qpos"]))
                extra_group.create_dataset("coffee_pod_positions", data=np.array(self.data["coffee_pod_positions"]))
                extra_group.create_dataset("coffee_pod_quats", data=np.array(self.data["coffee_pod_quats"]))
                extra_group.create_dataset(
                    "drawer_contact_positions", data=np.array(self.data["drawer_contact_positions"])
                )
                extra_group.create_dataset(
                    "drawer_pull_target_positions", data=np.array(self.data["drawer_pull_target_positions"])
                )
                extra_group.create_dataset(
                    "drawer_close_target_positions", data=np.array(self.data["drawer_close_target_positions"])
                )
                extra_group.create_dataset(
                    "pod_place_target_positions", data=np.array(self.data["pod_place_target_positions"])
                )
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


class DrawerEEFPlanner:
    def __init__(self, env, recorder):
        self.env = env
        self.recorder = recorder
        self.cfg = recorder.cfg
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.place_planned = False
        self.close_planned = False

    def _current_link6_world_pose(self):
        _, pose = get_body_pose(self.env, LINK6_EEF_BODY)
        return pose

    @staticmethod
    def _target_link6_pose_for_grasp_frame(world_t_grasp):
        return world_t_grasp @ T_GRASP_LINK6

    def _drawer_local_pose(self, local_pos, rot):
        world_t_drawer = get_body_pose_by_id(self.env, self.env.drawer_body_id)
        pos = world_t_drawer[:3, 3] + world_t_drawer[:3, :3] @ np.asarray(local_pos, dtype=np.float64)
        return make_pose(pos, rot)

    def _drawer_yaw(self):
        world_t_drawer = get_body_pose_by_id(self.env, self.env.drawer_body_id)
        x_axis = world_t_drawer[:3, 0]
        return float(np.arctan2(x_axis[1], x_axis[0]))

    def _translation_duration(self, start_pose, end_pose, minimum):
        speed = max(float(self.cfg.motion_speed), 0.02)
        distance = np.linalg.norm(end_pose[:3, 3] - start_pose[:3, 3])
        return max(distance / speed, minimum)

    def _append_traj(
        self,
        start_pose,
        end_pose,
        duration,
        gripper,
        drawer_qpos=None,
        attach_pod=False,
        carry_pod_with_drawer=False,
    ):
        self.segments.append(
            {
                "traj": MinJerkPoseTrajectory(start_pose, end_pose, duration),
                "duration": duration,
                "gripper": gripper,
                "pause": False,
                "drawer_qpos": drawer_qpos,
                "attach_pod": attach_pod,
                "carry_pod_with_drawer": carry_pod_with_drawer,
            }
        )

    def _append_pause(
        self,
        hold_pose,
        duration,
        gripper,
        drawer_qpos=None,
        attach_pod=False,
        carry_pod_with_drawer=False,
    ):
        self.segments.append(
            {
                "hold_pose": hold_pose.copy(),
                "duration": duration,
                "gripper": gripper,
                "pause": True,
                "drawer_qpos": drawer_qpos,
                "attach_pod": attach_pod,
                "carry_pod_with_drawer": carry_pod_with_drawer,
            }
        )

    def plan_task(self):
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.place_planned = False
        self.close_planned = False

        world_t_link6 = self._current_link6_world_pose()
        grasp_rot = downward_grasp_rotation(fold_yaw_to_half_turn(self._drawer_yaw()))
        drawer_contact = self._drawer_local_pose(self.cfg.contact_local_pos, grasp_rot)
        drawer_pull_end = self._drawer_local_pose(self.cfg.pull_end_local_pos, grasp_rot)

        drawer_hover = drawer_contact.copy()
        drawer_hover[:3, 3] += np.array([0.0, 0.0, self.cfg.approach_height])

        link6_hover = self._target_link6_pose_for_grasp_frame(drawer_hover)
        link6_contact = self._target_link6_pose_for_grasp_frame(drawer_contact)
        link6_pull_end = self._target_link6_pose_for_grasp_frame(drawer_pull_end)
        link6_pull_lift = link6_pull_end.copy()
        link6_pull_lift[:3, 3] += np.array([0.0, 0.0, self.cfg.post_open_lift_height])

        self._append_traj(
            world_t_link6,
            link6_hover,
            self._translation_duration(world_t_link6, link6_hover, 2.4),
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_start, self.cfg.drawer_qpos_start),
        )
        self._append_traj(
            link6_hover,
            link6_contact,
            self._translation_duration(link6_hover, link6_contact, 0.9),
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_start, self.cfg.drawer_qpos_start),
        )
        self._append_pause(
            link6_contact,
            self.cfg.contact_pause,
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_start, self.cfg.drawer_qpos_start),
        )
        self._append_traj(
            link6_contact,
            link6_pull_end,
            self.cfg.pull_duration,
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_start, self.cfg.drawer_qpos_end),
        )
        self._append_pause(
            link6_pull_end,
            self.cfg.post_pull_pause,
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
        )
        self._append_traj(
            link6_pull_end,
            link6_pull_lift,
            self._translation_duration(link6_pull_end, link6_pull_lift, 0.8),
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
        )

        world_t_pod = get_body_pose_by_id(self.env, self.env.pod_body_id)
        pod_grasp = make_pose(world_t_pod[:3, 3].copy(), grasp_rot)
        pod_grasp[:3, 3] += np.array([0.0, 0.0, self.cfg.pod_grasp_z_offset])
        pod_hover = pod_grasp.copy()
        pod_hover[:3, 3] += np.array([0.0, 0.0, self.cfg.approach_height])
        pod_lift = pod_grasp.copy()
        pod_lift[:3, 3] += np.array([0.0, 0.0, self.cfg.pod_lift_height])

        link6_pod_hover = self._target_link6_pose_for_grasp_frame(pod_hover)
        link6_pod_grasp = self._target_link6_pose_for_grasp_frame(pod_grasp)
        link6_pod_lift = self._target_link6_pose_for_grasp_frame(pod_lift)

        self._append_traj(
            link6_pull_lift,
            link6_pod_hover,
            self._translation_duration(link6_pull_lift, link6_pod_hover, 2.4),
            OPEN_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
        )
        self._append_traj(
            link6_pod_hover,
            link6_pod_grasp,
            self._translation_duration(link6_pod_hover, link6_pod_grasp, 0.9),
            OPEN_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
        )
        self._append_pause(
            link6_pod_grasp,
            0.55,
            CLOSE_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            attach_pod=True,
        )
        self._append_traj(
            link6_pod_grasp,
            link6_pod_lift,
            self._translation_duration(link6_pod_grasp, link6_pod_lift, 1.1),
            CLOSE_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            attach_pod=True,
        )
        return True

    def _plan_place_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        world_t_pod = get_body_pose_by_id(self.env, self.env.pod_body_id)
        link6_t_pod = np.linalg.inv(world_t_link6) @ world_t_pod
        pod_t_link6 = np.linalg.inv(link6_t_pod)
        pod_rot = world_t_pod[:3, :3].copy()

        pod_above_local = self.cfg.pod_place_local_pos.copy()
        pod_above_local[2] += self.cfg.pod_place_hover_height
        pod_release_local = self.cfg.pod_place_local_pos.copy()
        pod_post_release_local = self.cfg.pod_place_local_pos.copy()
        pod_post_release_local[2] += self.cfg.pod_post_release_lift_height
        pod_above = self._drawer_local_pose(pod_above_local, pod_rot)
        pod_release = self._drawer_local_pose(pod_release_local, pod_rot)
        pod_post_release = self._drawer_local_pose(pod_post_release_local, pod_rot)

        link6_above = pod_above @ pod_t_link6
        link6_release = pod_release @ pod_t_link6
        link6_post_release = pod_post_release @ pod_t_link6

        retreat_pos = pod_post_release[:3, 3].copy()
        world_t_drawer = get_body_pose_by_id(self.env, self.env.drawer_body_id)
        retreat_pos += world_t_drawer[:3, :3] @ np.array([self.cfg.pod_retreat_local_x, 0.0, 0.0])
        retreat_pos += np.array([0.0, 0.0, self.cfg.pod_retreat_height])
        pod_retreat = make_pose(retreat_pos, pod_rot)
        link6_retreat = pod_retreat @ pod_t_link6

        self._append_traj(
            world_t_link6,
            link6_above,
            self._translation_duration(world_t_link6, link6_above, 2.4),
            CLOSE_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            attach_pod=True,
        )
        self._append_traj(
            link6_above,
            link6_release,
            self._translation_duration(link6_above, link6_release, 0.9),
            CLOSE_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            attach_pod=True,
        )
        self._append_pause(
            link6_release,
            self.cfg.pod_release_pause,
            OPEN_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            attach_pod=False,
            carry_pod_with_drawer=True,
        )
        self._append_traj(
            link6_release,
            link6_post_release,
            self._translation_duration(link6_release, link6_post_release, 0.6),
            OPEN_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            attach_pod=False,
            carry_pod_with_drawer=True,
        )
        self._append_traj(
            link6_post_release,
            link6_retreat,
            self._translation_duration(link6_post_release, link6_retreat, 0.8),
            OPEN_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            attach_pod=False,
            carry_pod_with_drawer=True,
        )
        self.place_planned = True
        self.segment_start_time = sim_time

    def _plan_close_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        grasp_rot = downward_grasp_rotation(fold_yaw_to_half_turn(self._drawer_yaw()))
        drawer_outside = self._drawer_local_pose(self.cfg.close_outside_local_pos, grasp_rot)
        drawer_contact = self._drawer_local_pose(self.cfg.close_contact_local_pos, grasp_rot)
        drawer_close_end = self._drawer_local_pose(self.cfg.close_end_local_pos, grasp_rot)

        drawer_hover = drawer_outside.copy()
        drawer_hover[:3, 3] += np.array([0.0, 0.0, self.cfg.close_approach_height])

        link6_hover = self._target_link6_pose_for_grasp_frame(drawer_hover)
        link6_outside = self._target_link6_pose_for_grasp_frame(drawer_outside)
        link6_contact = self._target_link6_pose_for_grasp_frame(drawer_contact)
        link6_close_end = self._target_link6_pose_for_grasp_frame(drawer_close_end)

        self._append_traj(
            world_t_link6,
            link6_hover,
            self._translation_duration(world_t_link6, link6_hover, 1.8),
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            carry_pod_with_drawer=True,
        )
        self._append_traj(
            link6_hover,
            link6_outside,
            self._translation_duration(link6_hover, link6_outside, 0.7),
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            carry_pod_with_drawer=True,
        )
        self._append_pause(
            link6_outside,
            self.cfg.close_contact_pause,
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            carry_pod_with_drawer=True,
        )
        self._append_traj(
            link6_outside,
            link6_contact,
            self._translation_duration(link6_outside, link6_contact, 0.65),
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            carry_pod_with_drawer=True,
        )
        self._append_pause(
            link6_contact,
            self.cfg.close_contact_pause,
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_end),
            carry_pod_with_drawer=True,
        )
        self._append_traj(
            link6_contact,
            link6_close_end,
            self.cfg.close_duration,
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_end, self.cfg.drawer_qpos_closed),
            carry_pod_with_drawer=True,
        )
        self._append_pause(
            link6_close_end,
            0.4,
            HOLD_GRIPPER,
            drawer_qpos=(self.cfg.drawer_qpos_closed, self.cfg.drawer_qpos_closed),
            carry_pod_with_drawer=True,
        )
        self.close_planned = True
        self.segment_start_time = sim_time

    def should_attach_pod(self):
        if not self.cfg.attach_pod_after_grasp or self.current_segment_idx >= len(self.segments):
            return False
        return bool(self.segments[self.current_segment_idx].get("attach_pod", False))

    def should_carry_pod_with_drawer(self):
        if not self.cfg.carry_pod_with_drawer_during_close or self.current_segment_idx >= len(self.segments):
            return False
        return bool(self.segments[self.current_segment_idx].get("carry_pod_with_drawer", False))

    def drawer_qpos_target(self, sim_time):
        if self.current_segment_idx >= len(self.segments):
            return self.cfg.drawer_qpos_closed if self.close_planned else self.cfg.drawer_qpos_end
        seg = self.segments[self.current_segment_idx]
        qpos_span = seg.get("drawer_qpos", None)
        if qpos_span is None:
            return None
        start_qpos, end_qpos = qpos_span
        elapsed = sim_time - self.segment_start_time
        alpha = min_jerk_alpha(elapsed, seg["duration"])
        return float(start_qpos + (end_qpos - start_qpos) * alpha)

    def get_action(self, sim_time):
        if self.current_segment_idx >= len(self.segments):
            if not self.place_planned:
                self._plan_place_from_current_state(sim_time)
                return self.get_action(sim_time)
            if not self.close_planned:
                self._plan_close_from_current_state(sim_time)
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


def drawer_success_for_collection(env, cfg):
    return bool(env._check_success())


def drawer_failure_diagnostics(env, planner):
    drawer_pos = env.sim.data.body_xpos[env.drawer_body_id].copy()
    drawer_qpos = float(env.sim.data.qpos[env.drawer_qpos_addr])
    pod_pos = env.sim.data.body_xpos[env.pod_body_id].copy()
    world_t_drawer = get_body_pose_by_id(env, env.drawer_body_id)
    drawer_t_pod = np.linalg.inv(world_t_drawer) @ get_body_pose_by_id(env, env.pod_body_id)
    local_pos = drawer_t_pod[:3, 3]
    pod_in_drawer = bool(env._pod_in_drawer(pod_pos))
    drawer_closed = bool(env._drawer_closed())
    if planner.current_segment_idx < len(planner.segments):
        segment = planner.segments[planner.current_segment_idx]
        phase = (
            f"segment={planner.current_segment_idx}/{len(planner.segments)}, "
            f"attach_pod={segment.get('attach_pod', False)}, "
            f"carry_pod={segment.get('carry_pod_with_drawer', False)}, "
            f"qpos_span={segment.get('drawer_qpos', None)}"
        )
    else:
        phase = f"planner_done place_planned={planner.place_planned}, close_planned={planner.close_planned}"
    return (
        f"drawer_pos={np.round(drawer_pos, 4)}, drawer_qpos={drawer_qpos:.4f}, "
        f"drawer_closed={drawer_closed}, pod_pos={np.round(pod_pos, 4)}, "
        f"pod_local={np.round(local_pos, 4)}, pod_in_drawer={pod_in_drawer}, "
        f"success={env._check_success()}, {phase}"
    )


def worker_collect(worker_id, shared_counter, lock, cfg):
    env = create_env(cfg, worker_id=worker_id)
    recorder = DrawerDataRecorder(cfg)
    planner = DrawerEEFPlanner(env, recorder)
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
        set_drawer_qpos(env, cfg.drawer_qpos_start)

        obs = env._get_observations(force_update=True)
        for _ in range(cfg.settle_steps):
            if cfg.drive_drawer_qpos:
                set_drawer_qpos(env, cfg.drawer_qpos_start)
            obs, _, _, _ = env.step(np.zeros(env.action_dim))
            if cfg.drive_drawer_qpos:
                set_drawer_qpos(env, cfg.drawer_qpos_start)

        sync_arm_controller_to_current_state(env)
        recorder.set_link6_initial_reference(env)
        recorder.initial_drawer_pose = get_body_pose_by_id(env, env.drawer_body_id)
        recorder.initial_drawer_qpos = np.array([float(env.sim.data.qpos[env.drawer_qpos_addr])])
        recorder.initial_pod_pose = get_body_pose_by_id(env, env.pod_body_id)

        if not planner.plan_task():
            continue

        sim_time = 0.0
        planner.segment_start_time = sim_time
        success = False
        stable_success_steps = 0
        post_task_wait_steps = 0
        pod_joint = env.pod.joints[0] if getattr(env.pod, "joints", None) else None
        attached_link6_t_pod = None
        carried_pod_state = None

        for _ in range(cfg.max_steps):
            action = planner.get_action(sim_time)
            if action is None:
                if post_task_wait_steps >= cfg.post_task_wait_steps and stable_success_steps <= 0:
                    break
                action = current_link6_hold_action(env, recorder, HOLD_GRIPPER)
                post_task_wait_steps += 1

            qpos_target = None
            if cfg.drive_drawer_qpos:
                qpos_target = planner.drawer_qpos_target(sim_time)
                if qpos_target is not None:
                    set_drawer_qpos(env, qpos_target)

            attach_pod = planner.should_attach_pod() and pod_joint is not None
            carry_pod = (not attach_pod) and planner.should_carry_pod_with_drawer() and pod_joint is not None
            if attach_pod:
                carried_pod_state = None
                if attached_link6_t_pod is None:
                    _, world_t_link6 = get_body_pose(env, LINK6_EEF_BODY)
                    world_t_pod = get_body_pose_by_id(env, env.pod_body_id)
                    attached_link6_t_pod = np.linalg.inv(world_t_link6) @ world_t_pod
                    if cfg.snap_pod_to_grasp_frame_on_attach:
                        attached_link6_t_pod[:3, 3] = GRASP_FRAME_OFFSET_POS
                hold_pod_attached_to_link6(env, pod_joint, attached_link6_t_pod)
            elif carry_pod:
                attached_link6_t_pod = None
                if carried_pod_state is None:
                    carried_pod_state = make_drawer_carried_pod_state(env, cfg)
                carry_qpos = qpos_target if qpos_target is not None else float(env.sim.data.qpos[env.drawer_qpos_addr])
                hold_pod_carried_by_drawer(env, pod_joint, carried_pod_state, carry_qpos)
            else:
                attached_link6_t_pod = None
                carried_pod_state = None

            obs, _, _, _ = env.step(action)
            sim_time += dt

            if cfg.drive_drawer_qpos:
                qpos_target = planner.drawer_qpos_target(sim_time)
                if qpos_target is not None:
                    set_drawer_qpos(env, qpos_target)
                    obs = env._get_observations(force_update=True)

            if attach_pod:
                hold_pod_attached_to_link6(env, pod_joint, attached_link6_t_pod)
                obs = env._get_observations(force_update=True)
            elif carry_pod:
                carry_qpos = qpos_target if qpos_target is not None else float(env.sim.data.qpos[env.drawer_qpos_addr])
                hold_pod_carried_by_drawer(env, pod_joint, carried_pod_state, carry_qpos)
                obs = env._get_observations(force_update=True)

            recorder.record_frame(env, obs, sim_time, action)

            if drawer_success_for_collection(env, cfg):
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
                    print(
                        f"[Worker {worker_id}] SUCCESS coffee_pod in drawer and drawer closed. "
                        f"Saving demo {demo_idx}...",
                        flush=True,
                    )
                    recorder.save_success_demo(demo_idx)
        else:
            print(f"[Worker {worker_id}] Episode failed, discarded. {drawer_failure_diagnostics(env, planner)}", flush=True)

    env.close()


def run_main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--render", action="store_true", help="Show MuJoCo viewer instead of running headless.")
    parser.add_argument("--num_demos", type=int, default=50)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--save_dir", type=str, default="drawer_eef_pose_demonstrations")
    parser.add_argument("--max_steps", type=int, default=1200)
    parser.add_argument("--record_freq", type=int, default=20)
    parser.add_argument("--control_freq", type=int, default=20)
    parser.add_argument("--seed", type=int, default=-1, help="Use -1 for non-deterministic placement sampling.")
    parser.add_argument("--eef_kp", type=float, default=150.0)
    parser.add_argument("--motion_speed", type=float, default=0.18)
    parser.add_argument("--contact_local_x", type=float, default=0.14)
    parser.add_argument("--contact_local_y", type=float, default=0.0)
    parser.add_argument("--contact_local_z", type=float, default=0.04)
    parser.add_argument("--pull_distance", type=float, default=0.03)
    parser.add_argument("--drawer_qpos_start", type=float, default=0.06)
    parser.add_argument("--drawer_qpos_end", type=float, default=None)
    parser.add_argument("--drawer_qpos_closed", type=float, default=0.025)
    parser.add_argument("--approach_height", type=float, default=0.10)
    parser.add_argument("--post_open_lift_height", type=float, default=0.12)
    parser.add_argument("--contact_pause", type=float, default=0.25)
    parser.add_argument("--pull_duration", type=float, default=1.0)
    parser.add_argument("--post_pull_pause", type=float, default=0.4)
    parser.add_argument("--pod_grasp_z_offset", type=float, default=0.0)
    parser.add_argument("--pod_lift_height", type=float, default=0.12)
    parser.add_argument("--pod_place_front_local_x", type=float, default=0.13)
    parser.add_argument("--pod_place_local_x", type=float, default=0.145)
    parser.add_argument("--pod_place_local_y", type=float, default=0.0)
    parser.add_argument("--pod_place_local_z", type=float, default=0.035)
    parser.add_argument("--pod_place_hover_height", type=float, default=0.12)
    parser.add_argument("--pod_post_release_lift_height", type=float, default=0.06)
    parser.add_argument("--pod_release_pause", type=float, default=0.55)
    parser.add_argument("--pod_retreat_local_x", type=float, default=0.06)
    parser.add_argument("--pod_retreat_height", type=float, default=0.02)
    parser.add_argument("--close_contact_local_x", type=float, default=0.222)
    parser.add_argument("--close_approach_height", type=float, default=0.07)
    parser.add_argument("--close_outside_offset", type=float, default=0.035)
    parser.add_argument("--close_contact_pause", type=float, default=0.2)
    parser.add_argument("--close_duration", type=float, default=1.1)
    parser.add_argument("--success_stable_steps", type=int, default=20)
    parser.add_argument("--post_task_wait_steps", type=int, default=60)
    parser.add_argument("--drawer_qpos_tolerance", type=float, default=0.003)
    parser.add_argument("--disable_drive_drawer_qpos", action="store_true")
    parser.add_argument("--disable_attach_pod", action="store_true")
    parser.add_argument("--disable_snap_pod_to_grasp_frame", action="store_true")
    parser.add_argument("--disable_carry_pod_with_drawer", action="store_true")
    parser.add_argument("--disable_snap_pod_to_drawer_place", action="store_true")
    parser.add_argument("--no_video", action="store_true")
    parser.add_argument("--no_camera_obs", action="store_true", help="Disable camera observations for renderer-less tests.")
    parser.add_argument("--max_episodes", type=int, default=0, help="Debug limit per worker. 0 means unlimited.")
    args = parser.parse_args()

    drawer_qpos_end = (
        args.drawer_qpos_end if args.drawer_qpos_end is not None else args.drawer_qpos_start + args.pull_distance
    )
    cfg = replace(
        DrawerEEFCollectConfig(),
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
        contact_local_x=args.contact_local_x,
        contact_local_y=args.contact_local_y,
        contact_local_z=args.contact_local_z,
        pull_distance=args.pull_distance,
        drawer_qpos_start=args.drawer_qpos_start,
        drawer_qpos_end=drawer_qpos_end,
        drawer_qpos_closed=args.drawer_qpos_closed,
        approach_height=args.approach_height,
        post_open_lift_height=args.post_open_lift_height,
        contact_pause=args.contact_pause,
        pull_duration=args.pull_duration,
        post_pull_pause=args.post_pull_pause,
        pod_grasp_z_offset=args.pod_grasp_z_offset,
        pod_lift_height=args.pod_lift_height,
        pod_place_front_local_x=args.pod_place_front_local_x,
        pod_place_local_x=args.pod_place_local_x,
        pod_place_local_y=args.pod_place_local_y,
        pod_place_local_z=args.pod_place_local_z,
        pod_place_hover_height=args.pod_place_hover_height,
        pod_post_release_lift_height=args.pod_post_release_lift_height,
        pod_release_pause=args.pod_release_pause,
        pod_retreat_local_x=args.pod_retreat_local_x,
        pod_retreat_height=args.pod_retreat_height,
        close_contact_local_x=args.close_contact_local_x,
        close_approach_height=args.close_approach_height,
        close_outside_offset=args.close_outside_offset,
        close_contact_pause=args.close_contact_pause,
        close_duration=args.close_duration,
        success_stable_steps=args.success_stable_steps,
        post_task_wait_steps=args.post_task_wait_steps,
        drawer_qpos_tolerance=args.drawer_qpos_tolerance,
        drive_drawer_qpos=not args.disable_drive_drawer_qpos,
        attach_pod_after_grasp=not args.disable_attach_pod,
        snap_pod_to_grasp_frame_on_attach=not args.disable_snap_pod_to_grasp_frame,
        carry_pod_with_drawer_during_close=not args.disable_carry_pod_with_drawer,
        snap_pod_to_drawer_place_on_release=not args.disable_snap_pod_to_drawer_place,
        no_video=args.no_video,
        use_camera_obs=not args.no_camera_obs,
        max_episodes=args.max_episodes,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"Saving Drawer EEF_POSE demos to {cfg.save_dir} ({timestamp})", flush=True)

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
