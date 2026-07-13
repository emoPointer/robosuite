import argparse
import logging
import multiprocessing as mp
import os
import sys
from dataclasses import dataclass, replace
from datetime import datetime
from typing import Optional

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
    CLOSE_GRIPPER,
    FIXED_INITIAL_FULL_QPOS,
    FIXED_INITIAL_GRIPPER_QPOS,
    FIXED_INITIAL_QPOS,
    GRASP_FRAME_OFFSET_POS,
    GRASP_FRAME_OFFSET_ROT,
    LINK6_EEF_BODY,
    LINK6_REFERENCE_BODY,
    OPEN_GRIPPER,
    T_GRASP_LINK6,
    MinJerkPoseTrajectory,
    create_eef_pose_controller_config,
    current_link6_hold_action,
    downward_grasp_rotation,
    get_body_pose,
    get_body_pose_by_id,
    grasp_yaw_from_object,
    make_pose,
    pose_to_action_in_reference,
    randomize_camera_pose,
    reset_to_fixed_initial,
    sync_arm_controller_to_current_state,
)

logging.getLogger("moviepy").setLevel(logging.ERROR)

REAL_MUG_DIMENSIONS_MM = np.array([69.64, 103.97, 82.70])
REAL_MUG_TREE_DIMENSIONS_MM = np.array([175.0, 160.0, 190.0])
REAL_MUG_COLOR = "red"
REAL_MUG_TREE_COLOR = "wood"


@dataclass
class MugHangEEFCollectConfig:
    robot: str = "Arx5"
    env_name: str = "MugHang"
    gripper_type: str = "ArxGripper"
    shape_id: str = "b4ae56d6"
    mug_scale: float = 1.0
    random_mug_scale: bool = False
    control_freq: int = 20
    record_freq: int = 20
    save_dir: str = "mug_hang_eef_pose_demonstrations"
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
    motion_speed: float = 0.15
    grasp_yaw_offset: float = -np.pi / 2.0
    grasp_roll_offset: float = np.pi / 2.0
    grasp_offset_x: float = 0.0
    grasp_offset_y: Optional[float] = -0.05
    grasp_offset_z: Optional[float] = 0.03
    grasp_edge_fraction: float = 0.95
    grasp_height_fraction: float = 0.70
    grasp_z_offset: float = 0.0
    approach_height: float = 0.08
    lift_height: float = 0.0
    transit_clearance: float = 0.0
    pre_insert_clearance: float = 0.05
    insert_depth: float = 0.03
    target_center_y: float = 0.0
    target_center_z: float = 0.055
    mug_hang_local_x: float = 0.0
    mug_hang_local_y: float = 0.03
    mug_hang_local_z: float = 0.0
    mug_handle_axis: str = "x"
    hang_settle_time: float = 0.6
    release_pause: float = 0.6
    retreat_distance: float = 0.08
    retreat_height: float = 0.08
    success_stable_steps: int = 20
    post_task_wait_steps: int = 300
    attach_mug_after_grasp: bool = True
    debug_hang_alignment: bool = False
    alignment_log_interval: int = 10
    alignment_error_warn: float = 0.015
    alignment_correction_gain: float = 2.0
    alignment_correction_max: float = 0.06
    require_handle_engaged: bool = True
    handle_anchor_tolerance: float = 0.018
    handle_axis_alignment_min: float = 0.75
    handle_segment_margin: float = 0.005
    camera_pos_noise_std: float = 0.0
    camera_ori_noise_std: float = 0.0


class MugHangDataRecorder:
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
            "mug_positions": [],
            "mug_quats": [],
            "mug_tree_positions": [],
            "mug_tree_quats": [],
            "_timestamps": [],
        }
        self.video_frames = []
        self.last_record_time = -1.0
        self.world_t_link6_initial = None
        self.link6_initial_t_world = None
        self.link6_initial_body_name = None
        self.ee_body_name = None
        self.applied_initial_gripper_qpos = None
        self.initial_mug_pose = None
        self.initial_mug_tree_pose = None
        self.computed_grasp_offset = None

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

        self.data["mug_positions"].append(env.sim.data.body_xpos[env.mug_body_id].copy())
        self.data["mug_quats"].append(self._body_quat_xyzw(env, env.mug_body_id))
        self.data["mug_tree_positions"].append(env.sim.data.body_xpos[env.mug_tree_body_id].copy())
        self.data["mug_tree_quats"].append(self._body_quat_xyzw(env, env.mug_tree_body_id))

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
                root.attrs["task_object"] = "mug"
                root.attrs["target_object"] = "mug_tree"
                root.attrs["real_dimension_order"] = "x,y,z"
                root.attrs["real_task_object_dimensions_mm"] = REAL_MUG_DIMENSIONS_MM
                root.attrs["real_target_object_dimensions_mm"] = REAL_MUG_TREE_DIMENSIONS_MM
                root.attrs["real_task_object_color"] = REAL_MUG_COLOR
                root.attrs["real_target_object_color"] = REAL_MUG_TREE_COLOR
                root.attrs["shape_id"] = self.cfg.shape_id
                root.attrs["mug_scale"] = self.cfg.mug_scale
                root.attrs["random_mug_scale"] = self.cfg.random_mug_scale
                root.attrs["fixed_initial_arm_qpos"] = FIXED_INITIAL_QPOS
                root.attrs["fixed_initial_full_qpos"] = FIXED_INITIAL_FULL_QPOS
                root.attrs["requested_initial_gripper_qpos"] = FIXED_INITIAL_GRIPPER_QPOS
                root.attrs["grasp_frame_offset_pos_link6"] = GRASP_FRAME_OFFSET_POS
                root.attrs["grasp_frame_offset_rot_link6"] = GRASP_FRAME_OFFSET_ROT
                root.attrs["grasp_yaw_offset_rad"] = self.cfg.grasp_yaw_offset
                root.attrs["grasp_yaw_offset_deg"] = np.rad2deg(self.cfg.grasp_yaw_offset)
                root.attrs["grasp_roll_offset_rad"] = self.cfg.grasp_roll_offset
                root.attrs["grasp_roll_offset_deg"] = np.rad2deg(self.cfg.grasp_roll_offset)
                requested_grasp_offset = np.array(
                    [
                        self.cfg.grasp_offset_x,
                        np.nan if self.cfg.grasp_offset_y is None else self.cfg.grasp_offset_y,
                        np.nan if self.cfg.grasp_offset_z is None else self.cfg.grasp_offset_z,
                    ]
                )
                root.attrs["requested_grasp_offset_mug_frame"] = requested_grasp_offset
                if self.computed_grasp_offset is not None:
                    root.attrs["grasp_offset_mug_frame"] = self.computed_grasp_offset
                else:
                    root.attrs["grasp_offset_mug_frame"] = requested_grasp_offset
                root.attrs["grasp_edge_fraction"] = self.cfg.grasp_edge_fraction
                root.attrs["grasp_height_fraction"] = self.cfg.grasp_height_fraction
                root.attrs["grasp_z_offset"] = self.cfg.grasp_z_offset
                root.attrs["approach_height"] = self.cfg.approach_height
                root.attrs["lift_height"] = self.cfg.lift_height
                root.attrs["transit_clearance"] = self.cfg.transit_clearance
                root.attrs["pre_insert_clearance"] = self.cfg.pre_insert_clearance
                root.attrs["insert_depth"] = self.cfg.insert_depth
                root.attrs["target_center_y"] = self.cfg.target_center_y
                root.attrs["target_center_z"] = self.cfg.target_center_z
                root.attrs["mug_hang_local_point"] = np.array(
                    [self.cfg.mug_hang_local_x, self.cfg.mug_hang_local_y, self.cfg.mug_hang_local_z]
                )
                root.attrs["mug_handle_axis"] = self.cfg.mug_handle_axis
                root.attrs["hang_settle_time"] = self.cfg.hang_settle_time
                root.attrs["retreat_distance"] = self.cfg.retreat_distance
                root.attrs["retreat_height"] = self.cfg.retreat_height
                root.attrs["success_stable_steps"] = self.cfg.success_stable_steps
                root.attrs["attach_mug_after_grasp"] = self.cfg.attach_mug_after_grasp
                root.attrs["require_handle_engaged"] = self.cfg.require_handle_engaged
                root.attrs["handle_anchor_tolerance"] = self.cfg.handle_anchor_tolerance
                root.attrs["handle_axis_alignment_min"] = self.cfg.handle_axis_alignment_min
                root.attrs["handle_segment_margin"] = self.cfg.handle_segment_margin
                root.attrs["motion_speed"] = self.cfg.motion_speed
                if self.applied_initial_gripper_qpos is not None:
                    root.attrs["applied_initial_gripper_qpos"] = self.applied_initial_gripper_qpos
                if self.initial_mug_pose is not None:
                    root.attrs["initial_mug_pose"] = self.initial_mug_pose
                if self.initial_mug_tree_pose is not None:
                    root.attrs["initial_mug_tree_pose"] = self.initial_mug_tree_pose
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
                extra_group.create_dataset("mug_positions", data=np.array(self.data["mug_positions"]))
                extra_group.create_dataset("mug_quats", data=np.array(self.data["mug_quats"]))
                extra_group.create_dataset("mug_tree_positions", data=np.array(self.data["mug_tree_positions"]))
                extra_group.create_dataset("mug_tree_quats", data=np.array(self.data["mug_tree_quats"]))
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


class MugHangEEFPlanner:
    def __init__(self, env, recorder):
        self.env = env
        self.recorder = recorder
        self.cfg = recorder.cfg
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.hang_planned = False
        self.release_planned = False

    def _current_link6_world_pose(self):
        _, pose = get_body_pose(self.env, LINK6_EEF_BODY)
        return pose

    @staticmethod
    def _target_link6_pose_for_grasp_frame(world_t_grasp):
        return world_t_grasp @ T_GRASP_LINK6

    def _append_traj(
        self,
        start_pose,
        end_pose,
        duration,
        gripper,
        attach_mug=False,
        label=None,
        anchor_start_tree_xyz=None,
        anchor_end_tree_xyz=None,
    ):
        segment = {
            "traj": MinJerkPoseTrajectory(start_pose, end_pose, duration),
            "duration": duration,
            "gripper": gripper,
            "pause": False,
            "attach_mug": attach_mug,
            "label": label,
        }
        if anchor_start_tree_xyz is not None and anchor_end_tree_xyz is not None:
            segment["anchor_start_tree_xyz"] = np.asarray(anchor_start_tree_xyz, dtype=np.float64)
            segment["anchor_end_tree_xyz"] = np.asarray(anchor_end_tree_xyz, dtype=np.float64)
        self.segments.append(segment)

    def _append_pause(self, hold_pose, duration, gripper, attach_mug=False, label=None, anchor_tree_xyz=None):
        segment = {
            "hold_pose": hold_pose.copy(),
            "duration": duration,
            "gripper": gripper,
            "pause": True,
            "attach_mug": attach_mug,
            "label": label,
        }
        if anchor_tree_xyz is not None:
            segment["anchor_tree_xyz"] = np.asarray(anchor_tree_xyz, dtype=np.float64)
        self.segments.append(segment)

    def _legacy_append_traj(self, start_pose, end_pose, duration, gripper, attach_mug=False):
        self.segments.append(
            {
                "traj": MinJerkPoseTrajectory(start_pose, end_pose, duration),
                "duration": duration,
                "gripper": gripper,
                "pause": False,
                "attach_mug": attach_mug,
            }
        )

    def _legacy_append_pause(self, hold_pose, duration, gripper, attach_mug=False):
        self.segments.append(
            {
                "hold_pose": hold_pose.copy(),
                "duration": duration,
                "gripper": gripper,
                "pause": True,
                "attach_mug": attach_mug,
            }
        )

    def _tree_axes_and_geometry(self):
        world_t_tree = get_body_pose_by_id(self.env, self.env.mug_tree_body_id)
        base_size = np.array(self.env.mug_tree.base_size, dtype=np.float64)
        tree_size = np.array(self.env.mug_tree.tree_size, dtype=np.float64)
        branch_size = np.array(self.env.mug_tree.branch_size, dtype=np.float64)
        total_height = base_size[2] + tree_size[2]
        branch_base_x = tree_size[0] / 2.0
        branch_tip_x = tree_size[0] / 2.0 + branch_size[0]
        branch_z = base_size[2] + self.env.mug_tree.branch_height - total_height / 2.0
        return world_t_tree, branch_base_x, branch_tip_x, branch_z

    def _mug_hang_local_point(self):
        return np.array(
            [self.cfg.mug_hang_local_x, self.cfg.mug_hang_local_y, self.cfg.mug_hang_local_z],
            dtype=np.float64,
        )

    def _mug_handle_axis_local(self):
        axis_map = {
            "x": np.array([1.0, 0.0, 0.0]),
            "neg_x": np.array([-1.0, 0.0, 0.0]),
            "y": np.array([0.0, 1.0, 0.0]),
            "neg_y": np.array([0.0, -1.0, 0.0]),
        }
        if self.cfg.mug_handle_axis not in axis_map:
            raise ValueError(f"Unsupported mug_handle_axis: {self.cfg.mug_handle_axis}")
        return axis_map[self.cfg.mug_handle_axis]

    @staticmethod
    def _min_jerk_alpha(elapsed, duration):
        tau = float(np.clip(elapsed / max(duration, 0.1), 0.0, 1.0))
        return 10 * tau**3 - 15 * tau**4 + 6 * tau**5

    def _tree_point_world(self, tree_xyz):
        world_t_tree = get_body_pose_by_id(self.env, self.env.mug_tree_body_id)
        return world_t_tree[:3, 3] + world_t_tree[:3, :3] @ np.asarray(tree_xyz, dtype=np.float64)

    def _mug_anchor_world(self):
        world_t_mug = get_body_pose_by_id(self.env, self.env.mug_body_id)
        return world_t_mug[:3, 3] + world_t_mug[:3, :3] @ self._mug_hang_local_point()

    def _mug_anchor_tree(self):
        world_t_tree = get_body_pose_by_id(self.env, self.env.mug_tree_body_id)
        return world_t_tree[:3, :3].T @ (self._mug_anchor_world() - world_t_tree[:3, 3])

    def handle_engagement_report(self):
        world_t_tree, branch_base_x, branch_tip_x, branch_z = self._tree_axes_and_geometry()
        world_t_mug = get_body_pose_by_id(self.env, self.env.mug_body_id)
        mug_t_world = np.linalg.inv(world_t_mug)

        inner_tree = np.array([branch_base_x, 0.0, branch_z], dtype=np.float64)
        outer_tree = np.array([branch_tip_x, 0.0, branch_z], dtype=np.float64)
        inner_world = world_t_tree @ np.r_[inner_tree, 1.0]
        outer_world = world_t_tree @ np.r_[outer_tree, 1.0]
        inner_mug = (mug_t_world @ inner_world)[:3]
        outer_mug = (mug_t_world @ outer_world)[:3]

        anchor_mug = self._mug_hang_local_point()
        branch_vec_mug = outer_mug - inner_mug
        branch_len = float(np.linalg.norm(branch_vec_mug))
        if branch_len < 1e-8:
            branch_dir_mug = np.zeros(3)
            projected = 0.0
            closest_mug = inner_mug.copy()
            axis_alignment = 0.0
        else:
            branch_dir_mug = branch_vec_mug / branch_len
            projected = float(np.dot(anchor_mug - inner_mug, branch_dir_mug))
            projected_clamped = float(np.clip(projected, 0.0, branch_len))
            closest_mug = inner_mug + projected_clamped * branch_dir_mug
            axis_alignment = float(np.dot(branch_dir_mug, self._mug_handle_axis_local()))

        radial_error = float(np.linalg.norm(anchor_mug - closest_mug))
        inside_segment = -self.cfg.handle_segment_margin <= projected <= branch_len + self.cfg.handle_segment_margin
        engaged = (
            radial_error <= self.cfg.handle_anchor_tolerance
            and inside_segment
            and axis_alignment >= self.cfg.handle_axis_alignment_min
        )

        return {
            "engaged": bool(engaged),
            "anchor_mug": anchor_mug,
            "inner_mug": inner_mug,
            "outer_mug": outer_mug,
            "closest_mug": closest_mug,
            "radial_error": radial_error,
            "projected": projected,
            "branch_len": branch_len,
            "inside_segment": bool(inside_segment),
            "axis_alignment": axis_alignment,
            "inner_tree": inner_tree,
            "outer_tree": outer_tree,
        }

    def _expected_anchor_tree_for_segment(self, seg, elapsed):
        if "anchor_tree_xyz" in seg:
            return seg["anchor_tree_xyz"].copy()
        if "anchor_start_tree_xyz" not in seg or "anchor_end_tree_xyz" not in seg:
            return None
        alpha = self._min_jerk_alpha(elapsed, seg["duration"])
        return seg["anchor_start_tree_xyz"] + (seg["anchor_end_tree_xyz"] - seg["anchor_start_tree_xyz"]) * alpha

    def hang_alignment_report(self, sim_time):
        if self.current_segment_idx >= len(self.segments):
            return None
        seg = self.segments[self.current_segment_idx]
        elapsed = sim_time - self.segment_start_time
        expected_tree = self._expected_anchor_tree_for_segment(seg, elapsed)
        if expected_tree is None:
            return None

        world_t_tree, branch_base_x, branch_tip_x, _ = self._tree_axes_and_geometry()
        actual_tree = self._mug_anchor_tree()
        err_tree = actual_tree - expected_tree
        entry_tree = np.array([branch_tip_x, self.cfg.target_center_y, self.cfg.target_center_z], dtype=np.float64)
        final_tree = np.array(
            [branch_base_x + self.cfg.insert_depth, self.cfg.target_center_y, self.cfg.target_center_z],
            dtype=np.float64,
        )
        return {
            "label": seg.get("label", ""),
            "elapsed": elapsed,
            "actual_tree": actual_tree,
            "expected_tree": expected_tree,
            "error_tree": err_tree,
            "error_norm": float(np.linalg.norm(err_tree)),
            "entry_tree": entry_tree,
            "final_tree": final_tree,
            "tree_pos": world_t_tree[:3, 3].copy(),
        }

    def _mug_pose_in_tree_frame(self, tree_xyz):
        world_t_tree, _, _, _ = self._tree_axes_and_geometry()
        rot = self._target_mug_rotation(world_t_tree)
        hang_point_world = world_t_tree[:3, 3] + world_t_tree[:3, :3] @ np.asarray(tree_xyz, dtype=np.float64)
        pos = hang_point_world - rot @ self._mug_hang_local_point()
        return make_pose(pos, rot)

    def _target_mug_rotation(self, world_t_tree):
        branch_axis = world_t_tree[:3, :3] @ np.array([1.0, 0.0, 0.0])
        branch_axis[2] = 0.0
        branch_axis /= np.linalg.norm(branch_axis)
        handle_axis = self._mug_handle_axis_local()
        branch_yaw = np.arctan2(branch_axis[1], branch_axis[0])
        handle_yaw = np.arctan2(handle_axis[1], handle_axis[0])
        return R.from_euler("z", branch_yaw - handle_yaw).as_matrix()

    def _held_mug_to_link6_transform(self):
        world_t_link6 = self._current_link6_world_pose()
        world_t_mug = get_body_pose_by_id(self.env, self.env.mug_body_id)
        link6_t_mug = np.linalg.inv(world_t_link6) @ world_t_mug
        return np.linalg.inv(link6_t_mug)

    def _grasp_local_offset(self):
        x = float(self.cfg.grasp_offset_x)

        if self.cfg.grasp_offset_y is None:
            radius = abs(float(self.env.mug.horizontal_radius))
            y = -radius * float(self.cfg.grasp_edge_fraction)
        else:
            y = float(self.cfg.grasp_offset_y)

        if self.cfg.grasp_offset_z is None:
            bottom_z = float(self.env.mug.bottom_offset[2])
            top_z = float(self.env.mug.top_offset[2])
            z = bottom_z + (top_z - bottom_z) * float(self.cfg.grasp_height_fraction)
        else:
            z = float(self.cfg.grasp_offset_z)

        return np.array([x, y, z], dtype=np.float64)

    def plan_task(self):
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.hang_planned = False
        self.release_planned = False

        world_t_link6 = self._current_link6_world_pose()
        world_t_mug = get_body_pose_by_id(self.env, self.env.mug_body_id)
        world_t_tree, _, _, branch_z = self._tree_axes_and_geometry()

        grasp_yaw = grasp_yaw_from_object(world_t_mug, self.cfg.grasp_yaw_offset)
        grasp_local_offset = self._grasp_local_offset()
        self.recorder.computed_grasp_offset = grasp_local_offset.copy()
        grasp_pos = world_t_mug[:3, 3].copy() + world_t_mug[:3, :3] @ grasp_local_offset
        grasp_rot = downward_grasp_rotation(grasp_yaw) @ R.from_euler("x", self.cfg.grasp_roll_offset).as_matrix()
        mug_grasp = make_pose(grasp_pos, grasp_rot)
        mug_grasp[:3, 3] += np.array([0.0, 0.0, self.cfg.grasp_z_offset])

        mug_hover = mug_grasp.copy()
        mug_hover[:3, 3] += np.array([0.0, 0.0, self.cfg.approach_height])

        mug_anchor_world = world_t_mug[:3, 3] + world_t_mug[:3, :3] @ self._mug_hang_local_point()
        target_anchor_z = world_t_tree[2, 3] + branch_z + self.cfg.lift_height
        lift_distance = max(target_anchor_z - mug_anchor_world[2], 0.02)
        mug_lift = mug_grasp.copy()
        mug_lift[:3, 3] += np.array([0.0, 0.0, lift_distance])

        link6_hover = self._target_link6_pose_for_grasp_frame(mug_hover)
        link6_grasp = self._target_link6_pose_for_grasp_frame(mug_grasp)
        link6_lift = self._target_link6_pose_for_grasp_frame(mug_lift)

        speed = max(float(self.cfg.motion_speed), 0.02)
        dist = np.linalg.norm(link6_hover[:3, 3] - world_t_link6[:3, 3])
        self._append_traj(world_t_link6, link6_hover, max(dist / speed, 1.5), OPEN_GRIPPER)
        self._append_traj(
            link6_hover,
            link6_grasp,
            max(self.cfg.approach_height / (0.8 * speed), 0.8),
            OPEN_GRIPPER,
        )
        self._append_pause(link6_grasp, 0.5, CLOSE_GRIPPER, attach_mug=True)
        self._append_traj(
            link6_grasp,
            link6_lift,
            max(lift_distance / speed, 0.8),
            CLOSE_GRIPPER,
            attach_mug=True,
        )
        return True

    def _plan_hang_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        mug_t_link6 = self._held_mug_to_link6_transform()
        _, branch_base_x, branch_tip_x, _ = self._tree_axes_and_geometry()

        final_x = branch_base_x + self.cfg.insert_depth
        pre_insert_x = branch_tip_x + self.cfg.pre_insert_clearance
        y = self.cfg.target_center_y
        z = self.cfg.target_center_z
        transit_clearance = max(float(self.cfg.transit_clearance), 0.0)
        hang_high_tree = np.array([pre_insert_x, y, z + transit_clearance], dtype=np.float64)
        hang_pre_insert_tree = np.array([pre_insert_x, y, z], dtype=np.float64)
        hang_inserted_tree = np.array([final_x, y, z], dtype=np.float64)
        current_anchor_tree = self._mug_anchor_tree()

        mug_high = self._mug_pose_in_tree_frame(hang_high_tree)
        mug_pre_insert = self._mug_pose_in_tree_frame(hang_pre_insert_tree)
        mug_inserted = self._mug_pose_in_tree_frame(hang_inserted_tree)

        link6_high = mug_high @ mug_t_link6
        link6_pre_insert = mug_pre_insert @ mug_t_link6
        link6_inserted = mug_inserted @ mug_t_link6
        if self.cfg.debug_hang_alignment:
            print(
                "[MugHang align plan] "
                f"mug_local={np.round(self._mug_hang_local_point(), 4)}, "
                f"handle_axis={self.cfg.mug_handle_axis}, "
                f"current_anchor_tree={np.round(current_anchor_tree, 4)}, "
                f"entry_tree={np.round([branch_tip_x, y, z], 4)}, "
                f"pre_insert_tree={np.round(hang_pre_insert_tree, 4)}, "
                f"final_tree={np.round(hang_inserted_tree, 4)}",
                flush=True,
            )

        speed = max(float(self.cfg.motion_speed), 0.02)
        dist = np.linalg.norm(link6_high[:3, 3] - world_t_link6[:3, 3])
        self._append_traj(
            world_t_link6,
            link6_high,
            max(dist / speed, 1.5),
            CLOSE_GRIPPER,
            attach_mug=True,
            label="hang_move_to_pre_high" if transit_clearance > 1e-6 else "hang_move_to_pre_insert",
            anchor_start_tree_xyz=current_anchor_tree,
            anchor_end_tree_xyz=hang_high_tree,
        )
        if transit_clearance > 1e-6:
            self._append_pause(
                link6_high,
                0.5 * self.cfg.hang_settle_time,
                CLOSE_GRIPPER,
                attach_mug=True,
                label="hang_settle_pre_high",
                anchor_tree_xyz=hang_high_tree,
            )
            self._append_traj(
                link6_high,
                link6_pre_insert,
                max(transit_clearance / (0.8 * speed), 0.6),
                CLOSE_GRIPPER,
                attach_mug=True,
                label="hang_descend_to_pre_insert",
                anchor_start_tree_xyz=hang_high_tree,
                anchor_end_tree_xyz=hang_pre_insert_tree,
            )
        self._append_pause(
            link6_pre_insert,
            self.cfg.hang_settle_time,
            CLOSE_GRIPPER,
            attach_mug=True,
            label="hang_settle_pre_insert",
            anchor_tree_xyz=hang_pre_insert_tree,
        )
        insert_dist = np.linalg.norm(link6_inserted[:3, 3] - link6_pre_insert[:3, 3])
        self._append_traj(
            link6_pre_insert,
            link6_inserted,
            max(insert_dist / (0.5 * speed), 1.0),
            CLOSE_GRIPPER,
            attach_mug=True,
            label="hang_insert_anchor",
            anchor_start_tree_xyz=hang_pre_insert_tree,
            anchor_end_tree_xyz=hang_inserted_tree,
        )
        self._append_pause(
            link6_inserted,
            self.cfg.hang_settle_time,
            CLOSE_GRIPPER,
            attach_mug=True,
            label="hang_settle_inserted",
            anchor_tree_xyz=hang_inserted_tree,
        )
        self.hang_planned = True
        self.segment_start_time = sim_time

    def _plan_release_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        world_t_tree, _, _, _ = self._tree_axes_and_geometry()

        retreat_vec = (
            world_t_tree[:3, :3] @ np.array([self.cfg.retreat_distance, 0.0, 0.0])
            + np.array([0.0, 0.0, self.cfg.retreat_height])
        )
        link6_retreat = world_t_link6.copy()
        link6_retreat[:3, 3] += retreat_vec

        self._append_pause(world_t_link6, self.cfg.release_pause, OPEN_GRIPPER)
        retreat_dist = np.linalg.norm(link6_retreat[:3, 3] - world_t_link6[:3, 3])
        speed = max(float(self.cfg.motion_speed), 0.02)
        self._append_traj(world_t_link6, link6_retreat, max(retreat_dist / speed, 0.8), OPEN_GRIPPER)
        self._append_pause(link6_retreat, 0.5, OPEN_GRIPPER)
        self.release_planned = True
        self.segment_start_time = sim_time

    def should_attach_mug(self):
        if not self.cfg.attach_mug_after_grasp or self.current_segment_idx >= len(self.segments):
            return False
        return bool(self.segments[self.current_segment_idx].get("attach_mug", False))

    def get_action(self, sim_time):
        if self.current_segment_idx >= len(self.segments):
            if not self.hang_planned:
                self._plan_hang_from_current_state(sim_time)
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
        expected_anchor_tree = self._expected_anchor_tree_for_segment(seg, elapsed)
        if expected_anchor_tree is not None and self.cfg.alignment_correction_gain > 0.0:
            expected_anchor_world = self._tree_point_world(expected_anchor_tree)
            actual_anchor_world = self._mug_anchor_world()
            correction = expected_anchor_world - actual_anchor_world
            correction_norm = np.linalg.norm(correction)
            if correction_norm > self.cfg.alignment_correction_max:
                correction = correction / correction_norm * self.cfg.alignment_correction_max
            world_t_link6_target = world_t_link6_target.copy()
            world_t_link6_target[:3, 3] += float(self.cfg.alignment_correction_gain) * correction

        arm_action = pose_to_action_in_reference(self.recorder.world_t_link6_initial, world_t_link6_target)
        arm_action = np.clip(arm_action, [-1, -1, -1, -np.pi, -np.pi, -np.pi], [1, 1, 1, np.pi, np.pi, np.pi])
        return np.concatenate([arm_action, [seg["gripper"]]])


def create_env(cfg, worker_id=0):
    controller_config = create_eef_pose_controller_config(cfg)
    env_seed = None if cfg.seed is None or cfg.seed < 0 else cfg.seed + worker_id
    shape_id = None if cfg.shape_id in (None, "", "random") else cfg.shape_id
    env = suite.make(
        env_name=cfg.env_name,
        robots=cfg.robot,
        gripper_types=cfg.gripper_type,
        shape_id=shape_id,
        mug_scale=None if cfg.random_mug_scale else cfg.mug_scale,
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


def set_free_joint_pose_from_matrix(env, joint_name, pose):
    quat_xyzw = R.from_matrix(pose[:3, :3]).as_quat()
    quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
    env.sim.data.set_joint_qpos(joint_name, np.concatenate([pose[:3, 3], quat_wxyz]))
    env.sim.data.set_joint_qvel(joint_name, np.zeros(6))
    env.sim.forward()


def hold_mug_attached_to_link6(env, mug_joint, link6_t_mug):
    _, world_t_link6 = get_body_pose(env, LINK6_EEF_BODY)
    world_t_mug = world_t_link6 @ link6_t_mug
    set_free_joint_pose_from_matrix(env, mug_joint, world_t_mug)


def worker_collect(worker_id, shared_counter, lock, cfg):
    env = create_env(cfg, worker_id=worker_id)
    recorder = MugHangDataRecorder(cfg)
    planner = MugHangEEFPlanner(env, recorder)
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

        obs = env._get_observations(force_update=True)
        for _ in range(cfg.settle_steps):
            obs, _, _, _ = env.step(np.zeros(env.action_dim))
        sync_arm_controller_to_current_state(env)
        recorder.set_link6_initial_reference(env)

        recorder.initial_mug_pose = get_body_pose_by_id(env, env.mug_body_id)
        recorder.initial_mug_tree_pose = get_body_pose_by_id(env, env.mug_tree_body_id)

        if not planner.plan_task():
            continue

        sim_time = 0.0
        planner.segment_start_time = sim_time
        success = False
        stable_success_steps = 0
        post_task_wait_steps = 0
        mug_joint = env.mug.joints[0] if getattr(env.mug, "joints", None) else None
        attached_link6_t_mug = None

        for step_idx in range(cfg.max_steps):
            action = planner.get_action(sim_time)
            if action is None:
                if post_task_wait_steps >= cfg.post_task_wait_steps and stable_success_steps <= 0:
                    break
                action = current_link6_hold_action(env, recorder, OPEN_GRIPPER)
                post_task_wait_steps += 1

            attach_mug = planner.should_attach_mug() and mug_joint is not None
            if attach_mug:
                if attached_link6_t_mug is None:
                    _, world_t_link6 = get_body_pose(env, LINK6_EEF_BODY)
                    world_t_mug = get_body_pose_by_id(env, env.mug_body_id)
                    attached_link6_t_mug = np.linalg.inv(world_t_link6) @ world_t_mug
                hold_mug_attached_to_link6(env, mug_joint, attached_link6_t_mug)
            else:
                attached_link6_t_mug = None

            obs, _, _, _ = env.step(action)
            if attach_mug:
                hold_mug_attached_to_link6(env, mug_joint, attached_link6_t_mug)
                obs = env._get_observations(force_update=True)
            sim_time += dt
            recorder.record_frame(env, obs, sim_time, action)
            if cfg.debug_hang_alignment and step_idx % max(int(cfg.alignment_log_interval), 1) == 0:
                report = planner.hang_alignment_report(sim_time)
                if report is not None:
                    status = "WARN" if report["error_norm"] > cfg.alignment_error_warn else "OK"
                    print(
                        f"[Worker {worker_id}] ALIGN {status} step={step_idx} "
                        f"seg={report['label']} t={report['elapsed']:.2f} "
                        f"actual_tree={np.round(report['actual_tree'], 4)} "
                        f"expected_tree={np.round(report['expected_tree'], 4)} "
                        f"err={np.round(report['error_tree'], 4)} "
                        f"|err|={report['error_norm']:.4f} "
                        f"entry_tree={np.round(report['entry_tree'], 4)} "
                        f"final_tree={np.round(report['final_tree'], 4)}",
                        flush=True,
                    )
                handle_report = planner.handle_engagement_report()
                handle_status = "OK" if handle_report["engaged"] else "WARN"
                print(
                    f"[Worker {worker_id}] HANDLE {handle_status} step={step_idx} "
                    f"engaged={handle_report['engaged']} "
                    f"radial={handle_report['radial_error']:.4f} "
                    f"proj={handle_report['projected']:.4f}/{handle_report['branch_len']:.4f} "
                    f"inside={handle_report['inside_segment']} "
                    f"axis_align={handle_report['axis_alignment']:.3f} "
                    f"inner_mug={np.round(handle_report['inner_mug'], 4)} "
                    f"outer_mug={np.round(handle_report['outer_mug'], 4)} "
                    f"anchor_mug={np.round(handle_report['anchor_mug'], 4)}",
                    flush=True,
                )

            handle_report = planner.handle_engagement_report()
            handle_success = handle_report["engaged"] or not cfg.require_handle_engaged
            if env._check_success() and handle_success:
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
                    if cfg.debug_hang_alignment:
                        handle_report = planner.handle_engagement_report()
                        print(
                            f"[Worker {worker_id}] ALIGN FINAL anchor_tree="
                            f"{np.round(planner._mug_anchor_tree(), 4)} "
                            f"handle_engaged={handle_report['engaged']} "
                            f"handle_radial={handle_report['radial_error']:.4f} "
                            f"handle_axis_align={handle_report['axis_alignment']:.3f}",
                            flush=True,
                        )
                    print(f"[Worker {worker_id}] SUCCESS mug -> mug_tree. Saving demo {demo_idx}...", flush=True)
                    recorder.save_success_demo(demo_idx)
        else:
            mug_pos = env.sim.data.body_xpos[env.mug_body_id].copy()
            tree_pos = env.sim.data.body_xpos[env.mug_tree_body_id].copy()
            tree_rot = env.sim.data.body_xmat[env.mug_tree_body_id].reshape(3, 3).copy()
            mug_error = tree_rot.T @ (mug_pos - tree_pos)
            mug_vel = np.linalg.norm(env.sim.data.get_body_xvelp(env.mug.root_body))
            anchor_tree = planner._mug_anchor_tree()
            handle_report = planner.handle_engagement_report()
            print(
                f"[Worker {worker_id}] Episode failed, discarded. "
                f"mug_pos={np.round(mug_pos, 4)}, mug_tree_pos={np.round(tree_pos, 4)}, "
                f"tree_error={np.round(mug_error, 4)}, mug_vel={mug_vel:.4f}, "
                f"anchor_tree={np.round(anchor_tree, 4)}, on_tree={env._on_mug_tree(mug_pos)}, "
                f"handle_engaged={handle_report['engaged']}, "
                f"handle_radial={handle_report['radial_error']:.4f}, "
                f"handle_proj={handle_report['projected']:.4f}/{handle_report['branch_len']:.4f}, "
                f"handle_axis_align={handle_report['axis_alignment']:.3f}",
                flush=True,
            )

    env.close()


def run_main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--render", action="store_true", help="Show MuJoCo viewer instead of running headless.")
    parser.add_argument("--num_demos", type=int, default=50)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--save_dir", type=str, default="mug_hang_eef_pose_demonstrations")
    parser.add_argument("--max_steps", type=int, default=1200)
    parser.add_argument("--record_freq", type=int, default=20)
    parser.add_argument("--control_freq", type=int, default=20)
    parser.add_argument("--seed", type=int, default=-1, help="Use -1 for non-deterministic placement sampling.")
    parser.add_argument("--eef_kp", type=float, default=150.0)
    parser.add_argument("--motion_speed", type=float, default=0.15)
    parser.add_argument("--shape_id", type=str, default="b4ae56d6", help="ShapeNet mug id, or 'random'.")
    parser.add_argument("--mug_scale", type=float, default=1.0)
    parser.add_argument("--random_mug_scale", action="store_true")
    parser.add_argument(
        "--grasp_yaw_offset_deg",
        type=float,
        default=-90.0,
        help="Extra yaw around the downward grasp axis. -90 grips across mug local-x.",
    )
    parser.add_argument(
        "--grasp_roll_offset_deg",
        type=float,
        default=90.0,
        help="Local x-axis roll added to the first mug grasp pose before the normal hang trajectory.",
    )
    parser.add_argument("--grasp_offset_x", type=float, default=0.0)
    parser.add_argument(
        "--grasp_offset_y",
        type=float,
        default=-0.05,
        help="Mug-frame y grasp offset. Use none in code to auto-select the mug body edge opposite the handle.",
    )
    parser.add_argument(
        "--grasp_offset_z",
        type=float,
        default=0.03,
        help="Mug-frame z grasp offset. Use none in code to auto-select an upper side-wall grasp height.",
    )
    parser.add_argument("--grasp_edge_fraction", type=float, default=0.95)
    parser.add_argument("--grasp_height_fraction", type=float, default=0.70)
    parser.add_argument("--grasp_z_offset", type=float, default=0.0)
    parser.add_argument("--approach_height", type=float, default=0.08)
    parser.add_argument(
        "--lift_height",
        type=float,
        default=0.0,
        help="Additional height above the mug-tree branch after grasping; 0 aligns the handle with the branch.",
    )
    parser.add_argument(
        "--transit_clearance",
        type=float,
        default=0.0,
        help="Additional height above the branch while moving to the pre-insert pose.",
    )
    parser.add_argument("--pre_insert_clearance", type=float, default=0.05)
    parser.add_argument("--insert_depth", type=float, default=0.03)
    parser.add_argument("--target_center_y", type=float, default=0.0)
    parser.add_argument("--target_center_z", type=float, default=0.055)
    parser.add_argument("--mug_hang_local_x", type=float, default=0.0)
    parser.add_argument("--mug_hang_local_y", type=float, default=0.03)
    parser.add_argument("--mug_hang_local_z", type=float, default=0.0)
    parser.add_argument("--mug_handle_axis", type=str, default="x", choices=("x", "neg_x", "y", "neg_y"))
    parser.add_argument("--hang_settle_time", type=float, default=0.6)
    parser.add_argument("--release_pause", type=float, default=0.6)
    parser.add_argument("--retreat_distance", type=float, default=0.08)
    parser.add_argument("--retreat_height", type=float, default=0.08)
    parser.add_argument("--success_stable_steps", type=int, default=20)
    parser.add_argument("--post_task_wait_steps", type=int, default=300)
    parser.add_argument("--disable_attach_mug", action="store_true")
    parser.add_argument("--debug_hang_alignment", action="store_true")
    parser.add_argument("--alignment_log_interval", type=int, default=10)
    parser.add_argument("--alignment_error_warn", type=float, default=0.015)
    parser.add_argument("--alignment_correction_gain", type=float, default=2.0)
    parser.add_argument("--alignment_correction_max", type=float, default=0.06)
    parser.add_argument("--disable_handle_check", action="store_true")
    parser.add_argument("--handle_anchor_tolerance", type=float, default=0.018)
    parser.add_argument("--handle_axis_alignment_min", type=float, default=0.75)
    parser.add_argument("--handle_segment_margin", type=float, default=0.005)
    parser.add_argument("--no_video", action="store_true")
    parser.add_argument("--no_camera_obs", action="store_true", help="Disable camera observations for renderer-less tests.")
    parser.add_argument("--max_episodes", type=int, default=0, help="Debug limit per worker. 0 means unlimited.")
    args = parser.parse_args()

    cfg = replace(
        MugHangEEFCollectConfig(),
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
        shape_id=args.shape_id,
        mug_scale=args.mug_scale,
        random_mug_scale=args.random_mug_scale,
        grasp_yaw_offset=np.deg2rad(args.grasp_yaw_offset_deg),
        grasp_roll_offset=np.deg2rad(args.grasp_roll_offset_deg),
        grasp_offset_x=args.grasp_offset_x,
        grasp_offset_y=args.grasp_offset_y,
        grasp_offset_z=args.grasp_offset_z,
        grasp_edge_fraction=args.grasp_edge_fraction,
        grasp_height_fraction=args.grasp_height_fraction,
        grasp_z_offset=args.grasp_z_offset,
        approach_height=args.approach_height,
        lift_height=args.lift_height,
        transit_clearance=args.transit_clearance,
        pre_insert_clearance=args.pre_insert_clearance,
        insert_depth=args.insert_depth,
        target_center_y=args.target_center_y,
        target_center_z=args.target_center_z,
        mug_hang_local_x=args.mug_hang_local_x,
        mug_hang_local_y=args.mug_hang_local_y,
        mug_hang_local_z=args.mug_hang_local_z,
        mug_handle_axis=args.mug_handle_axis,
        hang_settle_time=args.hang_settle_time,
        release_pause=args.release_pause,
        retreat_distance=args.retreat_distance,
        retreat_height=args.retreat_height,
        success_stable_steps=args.success_stable_steps,
        post_task_wait_steps=args.post_task_wait_steps,
        attach_mug_after_grasp=not args.disable_attach_mug,
        debug_hang_alignment=args.debug_hang_alignment,
        alignment_log_interval=args.alignment_log_interval,
        alignment_error_warn=args.alignment_error_warn,
        alignment_correction_gain=args.alignment_correction_gain,
        alignment_correction_max=args.alignment_correction_max,
        require_handle_engaged=not args.disable_handle_check,
        handle_anchor_tolerance=args.handle_anchor_tolerance,
        handle_axis_alignment_min=args.handle_axis_alignment_min,
        handle_segment_margin=args.handle_segment_margin,
        no_video=args.no_video,
        use_camera_obs=not args.no_camera_obs,
        max_episodes=args.max_episodes,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"Saving MugHang EEF_POSE demos to {cfg.save_dir} ({timestamp})", flush=True)

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
