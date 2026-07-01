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
    MinJerkPoseTrajectory,
)
from robosuite.utils.transform_utils import convert_quat

logging.getLogger("moviepy").setLevel(logging.ERROR)


@dataclass
class BoxInBinEEFCollectConfig:
    robot: str = "Arx5"
    env_name: str = "BoxInBin"
    gripper_type: str = "ArxGripper"
    control_freq: int = 20
    record_freq: int = 20
    save_dir: str = "box_in_bin_eef_pose_demonstrations"
    img_size: tuple = (640, 480)
    save_size: tuple = (350, 350)
    num_demos: int = 50
    workers: int = 1
    headless: bool = True
    max_steps: int = 1000
    settle_steps: int = 20
    no_video: bool = False
    use_camera_obs: bool = True
    max_episodes: int = 0
    seed: int = -1
    eef_kp: float = 150.0
    motion_speed: float = 0.12
    grasp_yaw_offset: float = 0.0
    approach_height: float = 0.18
    lift_height: float = 0.12
    place_hover_height: float = 0.16
    box_release_height: float = 0.075
    retreat_height: float = 0.12
    success_stable_steps: int = 20
    camera_pos_noise_std: float = 0.0
    camera_ori_noise_std: float = 0.0


class BoxInBinDataRecorder:
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
            "box_positions": [],
            "box_quats": [],
            "bin_positions": [],
            "bin_quats": [],
            "_timestamps": [],
        }
        self.video_frames = []
        self.last_record_time = -1.0
        self.world_t_link6_initial = None
        self.link6_initial_t_world = None
        self.link6_initial_body_name = None
        self.ee_body_name = None
        self.applied_initial_gripper_qpos = None
        self.initial_box_pose = None
        self.initial_bin_pose = None

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

        self.data["box_positions"].append(env.sim.data.body_xpos[env.box_body_id].copy())
        self.data["box_quats"].append(self._body_quat_xyzw(env, env.box_body_id))
        self.data["bin_positions"].append(env.sim.data.body_xpos[env.bin_body_id].copy())
        self.data["bin_quats"].append(self._body_quat_xyzw(env, env.bin_body_id))

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
                root.attrs["task_object"] = "box"
                root.attrs["target_object"] = "bin"
                root.attrs["fixed_initial_arm_qpos"] = FIXED_INITIAL_QPOS
                root.attrs["fixed_initial_full_qpos"] = FIXED_INITIAL_FULL_QPOS
                root.attrs["requested_initial_gripper_qpos"] = FIXED_INITIAL_GRIPPER_QPOS
                root.attrs["grasp_frame_offset_pos_link6"] = GRASP_FRAME_OFFSET_POS
                root.attrs["grasp_frame_offset_rot_link6"] = GRASP_FRAME_OFFSET_ROT
                root.attrs["grasp_yaw_offset_rad"] = self.cfg.grasp_yaw_offset
                root.attrs["grasp_yaw_offset_deg"] = np.rad2deg(self.cfg.grasp_yaw_offset)
                root.attrs["grasp_yaw_folded_to_half_turn"] = True
                root.attrs["approach_height"] = self.cfg.approach_height
                root.attrs["lift_height"] = self.cfg.lift_height
                root.attrs["place_hover_height"] = self.cfg.place_hover_height
                root.attrs["box_release_height"] = self.cfg.box_release_height
                root.attrs["success_stable_steps"] = self.cfg.success_stable_steps
                root.attrs["motion_speed"] = self.cfg.motion_speed
                if self.applied_initial_gripper_qpos is not None:
                    root.attrs["applied_initial_gripper_qpos"] = self.applied_initial_gripper_qpos
                if self.initial_box_pose is not None:
                    root.attrs["initial_box_pose"] = self.initial_box_pose
                if self.initial_bin_pose is not None:
                    root.attrs["initial_bin_pose"] = self.initial_bin_pose
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
                extra_group.create_dataset("box_positions", data=np.array(self.data["box_positions"]))
                extra_group.create_dataset("box_quats", data=np.array(self.data["box_quats"]))
                extra_group.create_dataset("bin_positions", data=np.array(self.data["bin_positions"]))
                extra_group.create_dataset("bin_quats", data=np.array(self.data["bin_quats"]))
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


class BoxInBinEEFPlanner:
    def __init__(self, env, recorder):
        self.env = env
        self.recorder = recorder
        self.cfg = recorder.cfg
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.place_hover_planned = False
        self.release_planned = False

    def _current_link6_world_pose(self):
        _, pose = get_body_pose(self.env, LINK6_EEF_BODY)
        return pose

    @staticmethod
    def _target_link6_pose_for_grasp_frame(world_t_grasp):
        return world_t_grasp @ T_GRASP_LINK6

    def _append_traj(self, start_pose, end_pose, duration, gripper):
        self.segments.append(
            {
                "traj": MinJerkPoseTrajectory(start_pose, end_pose, duration),
                "duration": duration,
                "gripper": gripper,
                "pause": False,
            }
        )

    def _append_pause(self, hold_pose, duration, gripper):
        self.segments.append(
            {
                "hold_pose": hold_pose.copy(),
                "duration": duration,
                "gripper": gripper,
                "pause": True,
            }
        )

    def plan_task(self):
        self.segments = []
        self.current_segment_idx = 0
        self.segment_start_time = 0.0
        self.place_hover_planned = False
        self.release_planned = False

        world_t_link6 = self._current_link6_world_pose()
        world_t_box = get_body_pose_by_id(self.env, self.env.box_body_id)

        grasp_yaw = grasp_yaw_from_object(world_t_box, self.cfg.grasp_yaw_offset)
        box_grasp = make_pose(world_t_box[:3, 3].copy(), downward_grasp_rotation(grasp_yaw))

        box_hover = box_grasp.copy()
        box_hover[:3, 3] += np.array([0.0, 0.0, self.cfg.approach_height])

        box_lift = box_grasp.copy()
        box_lift[:3, 3] += np.array([0.0, 0.0, self.cfg.lift_height])

        link6_hover = self._target_link6_pose_for_grasp_frame(box_hover)
        link6_grasp = self._target_link6_pose_for_grasp_frame(box_grasp)
        link6_lift = self._target_link6_pose_for_grasp_frame(box_lift)

        speed = max(float(self.cfg.motion_speed), 0.02)
        dist = np.linalg.norm(link6_hover[:3, 3] - world_t_link6[:3, 3])
        self._append_traj(world_t_link6, link6_hover, max(dist / speed, 3.0), OPEN_GRIPPER)
        self._append_traj(link6_hover, link6_grasp, 1.8, OPEN_GRIPPER)
        self._append_pause(link6_grasp, 0.8, CLOSE_GRIPPER)
        self._append_traj(link6_grasp, link6_lift, 2.0, CLOSE_GRIPPER)
        return True

    def _held_box_to_link6_transform(self):
        world_t_link6 = self._current_link6_world_pose()
        world_t_box = get_body_pose_by_id(self.env, self.env.box_body_id)
        link6_t_box = np.linalg.inv(world_t_link6) @ world_t_box
        return np.linalg.inv(link6_t_box)

    def _target_box_pose_at_bin(self, z_offset_from_bin_center, box_rot):
        world_t_bin = get_body_pose_by_id(self.env, self.env.bin_body_id)
        pos = world_t_bin[:3, 3].copy()
        pos += world_t_bin[:3, :3] @ np.array([0.0, 0.0, z_offset_from_bin_center])
        return make_pose(pos, box_rot)

    def _plan_place_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        world_t_box = get_body_pose_by_id(self.env, self.env.box_body_id)
        box_t_link6 = self._held_box_to_link6_transform()
        box_rot = world_t_box[:3, :3].copy()

        box_hover = self._target_box_pose_at_bin(self.cfg.place_hover_height, box_rot)
        link6_hover = box_hover @ box_t_link6

        speed = max(float(self.cfg.motion_speed), 0.02)
        dist = np.linalg.norm(link6_hover[:3, 3] - world_t_link6[:3, 3])
        self._append_traj(world_t_link6, link6_hover, max(dist / speed, 3.0), CLOSE_GRIPPER)
        self.place_hover_planned = True
        self.segment_start_time = sim_time

    def _plan_release_from_current_state(self, sim_time):
        world_t_link6 = self._current_link6_world_pose()
        world_t_box = get_body_pose_by_id(self.env, self.env.box_body_id)
        box_t_link6 = self._held_box_to_link6_transform()
        box_rot = world_t_box[:3, :3].copy()

        box_release = self._target_box_pose_at_bin(self.cfg.box_release_height, box_rot)
        link6_release = box_release @ box_t_link6
        link6_retreat = link6_release.copy()
        link6_retreat[:3, 3] += np.array([0.0, 0.0, self.cfg.retreat_height])

        self._append_traj(world_t_link6, link6_release, 1.8, CLOSE_GRIPPER)
        self._append_pause(link6_release, 1.0, OPEN_GRIPPER)
        self._append_traj(link6_release, link6_retreat, 1.2, OPEN_GRIPPER)
        self._append_pause(link6_retreat, 1.0, OPEN_GRIPPER)
        self.release_planned = True
        self.segment_start_time = sim_time

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


def worker_collect(worker_id, shared_counter, lock, cfg):
    env = create_env(cfg, worker_id=worker_id)
    recorder = BoxInBinDataRecorder(cfg)
    planner = BoxInBinEEFPlanner(env, recorder)
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

        recorder.initial_box_pose = get_body_pose_by_id(env, env.box_body_id)
        recorder.initial_bin_pose = get_body_pose_by_id(env, env.bin_body_id)

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

            obs, _, _, _ = env.step(action)
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
                    print(f"[Worker {worker_id}] SUCCESS box -> bin. Saving demo {demo_idx}...", flush=True)
                    recorder.save_success_demo(demo_idx)
        else:
            box_pos = env.sim.data.body_xpos[env.box_body_id].copy()
            bin_pos = env.sim.data.body_xpos[env.bin_body_id].copy()
            print(
                f"[Worker {worker_id}] Episode failed, discarded. "
                f"box_pos={np.round(box_pos, 4)}, bin_pos={np.round(bin_pos, 4)}",
                flush=True,
            )

    env.close()


def run_main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--render", action="store_true", help="Show MuJoCo viewer instead of running headless.")
    parser.add_argument("--num_demos", type=int, default=50)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--save_dir", type=str, default="box_in_bin_eef_pose_demonstrations")
    parser.add_argument("--max_steps", type=int, default=1000)
    parser.add_argument("--record_freq", type=int, default=20)
    parser.add_argument("--control_freq", type=int, default=20)
    parser.add_argument("--seed", type=int, default=-1, help="Use -1 for non-deterministic placement sampling.")
    parser.add_argument("--eef_kp", type=float, default=150.0)
    parser.add_argument("--motion_speed", type=float, default=0.12)
    parser.add_argument(
        "--grasp_yaw_offset_deg",
        type=float,
        default=0.0,
        help="Extra yaw around the downward grasp axis. 0 closes along box local-y.",
    )
    parser.add_argument("--approach_height", type=float, default=0.18)
    parser.add_argument("--lift_height", type=float, default=0.12)
    parser.add_argument("--place_hover_height", type=float, default=0.16)
    parser.add_argument("--box_release_height", type=float, default=0.075)
    parser.add_argument("--retreat_height", type=float, default=0.12)
    parser.add_argument("--success_stable_steps", type=int, default=20)
    parser.add_argument("--no_video", action="store_true")
    parser.add_argument("--no_camera_obs", action="store_true", help="Disable camera observations for renderer-less tests.")
    parser.add_argument("--max_episodes", type=int, default=0, help="Debug limit per worker. 0 means unlimited.")
    args = parser.parse_args()

    cfg = replace(
        BoxInBinEEFCollectConfig(),
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
        approach_height=args.approach_height,
        lift_height=args.lift_height,
        place_hover_height=args.place_hover_height,
        box_release_height=args.box_release_height,
        retreat_height=args.retreat_height,
        success_stable_steps=args.success_stable_steps,
        no_video=args.no_video,
        use_camera_obs=not args.no_camera_obs,
        max_episodes=args.max_episodes,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"Saving BoxInBin EEF_POSE demos to {cfg.save_dir} ({timestamp})", flush=True)

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
