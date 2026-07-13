import numpy as np

import robosuite.utils.transform_utils as T
from robosuite.environments.manipulation._arx_task_utils import (
    ARX_CAMERA_NAMES,
    ARX_DEFAULT_BASE,
    ARX_DEFAULT_GRIPPER,
    ARX_DEFAULT_ROBOT,
    ARX_TABLE_FULL_SIZE,
    ARX_TABLE_OFFSET,
    set_arx_base_pose,
    set_object_pose,
)
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects import MugTreeObject, ShapeNetMugObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import SequentialCompositeSampler, UniformRandomSampler
from robosuite.utils.transform_utils import convert_quat


class MugHang(ManipulationEnv):
    """
    Single-arm task where the robot hangs a mug on a mug tree.
    """

    def __init__(
        self,
        robots=ARX_DEFAULT_ROBOT,
        shape_id=None,
        mug_scale=1.0,
        env_configuration="default",
        controller_configs=None,
        gripper_types=ARX_DEFAULT_GRIPPER,
        base_types=ARX_DEFAULT_BASE,
        initialization_noise="default",
        table_full_size=ARX_TABLE_FULL_SIZE,
        table_friction=(1.0, 5e-3, 1e-4),
        use_camera_obs=True,
        use_object_obs=True,
        reward_scale=1.0,
        reward_shaping=False,
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        lite_physics=True,
        horizon=2000,
        ignore_done=True,
        hard_reset=True,
        camera_names=ARX_CAMERA_NAMES,
        camera_heights=480,
        camera_widths=640,
        camera_depths=False,
        camera_segmentations=None,
        renderer="mjviewer",
        renderer_config=None,
        seed=None,
    ):
        self.shape_id = shape_id
        self.mug_scale = mug_scale
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = ARX_TABLE_OFFSET.copy()

        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping
        self.use_object_obs = use_object_obs
        self.placement_initializer = placement_initializer
        self._provided_placement_initializer = placement_initializer is not None

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            base_types=base_types,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            lite_physics=lite_physics,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
            renderer=renderer,
            renderer_config=renderer_config,
            seed=seed,
        )

    def reward(self, action=None):
        reward = 1.0 if self._check_success() else 0.0

        if reward == 0.0 and self.reward_shaping:
            reach_dist = self._gripper_to_target(
                gripper=self.robots[0].gripper,
                target=self.mug.root_body,
                target_type="body",
                return_distance=True,
            )
            reaching_reward = 1 - np.tanh(10.0 * reach_dist)
            grasping_reward = 0.25 if self._check_grasp(self.robots[0].gripper, self.mug) else 0.0
            mug_pos = self.sim.data.body_xpos[self.mug_body_id]
            tree_pos = self.sim.data.body_xpos[self.mug_tree_body_id]
            placing_dist = np.linalg.norm(mug_pos[:2] - tree_pos[:2])
            placing_reward = 1 - np.tanh(10.0 * placing_dist)
            reward = 0.25 * reaching_reward + grasping_reward + 0.5 * placing_reward

        if self.reward_scale is not None:
            reward *= self.reward_scale

        return reward

    def _load_model(self):
        super()._load_model()
        set_arx_base_pose(self.robots[0].robot_model)

        mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )
        mujoco_arena.set_origin([0, 0, 0])

        if self.shape_id is None:
            self.active_shape_id = str(self.rng.choice(ShapeNetMugObject.SHAPE_IDS))
        else:
            self.active_shape_id = self.shape_id
        mug_scale = float(self.rng.uniform(0.9, 1.1)) if self.mug_scale is None else float(self.mug_scale)

        self.mug = ShapeNetMugObject(
            name="mug",
            shape_id=self.active_shape_id,
            scale=mug_scale,
            rgba=(1.0, 0.0, 0.0, 1.0),
        )
        self.mug_tree = MugTreeObject(name="mug_tree", joints=None, use_texture=True)

        self._get_placement_initializer()

        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[self.mug_tree, self.mug],
        )

    def _get_placement_initializer(self):
        if self._provided_placement_initializer:
            self.placement_initializer.reset()
            return

        self.placement_initializer = SequentialCompositeSampler(name="ObjectSampler")
        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="MugTreeSampler",
                mujoco_objects=self.mug_tree,
                x_range=[-0.37, -0.34],
                y_range=[0.01, 0.04],
                rotation=(-np.pi / 2, -np.pi / 2),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.001,
                rng=self.rng,
            )
        )
        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="MugSampler",
                mujoco_objects=self.mug,
                x_range=[-0.35, -0.31],
                y_range=[-0.34, -0.30],
                rotation=(-2 * np.pi / 3, -np.pi / 3),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.001,
                rng=self.rng,
            )
        )

    def _setup_references(self):
        super()._setup_references()

        self.mug_body_id = self.sim.model.body_name2id(self.mug.root_body)
        self.mug_tree_body_id = self.sim.model.body_name2id(self.mug_tree.root_body)
        self.object_body_id = self.mug_body_id

    def _setup_observables(self):
        observables = super()._setup_observables()

        if self.use_object_obs:
            modality = "object"

            @sensor(modality=modality)
            def mug_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.mug_body_id])

            @sensor(modality=modality)
            def mug_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.mug_body_id]), to="xyzw")

            @sensor(modality=modality)
            def mug_tree_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.mug_tree_body_id])

            @sensor(modality=modality)
            def mug_tree_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.mug_tree_body_id]), to="xyzw")

            @sensor(modality=modality)
            def mug_to_tree_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.mug_tree_body_id]) - np.array(
                    self.sim.data.body_xpos[self.mug_body_id]
                )

            @sensor(modality=modality)
            def mug_hanged(obs_cache):
                return [float(self._check_success())]

            sensors = [mug_pos, mug_quat, mug_tree_pos, mug_tree_quat, mug_to_tree_pos, mug_hanged]

            arm_prefixes = self._get_arm_prefixes(self.robots[0], include_robot_name=False)
            full_prefixes = self._get_arm_prefixes(self.robots[0])
            sensors += [
                self._get_obj_eef_sensor(full_pf, "mug_pos", f"{arm_pf}gripper_to_mug_pos", modality)
                for arm_pf, full_pf in zip(arm_prefixes, full_prefixes)
            ]

            for s in sensors:
                observables[s.__name__] = Observable(
                    name=s.__name__,
                    sensor=s,
                    sampling_rate=self.control_freq,
                )

        return observables

    def _reset_internal(self):
        super()._reset_internal()

        if not self.deterministic_reset:
            object_placements = self.placement_initializer.sample()

            for obj_pos, obj_quat, obj in object_placements.values():
                set_object_pose(self.sim, obj, obj_pos, obj_quat)
            self.sim.forward()

    def visualize(self, vis_settings):
        super().visualize(vis_settings=vis_settings)

        if vis_settings["grippers"]:
            self._visualize_gripper_to_target(gripper=self.robots[0].gripper, target=self.mug)

    def _on_mug_tree(self, mug_pos):
        mug_tree_pos = np.array(self.sim.data.body_xpos[self.mug_tree_body_id])
        mug_tree_quat = convert_quat(np.array(self.sim.data.body_xquat[self.mug_tree_body_id]), to="xyzw")
        mug_tree_rot_mat = T.quat2mat(mug_tree_quat)
        error = mug_tree_rot_mat.T @ (mug_pos - mug_tree_pos)
        mug_tree_scale = float(self.mug_tree.tree_size[2] / 0.16)

        return (
            abs(error[0]) < 0.085 * mug_tree_scale
            and abs(error[1]) < 0.05 * mug_tree_scale
            and mug_pos[2] > self.table_offset[2] + 0.11 * mug_tree_scale
        )

    def _check_success(self):
        mug_pos = self.sim.data.body_xpos[self.mug_body_id]
        mug_vel = self.sim.data.get_body_xvelp(self.mug.root_body)
        mug_still = np.linalg.norm(mug_vel) < 0.01
        on_mug_tree = self._on_mug_tree(mug_pos)

        try:
            mug_grasped = self._check_grasp(self.robots[0].gripper, self.mug)
        except Exception:
            mug_grasped = False

        return on_mug_tree and mug_still and not mug_grasped
