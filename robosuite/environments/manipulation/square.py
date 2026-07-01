import numpy as np

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
from robosuite.models.objects import PegWithBaseObject, SquareNutObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import SequentialCompositeSampler, UniformRandomSampler
from robosuite.utils.transform_utils import convert_quat


class Square(ManipulationEnv):
    """
    Single-arm task where the robot places a square nut onto a square peg.
    """

    def __init__(
        self,
        robots=ARX_DEFAULT_ROBOT,
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
                target=self.nut.root_body,
                target_type="body",
                return_distance=True,
            )
            reaching_reward = 1 - np.tanh(10.0 * reach_dist)
            grasping_reward = 0.25 if self._check_grasp(self.robots[0].gripper, self.nut) else 0.0
            nut_pos = self.sim.data.body_xpos[self.nut_body_id]
            peg_pos = self.sim.data.body_xpos[self.peg_body_id]
            placing_dist = np.linalg.norm(nut_pos[:2] - peg_pos[:2])
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

        self.nut = SquareNutObject(name="square_nut")
        self.peg = PegWithBaseObject(name="peg")

        self._get_placement_initializer()

        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[self.peg, self.nut],
        )

    def _get_placement_initializer(self):
        if self._provided_placement_initializer:
            self.placement_initializer.reset()
            return

        self.placement_initializer = SequentialCompositeSampler(name="ObjectSampler")
        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="PegSampler",
                mujoco_objects=self.peg,
                x_range=[-0.30, -0.24],
                y_range=[-0.16, -0.10],
                rotation=(-np.pi / 36, np.pi / 36),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=False,
                reference_pos=self.table_offset,
                z_offset=0.001,
                rng=self.rng,
            )
        )
        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="SquareNutSampler",
                mujoco_objects=self.nut,
                x_range=[-0.30, -0.24],
                y_range=[-0.30, -0.22],
                rotation=(np.pi - np.pi / 36, np.pi + np.pi / 36),
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

        self.nut_body_id = self.sim.model.body_name2id(self.nut.root_body)
        self.peg_body_id = self.sim.model.body_name2id(self.peg.root_body)
        self.object_body_id = self.nut_body_id

    def _setup_observables(self):
        observables = super()._setup_observables()

        if self.use_object_obs:
            modality = "object"

            @sensor(modality=modality)
            def square_nut_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.nut_body_id])

            @sensor(modality=modality)
            def square_nut_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.nut_body_id]), to="xyzw")

            @sensor(modality=modality)
            def peg_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.peg_body_id])

            @sensor(modality=modality)
            def peg_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.peg_body_id]), to="xyzw")

            @sensor(modality=modality)
            def nut_to_peg_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.peg_body_id]) - np.array(
                    self.sim.data.body_xpos[self.nut_body_id]
                )

            @sensor(modality=modality)
            def nut_on_peg(obs_cache):
                return [float(self._check_success())]

            sensors = [square_nut_pos, square_nut_quat, peg_pos, peg_quat, nut_to_peg_pos, nut_on_peg]

            arm_prefixes = self._get_arm_prefixes(self.robots[0], include_robot_name=False)
            full_prefixes = self._get_arm_prefixes(self.robots[0])
            sensors += [
                self._get_obj_eef_sensor(full_pf, "square_nut_pos", f"{arm_pf}gripper_to_square_nut_pos", modality)
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

            for _ in range(20):
                self.sim.step()
            self.sim.forward()

    def visualize(self, vis_settings):
        super().visualize(vis_settings=vis_settings)

        if vis_settings["grippers"]:
            self._visualize_gripper_to_target(gripper=self.robots[0].gripper, target=self.nut)

    def _nut_on_peg(self, nut_pos, peg_pos):
        xy_close = np.linalg.norm(nut_pos[:2] - peg_pos[:2]) < 0.04
        height_ok = self.table_offset[2] + 0.005 < nut_pos[2] < self.table_offset[2] + 0.12
        return xy_close and height_ok

    def _check_success(self):
        nut_pos = self.sim.data.body_xpos[self.nut_body_id]
        peg_pos = self.sim.data.body_xpos[self.peg_body_id]
        on_peg = self._nut_on_peg(nut_pos, peg_pos)

        try:
            nut_grasped = self._check_grasp(self.robots[0].gripper, self.nut)
        except Exception:
            nut_grasped = False

        return on_peg and not nut_grasped
