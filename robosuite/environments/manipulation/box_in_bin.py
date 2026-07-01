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
from robosuite.models.objects import BoxObject, ShortBinObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.mjcf_utils import CustomMaterial
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import SequentialCompositeSampler, UniformRandomSampler
from robosuite.utils.transform_utils import convert_quat


class BoxInBin(ManipulationEnv):
    """
    Single-arm task where the robot places a box into a short bin.
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

        self.bin_size = np.array([0.224, 0.154, 0.039]) * 0.5
        self.bin_thickness = 0.0095 * 0.5
        self.box_half_size = np.array([0.113, 0.045, 0.101]) * 0.25
        self.bin_success_margin = 0.015 * 0.5

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
                target=self.box.root_body,
                target_type="body",
                return_distance=True,
            )
            reaching_reward = 1 - np.tanh(10.0 * reach_dist)
            grasping_reward = 0.25 if self._check_grasp(self.robots[0].gripper, self.box) else 0.0
            box_pos = self.sim.data.body_xpos[self.box_body_id]
            bin_pos = self.sim.data.body_xpos[self.bin_body_id]
            placing_dist = np.linalg.norm(box_pos[:2] - bin_pos[:2])
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

        self.bin = ShortBinObject(
            name="bin",
            bin_size=self.bin_size,
            wall_thickness=self.bin_thickness,
            transparent_walls=False,
            density=100.0,
            joints=[dict(type="free", damping="0.0005")],
        )

        tex_attrib = {"type": "cube"}
        mat_attrib = {"texrepeat": "1 1", "specular": "0.4", "shininess": "0.1"}
        redwood = CustomMaterial(
            texture="WoodRed",
            tex_name="redwood",
            mat_name="redwood_mat",
            tex_attrib=tex_attrib,
            mat_attrib=mat_attrib,
        )
        self.box = BoxObject(
            name="box",
            size=self.box_half_size,
            rgba=[1, 0, 0, 1],
            material=redwood,
            rng=self.rng,
        )

        self._get_placement_initializer()

        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[self.bin, self.box],
        )

    def _get_placement_initializer(self):
        if self._provided_placement_initializer:
            self.placement_initializer.reset()
            return

        self.placement_initializer = SequentialCompositeSampler(name="ObjectSampler")
        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="BinSampler",
                mujoco_objects=self.bin,
                x_range=[-0.5, -0.2],
                y_range=[-0.41, 0.0],
                rotation=(-np.pi, np.pi),
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
                name="BoxSampler",
                mujoco_objects=self.box,
                x_range=[-0.5, -0.2],
                y_range=[-0.41, 0.0],
                rotation=(-np.pi, np.pi),
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

        self.bin_body_id = self.sim.model.body_name2id(self.bin.root_body)
        self.box_body_id = self.sim.model.body_name2id(self.box.root_body)
        self.object_body_id = self.box_body_id

    def _setup_observables(self):
        observables = super()._setup_observables()

        if self.use_object_obs:
            modality = "object"

            @sensor(modality=modality)
            def box_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.box_body_id])

            @sensor(modality=modality)
            def box_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.box_body_id]), to="xyzw")

            @sensor(modality=modality)
            def bin_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.bin_body_id])

            @sensor(modality=modality)
            def bin_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.bin_body_id]), to="xyzw")

            @sensor(modality=modality)
            def box_to_bin_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.bin_body_id]) - np.array(
                    self.sim.data.body_xpos[self.box_body_id]
                )

            @sensor(modality=modality)
            def box_in_bin(obs_cache):
                return [float(self._check_success())]

            sensors = [box_pos, box_quat, bin_pos, bin_quat, box_to_bin_pos, box_in_bin]

            arm_prefixes = self._get_arm_prefixes(self.robots[0], include_robot_name=False)
            full_prefixes = self._get_arm_prefixes(self.robots[0])
            sensors += [
                self._get_obj_eef_sensor(full_pf, "box_pos", f"{arm_pf}gripper_to_box_pos", modality)
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
            self._visualize_gripper_to_target(gripper=self.robots[0].gripper, target=self.box)

    def _box_in_bin(self, box_pos):
        bin_pos = np.array(self.sim.data.body_xpos[self.bin_body_id])
        bin_quat = convert_quat(np.array(self.sim.data.body_xquat[self.bin_body_id]), to="xyzw")
        bin_rot_mat = T.quat2mat(bin_quat)
        error = bin_rot_mat.T @ (box_pos - bin_pos)

        return (
            abs(error[0]) < self.bin_size[0] / 2 - self.bin_success_margin
            and abs(error[1]) < self.bin_size[1] / 2 - self.bin_success_margin
            and self.table_offset[2] < box_pos[2] < self.table_offset[2] + self.bin_thickness + self.box_half_size[2] + 0.04
        )

    def _check_success(self):
        box_pos = self.sim.data.body_xpos[self.box_body_id]
        box_in_bin = self._box_in_bin(box_pos)

        try:
            box_grasped = self._check_grasp(self.robots[0].gripper, self.box)
        except Exception:
            box_grasped = False

        return box_in_bin and not box_grasped
