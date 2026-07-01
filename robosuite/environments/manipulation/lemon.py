import numpy as np

from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects import LemonObject, PlateObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import SequentialCompositeSampler, UniformRandomSampler
from robosuite.utils.sim_utils import check_contact
from robosuite.utils.transform_utils import convert_quat


class Lemon(ManipulationEnv):
    """
    Single-arm task where the robot places a lemon into a plate.
    """

    def __init__(
        self,
        robots="Arx5",
        env_configuration="default",
        controller_configs=None,
        gripper_types="ArxGripper",
        base_types="NullBase",
        initialization_noise="default",
        table_full_size=(1.2, 0.9, 0.05),
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
        camera_names=("external_cam", "robot0_right_eye_in_hand"),
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
        self.table_offset = np.array((0, 0, 0.8))

        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping
        self.use_object_obs = use_object_obs
        self.placement_initializer = placement_initializer
        self._provided_placement_initializer = placement_initializer is not None

        self.success_xy_tolerance = 0.045
        self.success_min_height = 0.005
        self.success_max_height = 0.12

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
                target=self.lemon.root_body,
                target_type="body",
                return_distance=True,
            )
            reaching_reward = 1 - np.tanh(10.0 * reach_dist)
            grasping_reward = 0.25 if self._check_grasp(self.robots[0].gripper, self.lemon) else 0.0
            lemon_pos = self.sim.data.body_xpos[self.lemon_body_id]
            plate_pos = self.sim.data.body_xpos[self.plate_body_id]
            placing_dist = np.linalg.norm(lemon_pos[:2] - plate_pos[:2])
            placing_reward = 1 - np.tanh(10.0 * placing_dist)
            reward = 0.25 * reaching_reward + grasping_reward + 0.5 * placing_reward

        if self.reward_scale is not None:
            reward *= self.reward_scale

        return reward

    def _load_model(self):
        super()._load_model()

        self.robots[0].robot_model.set_base_xpos((-0.535, -0.21, 0.8))

        mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )
        mujoco_arena.set_origin([0, 0, 0])

        self.lemon = LemonObject(name="lemon")
        self.plate = PlateObject(name="plate")

        self._get_placement_initializer()

        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[self.plate, self.lemon],
        )

    def _get_placement_initializer(self):
        if self._provided_placement_initializer:
            self.placement_initializer.reset()
            self.placement_initializer.add_objects([self.plate, self.lemon])
            return

        self.placement_initializer = SequentialCompositeSampler(name="ObjectSampler")

        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="PlateSampler",
                mujoco_objects=self.plate,
                x_range=[-0.4, -0.2],
                y_range=[-0.3, -0.1],
                rotation=(-np.pi, np.pi),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.002,
                rng=self.rng,
            )
        )

        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="LemonSampler",
                mujoco_objects=self.lemon,
                x_range=[-0.4, -0.2],
                y_range=[-0.3, -0.1],
                rotation=(-np.pi, np.pi),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.01,
                rng=self.rng,
            )
        )

    def _setup_references(self):
        super()._setup_references()

        self.lemon_body_id = self.sim.model.body_name2id(self.lemon.root_body)
        self.plate_body_id = self.sim.model.body_name2id(self.plate.root_body)
        self.object_body_id = self.lemon_body_id

    def _setup_observables(self):
        observables = super()._setup_observables()

        if self.use_object_obs:
            modality = "object"

            @sensor(modality=modality)
            def lemon_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.lemon_body_id])

            @sensor(modality=modality)
            def lemon_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.lemon_body_id]), to="xyzw")

            @sensor(modality=modality)
            def plate_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.plate_body_id])

            @sensor(modality=modality)
            def plate_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.plate_body_id]), to="xyzw")

            @sensor(modality=modality)
            def lemon_to_plate_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.plate_body_id]) - np.array(
                    self.sim.data.body_xpos[self.lemon_body_id]
                )

            @sensor(modality=modality)
            def lemon_in_plate(obs_cache):
                return [float(self._check_success())]

            sensors = [lemon_pos, lemon_quat, plate_pos, plate_quat, lemon_to_plate_pos, lemon_in_plate]

            arm_prefixes = self._get_arm_prefixes(self.robots[0], include_robot_name=False)
            full_prefixes = self._get_arm_prefixes(self.robots[0])
            sensors += [
                self._get_obj_eef_sensor(full_pf, "lemon_pos", f"{arm_pf}gripper_to_lemon_pos", modality)
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
                if len(obj.joints) == 0:
                    body_id = self.sim.model.body_name2id(obj.root_body)
                    self.sim.model.body_pos[body_id] = np.array(obj_pos)
                    self.sim.model.body_quat[body_id] = np.array(obj_quat)
                else:
                    self.sim.data.set_joint_qpos(
                        obj.joints[0], np.concatenate([np.array(obj_pos), np.array(obj_quat)])
                    )
            self.sim.forward()

    def visualize(self, vis_settings):
        super().visualize(vis_settings=vis_settings)

        if vis_settings["grippers"]:
            self._visualize_gripper_to_target(gripper=self.robots[0].gripper, target=self.lemon)

    def _check_success(self):
        lemon_pos = self.sim.data.body_xpos[self.lemon_body_id]
        plate_pos = self.sim.data.body_xpos[self.plate_body_id]

        xy_dist = np.linalg.norm(lemon_pos[:2] - plate_pos[:2])
        height_delta = lemon_pos[2] - plate_pos[2]
        in_plate_xy = xy_dist < self.success_xy_tolerance
        in_plate_height = self.success_min_height < height_delta < self.success_max_height
        lemon_plate_contact = check_contact(self.sim, self.lemon, self.plate)

        try:
            lemon_grasped = self._check_grasp(self.robots[0].gripper, self.lemon)
        except Exception:
            lemon_grasped = False

        return in_plate_xy and in_plate_height and lemon_plate_contact and not lemon_grasped
