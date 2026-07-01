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
from robosuite.models.objects import CoffeePodObject, DrawerRL2Object
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import SequentialCompositeSampler, UniformRandomSampler
from robosuite.utils.transform_utils import convert_quat


class Drawer(ManipulationEnv):
    """
    Single-arm task where the robot puts a coffee pod into a drawer and closes it.
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
        self.initial_drawer_qpos = 0.06

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
                target=self.pod.root_body,
                target_type="body",
                return_distance=True,
            )
            reaching_reward = 1 - np.tanh(10.0 * reach_dist)
            grasping_reward = 0.25 if self._check_grasp(self.robots[0].gripper, self.pod) else 0.0
            pod_pos = self.sim.data.body_xpos[self.pod_body_id]
            drawer_pos = self.sim.data.body_xpos[self.drawer_body_id]
            placing_dist = np.linalg.norm(pod_pos[:2] - drawer_pos[:2])
            placing_reward = 1 - np.tanh(10.0 * placing_dist)
            closing_reward = 1 - np.tanh(10.0 * abs(self.sim.data.qpos[self.drawer_qpos_addr]))
            reward = 0.2 * reaching_reward + grasping_reward + 0.35 * placing_reward + 0.2 * closing_reward

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

        self.drawer = DrawerRL2Object(name="drawer")
        self.pod = CoffeePodObject(name="coffee_pod")

        self._get_placement_initializer()

        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[self.drawer, self.pod],
        )

    def _get_placement_initializer(self):
        if self._provided_placement_initializer:
            self.placement_initializer.reset()
            return

        self.placement_initializer = SequentialCompositeSampler(name="ObjectSampler")
        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="DrawerSampler",
                mujoco_objects=self.drawer,
                x_range=[-0.235, -0.205],
                y_range=[-0.292, -0.258],
                rotation=(np.pi - 0.006 * np.pi, np.pi + 0.006 * np.pi),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=False,
                reference_pos=self.table_offset,
                z_offset=0.0,
                rng=self.rng,
            )
        )
        self.placement_initializer.append_sampler(
            UniformRandomSampler(
                name="CoffeePodSampler",
                mujoco_objects=self.pod,
                x_range=[-0.445, -0.345],
                y_range=[-0.105, 0.025],
                rotation=(-0.10 * np.pi, 0.10 * np.pi),
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

        self.drawer_body_id = self.sim.model.body_name2id(self.drawer.root_body)
        self.pod_body_id = self.sim.model.body_name2id(self.pod.root_body)
        self.drawer_qpos_addr = self.sim.model.get_joint_qpos_addr(self.drawer.joints[0])
        self.drawer_qvel_addr = self.sim.model.get_joint_qvel_addr(self.drawer.joints[0])
        self.object_body_id = self.pod_body_id

    def _setup_observables(self):
        observables = super()._setup_observables()

        if self.use_object_obs:
            modality = "object"

            @sensor(modality=modality)
            def coffee_pod_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.pod_body_id])

            @sensor(modality=modality)
            def coffee_pod_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.pod_body_id]), to="xyzw")

            @sensor(modality=modality)
            def drawer_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.drawer_body_id])

            @sensor(modality=modality)
            def drawer_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.drawer_body_id]), to="xyzw")

            @sensor(modality=modality)
            def drawer_qpos(obs_cache):
                return [float(self.sim.data.qpos[self.drawer_qpos_addr])]

            @sensor(modality=modality)
            def pod_to_drawer_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.drawer_body_id]) - np.array(
                    self.sim.data.body_xpos[self.pod_body_id]
                )

            @sensor(modality=modality)
            def pod_in_drawer(obs_cache):
                return [float(self._check_success())]

            sensors = [
                coffee_pod_pos,
                coffee_pod_quat,
                drawer_pos,
                drawer_quat,
                drawer_qpos,
                pod_to_drawer_pos,
                pod_in_drawer,
            ]

            arm_prefixes = self._get_arm_prefixes(self.robots[0], include_robot_name=False)
            full_prefixes = self._get_arm_prefixes(self.robots[0])
            sensors += [
                self._get_obj_eef_sensor(full_pf, "coffee_pod_pos", f"{arm_pf}gripper_to_coffee_pod_pos", modality)
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
                set_object_pose(self.sim, obj, obj_pos, obj_quat, use_model_body_pos=(obj is self.drawer))

            self.sim.forward()

        self.sim.data.qpos[self.drawer_qpos_addr] = self.initial_drawer_qpos + self.rng.uniform(-0.01, 0.01)
        self.sim.data.qvel[self.drawer_qvel_addr] = 0.0
        self.sim.forward()

    def visualize(self, vis_settings):
        super().visualize(vis_settings=vis_settings)

        if vis_settings["grippers"]:
            self._visualize_gripper_to_target(gripper=self.robots[0].gripper, target=self.pod)

    def _drawer_closed(self):
        return self.sim.data.qpos[self.drawer_qpos_addr] < 0.03

    def _pod_in_drawer(self, pod_pos):
        drawer_pos = np.array(self.sim.data.body_xpos[self.drawer_body_id])
        drawer_quat = convert_quat(np.array(self.sim.data.body_xquat[self.drawer_body_id]), to="xyzw")
        drawer_rot_mat = T.quat2mat(drawer_quat)
        error = drawer_rot_mat.T @ (pod_pos - drawer_pos)
        drawer_qpos = self.sim.data.qpos[self.drawer_qpos_addr]

        return (
            abs(error[0] - drawer_qpos) < 0.08
            and abs(error[1]) < 0.08
            and pod_pos[2] < self.table_offset[2] + 0.06
        )

    def _check_success(self):
        pod_pos = self.sim.data.body_xpos[self.pod_body_id]
        return self._drawer_closed() and self._pod_in_drawer(pod_pos)
