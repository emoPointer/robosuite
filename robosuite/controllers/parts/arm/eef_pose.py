import numpy as np
from scipy.spatial.transform import Rotation

import robosuite.utils.transform_utils as T
from robosuite.controllers.parts.arm.osc import OperationalSpaceController
from robosuite.utils.control_utils import opspace_matrices, orientation_error, nullspace_torques


class EEFPoseController(OperationalSpaceController):
    """
    End-effector pose controller for ARX-style pose labels.

    Action format is [x, y, z, rx, ry, rz], where xyz is in meters and
    rx / ry / rz is a rotation vector. The pose is expressed in the initial
    reference body's frame, normally link6_initial, and the controlled EEF frame
    is a MuJoCo body, normally link6.

    input_type controls how the six pose values are interpreted:
        - absolute: action is the target pose in link6_initial.
        - delta / relative: action is added to the current link6 pose after the
          current pose is converted into link6_initial.
    """

    def __init__(
        self,
        *args,
        eef_body_name="link6",
        reference_body_name="link6",
        input_ref_frame="link6_initial",
        input_type="absolute",
        use_nullspace_torques=False,
        nullspace_kp=10,
        **kwargs,
    ):
        self.pose_input_type = self._normalize_input_type(input_type)
        self.input_pose_ref_frame = self._normalize_input_ref_frame(input_ref_frame)
        self.eef_body_name = eef_body_name
        self.reference_body_name = reference_body_name
        self.use_nullspace_torques = use_nullspace_torques
        self.nullspace_kp = nullspace_kp
        self._resolved_eef_body_name = None
        self._resolved_reference_body_name = None
        self.world_t_reference_initial = None
        self.reference_initial_t_world = None

        kwargs["control_ori"] = True
        # The parent OSC only accepts base/world. We override all pose-frame
        # handling below, so keep it in a valid parent state.
        kwargs["input_ref_frame"] = "world"
        kwargs["input_type"] = "absolute"
        super().__init__(*args, **kwargs)
        self.input_type = self.pose_input_type

        if self.interpolator_pos is not None or self.interpolator_ori is not None:
            raise NotImplementedError("EEF_POSE does not support interpolation yet.")

    @staticmethod
    def _normalize_input_type(input_type):
        aliases = {
            "abs": "absolute",
            "absolute": "absolute",
            "delta": "delta",
            "relative": "delta",
        }
        normalized = aliases.get(input_type)
        if normalized is None:
            raise ValueError(
                "EEF_POSE input_type must be 'absolute', 'delta', or 'relative', "
                f"got: {input_type}"
            )
        return normalized

    @staticmethod
    def _normalize_input_ref_frame(input_ref_frame):
        aliases = {
            "link6_init": "link6_initial",
            "link6_initial": "link6_initial",
        }
        normalized = aliases.get(input_ref_frame)
        if normalized != "link6_initial":
            raise ValueError(
                "EEF_POSE currently expects input_ref_frame='link6_initial' "
                f"(alias 'link6_init' is accepted), got: {input_ref_frame}"
            )
        return normalized

    def _resolve_body_name(self, short_name):
        candidates = []
        if short_name:
            candidates.append(short_name)
        if self.naming_prefix and short_name and not short_name.startswith(self.naming_prefix):
            candidates.insert(0, f"{self.naming_prefix}{short_name}")

        for name in candidates:
            try:
                self.sim.model.body_name2id(name)
                return name
            except ValueError:
                pass
        raise ValueError(f"Could not find body '{short_name}' in MuJoCo model.")

    @staticmethod
    def _make_pose(pos, rot):
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = pos
        return pose

    def _body_pose(self, body_name):
        pos = np.array(self.sim.data.get_body_xpos(body_name))
        rot = np.array(self.sim.data.get_body_xmat(body_name))
        return self._make_pose(pos, rot)

    def _target_pose_in_world(self):
        if self.world_t_reference_initial is None:
            self.reset_goal()

        reference_t_target = self._make_pose(self.goal_pos, self.goal_ori)
        return self.world_t_reference_initial @ reference_t_target

    def _current_pose_in_reference_initial(self):
        if self.reference_initial_t_world is None:
            self.reset_goal()

        if self._resolved_eef_body_name is None:
            self._resolved_eef_body_name = self._resolve_body_name(self.eef_body_name)

        world_t_eef = self._body_pose(self._resolved_eef_body_name)
        return self.reference_initial_t_world @ world_t_eef

    @staticmethod
    def _pose_to_action(pose):
        rotvec = Rotation.from_matrix(pose[:3, :3]).as_rotvec()
        return np.concatenate([pose[:3, 3], rotvec]).astype(np.float64)

    def _delta_pose_to_absolute_action(self, delta_pose):
        current_pose = self._current_pose_in_reference_initial()
        current_action = self._pose_to_action(current_pose)
        target_action = current_action + np.asarray(delta_pose, dtype=np.float64)
        target_action[3:6] = Rotation.from_rotvec(target_action[3:6]).as_rotvec()
        return target_action

    def _capture_reference_initial(self):
        if self._resolved_reference_body_name is None:
            self._resolved_reference_body_name = self._resolve_body_name(self.reference_body_name)

        self.world_t_reference_initial = self._body_pose(self._resolved_reference_body_name)
        self.reference_initial_t_world = np.linalg.inv(self.world_t_reference_initial)

    def update_reference_data(self):
        if self._resolved_eef_body_name is None:
            self._resolved_eef_body_name = self._resolve_body_name(self.eef_body_name)

        name = self._resolved_eef_body_name
        self.ref_pos[:] = np.array(self.sim.data.get_body_xpos(name))
        self.ref_ori_mat[:, :] = np.array(self.sim.data.get_body_xmat(name))
        self.ref_pos_vel[:] = np.array(self.sim.data.get_body_xvelp(name))
        self.ref_ori_vel[:] = np.array(self.sim.data.get_body_xvelr(name))
        self.J_pos[:, :] = np.array(self.sim.data.get_body_jacp(name).reshape((3, -1))[:, self.qvel_index])
        self.J_ori[:, :] = np.array(self.sim.data.get_body_jacr(name).reshape((3, -1))[:, self.qvel_index])
        self.J_full[:, :] = np.vstack([self.J_pos, self.J_ori])

    def set_goal(self, action):
        """
        Sets the desired link6 pose in the link6_initial frame.
        """
        self.update()

        if self.impedance_mode == "variable":
            damping_ratio, kp, goal_update = action[:6], action[6:12], action[12:]
            self.kp = np.clip(kp, self.kp_min, self.kp_max)
            self.kd = 2 * np.sqrt(self.kp) * np.clip(damping_ratio, self.damping_ratio_min, self.damping_ratio_max)
        elif self.impedance_mode == "variable_kp":
            kp, goal_update = action[:6], action[6:]
            self.kp = np.clip(kp, self.kp_min, self.kp_max)
            self.kd = 2 * np.sqrt(self.kp)
        else:
            goal_update = action

        goal_update = np.clip(np.array(goal_update, dtype=np.float64), self.input_min, self.input_max)
        if self.pose_input_type == "absolute":
            target_action = goal_update[:6]
        elif self.pose_input_type == "delta":
            target_action = self._delta_pose_to_absolute_action(goal_update[:6])
        else:
            raise ValueError(f"Unsupported EEF_POSE input_type: {self.pose_input_type}")

        self.goal_pos = target_action[:3]
        self.goal_ori = Rotation.from_rotvec(target_action[3:6]).as_matrix()

        if self.position_limits is not None:
            self.goal_pos = np.clip(self.goal_pos, self.position_limits[0], self.position_limits[1])
        if self.orientation_limits is not None:
            rotvec = Rotation.from_matrix(self.goal_ori).as_rotvec()
            rotvec = np.clip(rotvec, self.orientation_limits[0], self.orientation_limits[1])
            self.goal_ori = Rotation.from_rotvec(rotvec).as_matrix()

    def run_controller(self):
        """
        Runs OSC on the controlled body frame, with target pose expressed in link6_initial.
        """
        self.update()

        desired_world_pose = self._target_pose_in_world()
        desired_world_pos = desired_world_pose[:3, 3]
        desired_world_ori = desired_world_pose[:3, :3]

        position_error = desired_world_pos - self.ref_pos
        vel_pos_error = -self.ref_pos_vel
        desired_force = np.multiply(position_error, self.kp[0:3]) + np.multiply(vel_pos_error, self.kd[0:3])

        ori_error = orientation_error(desired_world_ori, self.ref_ori_mat)
        vel_ori_error = -self.ref_ori_vel
        desired_torque = np.multiply(ori_error, self.kp[3:6]) + np.multiply(vel_ori_error, self.kd[3:6])

        lambda_full, lambda_pos, lambda_ori, nullspace_matrix = opspace_matrices(
            self.mass_matrix, self.J_full, self.J_pos, self.J_ori
        )

        if self.uncoupling:
            decoupled_force = np.dot(lambda_pos, desired_force)
            decoupled_torque = np.dot(lambda_ori, desired_torque)
            decoupled_wrench = np.concatenate([decoupled_force, decoupled_torque])
        else:
            desired_wrench = np.concatenate([desired_force, desired_torque])
            decoupled_wrench = np.dot(lambda_full, desired_wrench)

        self.torques = np.dot(self.J_full.T, decoupled_wrench)
        if self.use_torque_compensation:
            self.torques += self.torque_compensation

        if self.use_nullspace_torques:
            self.torques += nullspace_torques(
                self.mass_matrix,
                nullspace_matrix,
                self.initial_joint,
                self.joint_pos,
                self.joint_vel,
                joint_kp=self.nullspace_kp,
            )

        super(OperationalSpaceController, self).run_controller()
        return self.torques

    def reset_goal(self, goal_update_mode="achieved"):
        """
        Captures the current reference body as link6_initial and sets target pose to identity.
        """
        self.update(force=True)
        self._capture_reference_initial()
        self.goal_pos = np.zeros(3)
        self.goal_ori = np.eye(3)

        assert goal_update_mode in ["achieved", "desired"]
        self._goal_update_mode = goal_update_mode

    def delta_to_abs_action(self, delta_ac, goal_update_mode):
        if goal_update_mode != "achieved":
            raise NotImplementedError("EEF_POSE delta conversion currently uses the achieved current link6 pose.")
        return self._delta_pose_to_absolute_action(np.asarray(delta_ac, dtype=np.float64)[:6])

    @property
    def name(self):
        return "EEF_POSE"
