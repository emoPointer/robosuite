import numpy as np
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class Arx5(ManipulatorModel):
    """
    Arx5 is a single-arm robot, typically for customizable mounting on quadruped.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/arx5/robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "NullBase"

    @property
    def default_gripper(self):
        return {"right": "ArxGripper"}  # use ArxGripper

    @property
    def default_controller_config(self):
        return {"right": "default_arx5"}

    @property
    def init_qpos(self):
        # 修改初始关节角度，让末端执行器更接近桌子中心
        return np.array([0.0, -0.1, 0.3, -0.3, 0.0, 0.0])  # 更低和前倾的姿态

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (0.0, 0.0, 0.8),  # 将机器人放在桌子中心上方，高度0.8m
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"
