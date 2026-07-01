import numpy as np

import robosuite.utils.transform_utils as T
from robosuite.models.objects import CompositeObject
from robosuite.utils.mjcf_utils import CustomMaterial, add_to_dict


class ShortBinObject(CompositeObject):
    """
    Four-walled short bin target used by BoxInBin.
    """

    def __init__(
        self,
        name,
        bin_size=(0.224, 0.154, 0.039),
        wall_thickness=0.0095,
        transparent_walls=False,
        friction=None,
        density=1000.0,
        use_texture=True,
        rgba=(0.2, 0.1, 0.0, 1.0),
        joints=None,
    ):
        self._name = name
        self.bin_size = np.array(bin_size)
        self.wall_thickness = wall_thickness
        self.transparent_walls = transparent_walls
        self.friction = friction if friction is None else np.array(friction)
        self.density = density
        self.use_texture = use_texture
        self.rgba = rgba
        self.bin_mat_name = "light_wood_mat"
        self._base_geom = "base"
        self._important_sites = {}
        self._joints = joints

        super().__init__(**self._get_geom_attrs())

        tex_attrib = {"type": "cube"}
        mat_attrib = {
            "texrepeat": "3 3",
            "specular": "0.4",
            "shininess": "0.1",
        }
        bin_mat = CustomMaterial(
            texture="WoodLight",
            tex_name="light_wood",
            mat_name=self.bin_mat_name,
            tex_attrib=tex_attrib,
            mat_attrib=mat_attrib,
        )
        self.append_material(bin_mat)

    def _get_geom_attrs(self):
        base_args = {
            "total_size": self.bin_size / 2.0,
            "name": self.name,
            "locations_relative_to_center": True,
            "obj_types": "all",
            "density": self.density,
            "joints": self._joints,
        }
        obj_args = {}

        add_to_dict(
            dic=obj_args,
            geom_types="box",
            geom_locations=(0, 0, -(self.bin_size[2] - self.wall_thickness) / 2),
            geom_quats=(1, 0, 0, 0),
            geom_sizes=(
                np.array((self.bin_size[0], self.bin_size[1], self.wall_thickness))
                - np.array((self.wall_thickness, self.wall_thickness, 0))
            )
            / 2,
            geom_names=self._base_geom,
            geom_rgbas=None if self.use_texture else self.rgba,
            geom_materials=self.bin_mat_name if self.use_texture else None,
            geom_frictions=self.friction,
        )

        x_vals = np.array(
            [0, -(self.bin_size[0] - self.wall_thickness) / 2, 0, (self.bin_size[0] - self.wall_thickness) / 2]
        )
        y_vals = np.array(
            [-(self.bin_size[1] - self.wall_thickness) / 2, 0, (self.bin_size[1] - self.wall_thickness) / 2, 0]
        )
        w_vals = np.array([self.bin_size[0], self.bin_size[1], self.bin_size[0], self.bin_size[1]])
        r_vals = np.array([np.pi / 2, 0, -np.pi / 2, np.pi])

        if self.transparent_walls:
            wall_rgba = (1.0, 1.0, 1.0, 0.3)
            wall_mat = None
        else:
            wall_rgba = None if self.use_texture else self.rgba
            wall_mat = self.bin_mat_name if self.use_texture else None

        for i, (x, y, w, r) in enumerate(zip(x_vals, y_vals, w_vals, r_vals)):
            add_to_dict(
                dic=obj_args,
                geom_types="box",
                geom_locations=(x, y, 0),
                geom_quats=T.convert_quat(T.axisangle2quat(np.array([0, 0, r])), to="wxyz"),
                geom_sizes=(self.wall_thickness / 2, w / 2, self.bin_size[2] / 2),
                geom_names=f"wall{i}",
                geom_rgbas=wall_rgba,
                geom_materials=wall_mat,
                geom_frictions=self.friction,
            )

        obj_args.update(base_args)
        return obj_args

    @property
    def base_geoms(self):
        return [self.correct_naming(self._base_geom)]


class MugTreeObject(CompositeObject):
    """
    Fixed mug hanger target used by MugHang.
    """

    def __init__(
        self,
        name,
        base_size=(0.16, 0.16, 0.03),
        branch_height=0.12,
        branch_size=(0.08, 0.005, 0.015),
        tree_size=(0.03, 0.03, 0.16),
        friction=None,
        density=5000.0,
        use_texture=True,
        rgba=(0.2, 0.1, 0.0, 1.0),
        joints=None,
    ):
        self._name = name
        self.base_size = np.array(base_size)
        self.branch_height = branch_height
        self.branch_size = np.array(branch_size)
        self.tree_size = np.array(tree_size)
        self.friction = friction if friction is None else np.array(friction)
        self.density = density
        self.use_texture = use_texture
        self.rgba = rgba
        self.tree_mat_name = "light_wood_mat"
        self._base_geom = "base"
        self._important_sites = {}
        self._joints = joints

        super().__init__(**self._get_geom_attrs())

        tex_attrib = {"type": "cube"}
        mat_attrib = {
            "texrepeat": "3 3",
            "specular": "0.4",
            "shininess": "0.1",
        }
        tree_mat = CustomMaterial(
            texture="WoodLight",
            tex_name="light_wood",
            mat_name=self.tree_mat_name,
            tex_attrib=tex_attrib,
            mat_attrib=mat_attrib,
        )
        self.append_material(tree_mat)

    def _get_geom_attrs(self):
        total_height = self.base_size[2] + self.tree_size[2]
        base_args = {
            "total_size": (self.base_size + np.array([0, 0, self.tree_size[2]])) / 2.0,
            "name": self.name,
            "locations_relative_to_center": True,
            "obj_types": "all",
            "density": self.density,
            "joints": self._joints,
        }
        obj_args = {}

        add_to_dict(
            dic=obj_args,
            geom_types="box",
            geom_locations=(0, 0, self.base_size[2] / 2 - total_height / 2),
            geom_quats=(1, 0, 0, 0),
            geom_sizes=self.base_size / 2,
            geom_names=self._base_geom,
            geom_rgbas=None if self.use_texture else self.rgba,
            geom_materials=self.tree_mat_name if self.use_texture else None,
            geom_frictions=self.friction,
        )
        add_to_dict(
            dic=obj_args,
            geom_types="box",
            geom_locations=(0, 0, self.base_size[2] + self.tree_size[2] / 2 - total_height / 2),
            geom_quats=(1, 0, 0, 0),
            geom_sizes=self.tree_size / 2,
            geom_names="tree",
            geom_rgbas=None if self.use_texture else self.rgba,
            geom_materials=self.tree_mat_name if self.use_texture else None,
            geom_frictions=self.friction,
        )
        add_to_dict(
            dic=obj_args,
            geom_types="box",
            geom_locations=(
                self.tree_size[0] / 2 + self.branch_size[0] / 2,
                0,
                self.base_size[2] + self.branch_height - total_height / 2,
            ),
            geom_quats=(1, 0, 0, 0),
            geom_sizes=self.branch_size / 2,
            geom_names="branch",
            geom_rgbas=None if self.use_texture else self.rgba,
            geom_materials=self.tree_mat_name if self.use_texture else None,
            geom_frictions=self.friction,
        )

        obj_args.update(base_args)
        return obj_args

    @property
    def base_geoms(self):
        return [self.correct_naming(self._base_geom)]
