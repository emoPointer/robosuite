import numpy as np

from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import array_to_string, find_elements, get_elements, new_geom, xml_path_completion


class BottleObject(MujocoXMLObject):
    """
    Bottle object
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/bottle.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class CanObject(MujocoXMLObject):
    """
    Coke can object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/can.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class LemonObject(MujocoXMLObject):
    """
    Lemon object
    """

    def __init__(self, name, scale=(0.75, 1.0, 1.0)):
        super().__init__(
            xml_path_completion("objects/lemon.xml"),
            name=name,
            joints=[dict(type="free", damping="0.01")],
            obj_type="all",
            duplicate_collision_geoms=False,
            scale=scale,
        )


class PlateObject(MujocoXMLObject):
    """
    Plate object.
    """

    def __init__(self, name, scale=0.5):
        super().__init__(
            xml_path_completion("objects/plate.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=False,
            scale=scale,
        )


class PegWithBaseObject(MujocoXMLObject):
    """
    Square peg mounted on a base, used by the Square task.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/peg_with_base.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=False,
        )


class SquareNutThickObject(MujocoXMLObject):
    """
    Thicker square nut variant copied from ot-sim2real.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/square_nut_thick_multi_layer.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle_site"})
        return dic


class DrawerRL2Object(MujocoXMLObject):
    """
    Sliding drawer fixture used by the Drawer task.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/drawer_RL2.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=False,
        )

    @property
    def bottom_offset(self):
        return np.array([0, 0, 0])

    @property
    def top_offset(self):
        return np.array([0, 0, 0.075])

    @property
    def horizontal_radius(self):
        return 0.15


class CoffeePodObject(MujocoXMLObject):
    """
    Coffee pod object used by the Drawer task.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/coffee_pod.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
        )


class ShapeNetMugObject(MujocoXMLObject):
    """
    ShapeNet mug object used by MugHang.
    """

    SHAPE_IDS = ("3143a4ac", "34ae0b61", "d75af64a", "48e260a6", "b4ae56d6")

    def __init__(self, name, shape_id="3143a4ac", scale=1.0, primitive_collision=False):
        assert shape_id in self.SHAPE_IDS, f"Unknown ShapeNet mug id: {shape_id}"
        super().__init__(
            xml_path_completion(f"shapenet_core/mugs/{shape_id}/model.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=False,
            scale=scale,
        )
        self.shape_id = shape_id
        self.primitive_collision = primitive_collision
        if self.primitive_collision:
            self._replace_collision_meshes_with_primitives()
        else:
            self._make_collision_meshes_visible()

    def _make_collision_meshes_visible(self):
        collision_rgba = (0.1, 0.8, 0.2, 0.30)
        for _, geom in get_elements(self._obj, "geom"):
            if geom.get("group") == "0":
                geom.set("rgba", array_to_string(collision_rgba))

    def _replace_collision_meshes_with_primitives(self):
        for parent, geom in list(get_elements(self._obj, "geom")):
            if geom.get("group") == "0":
                parent.remove(geom)

        prefix = self.naming_prefix
        radius = abs(float(self.horizontal_radius))
        bottom_z = float(self.bottom_offset[2])
        top_z = float(self.top_offset[2])
        height = max(top_z - bottom_z, 1e-3)
        center_z = 0.5 * (bottom_z + top_z)

        collision_rgba = (0.1, 0.8, 0.2, 0.35)
        common = dict(
            group=0,
            contype=1,
            conaffinity=1,
            density=100,
            friction=(0.95, 0.3, 0.1),
            solimp=(0.998, 0.998, 0.001),
            solref=(0.001, 1),
            rgba=collision_rgba,
        )

        cup_radius = 0.88 * radius
        self._obj.append(
            new_geom(
                name=f"{prefix}cup_collision",
                type="cylinder",
                size=(cup_radius, 0.5 * height),
                pos=(0.0, 0.0, center_z),
                **common,
            )
        )

        contact_geoms = ["cup_collision"]
        handle_y = max(1.15 * radius, 0.040)
        handle_half_x = 0.95 * radius
        handle_half_z = 0.38 * height
        handle_radius = 0.16 * radius

        handle_segments = (
            ("handle_top_collision", (-handle_half_x, handle_y, handle_half_z), (handle_half_x, handle_y, handle_half_z)),
            (
                "handle_bottom_collision",
                (-handle_half_x, handle_y, -handle_half_z),
                (handle_half_x, handle_y, -handle_half_z),
            ),
            ("handle_left_collision", (-handle_half_x, handle_y, -handle_half_z), (-handle_half_x, handle_y, handle_half_z)),
            (
                "handle_right_collision",
                (handle_half_x, handle_y, -handle_half_z),
                (handle_half_x, handle_y, handle_half_z),
            ),
        )
        for name, start, end in handle_segments:
            contact_geoms.append(name)
            self._obj.append(
                new_geom(
                    name=f"{prefix}{name}",
                    type="capsule",
                    size=(handle_radius,),
                    pos=None,
                    fromto=(*start, *end),
                    **common,
                )
            )
        self._contact_geoms = contact_geoms


class MilkObject(MujocoXMLObject):
    """
    Milk carton object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/milk.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class BreadObject(MujocoXMLObject):
    """
    Bread loaf object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/bread.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class CerealObject(MujocoXMLObject):
    """
    Cereal box object (used in PickPlace)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/cereal.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class SquareNutObject(MujocoXMLObject):
    """
    Square nut object (used in NutAssembly)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/square-nut.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of nut handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle_site"})
        return dic


class RoundNutObject(MujocoXMLObject):
    """
    Round nut (used in NutAssembly)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/round-nut.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of nut handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle_site"})
        return dic


class MilkVisualObject(MujocoXMLObject):
    """
    Visual fiducial of milk carton (used in PickPlace).

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/milk-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class BreadVisualObject(MujocoXMLObject):
    """
    Visual fiducial of bread loaf (used in PickPlace)

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/bread-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class CerealVisualObject(MujocoXMLObject):
    """
    Visual fiducial of cereal box (used in PickPlace)

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/cereal-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class CanVisualObject(MujocoXMLObject):
    """
    Visual fiducial of coke can (used in PickPlace)

    Fiducial objects are not involved in collision physics.
    They provide a point of reference to indicate a position.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/can-visual.xml"),
            name=name,
            joints=None,
            obj_type="visual",
            duplicate_collision_geoms=True,
        )


class PlateWithHoleObject(MujocoXMLObject):
    """
    Square plate with a hole in the center (used in PegInHole)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/plate-with-hole.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class DoorObject(MujocoXMLObject):
    """
    Door with handle (used in Door)

    Args:
        friction (3-tuple of float): friction parameters to override the ones specified in the XML
        damping (float): damping parameter to override the ones specified in the XML
        lock (bool): Whether to use the locked door variation object or not
    """

    def __init__(self, name, friction=None, damping=None, lock=False):
        xml_path = "objects/door.xml"
        if lock:
            xml_path = "objects/door_lock.xml"
        super().__init__(
            xml_path_completion(xml_path), name=name, joints=None, obj_type="all", duplicate_collision_geoms=True
        )

        # Set relevant body names
        self.door_body = self.naming_prefix + "door"
        self.frame_body = self.naming_prefix + "frame"
        self.latch_body = self.naming_prefix + "latch"
        self.hinge_joint = self.naming_prefix + "hinge"

        self.lock = lock
        self.friction = friction
        self.damping = damping
        if self.friction is not None:
            self._set_door_friction(self.friction)
        if self.damping is not None:
            self._set_door_damping(self.damping)

    def _set_door_friction(self, friction):
        """
        Helper function to override the door friction directly in the XML

        Args:
            friction (3-tuple of float): friction parameters to override the ones specified in the XML
        """
        hinge = find_elements(root=self.worldbody, tags="joint", attribs={"name": self.hinge_joint}, return_first=True)
        hinge.set("frictionloss", array_to_string(np.array([friction])))

    def _set_door_damping(self, damping):
        """
        Helper function to override the door friction directly in the XML

        Args:
            damping (float): damping parameter to override the ones specified in the XML
        """
        hinge = find_elements(root=self.worldbody, tags="joint", attribs={"name": self.hinge_joint}, return_first=True)
        hinge.set("damping", array_to_string(np.array([damping])))

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of door handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"handle": self.naming_prefix + "handle"})
        return dic
