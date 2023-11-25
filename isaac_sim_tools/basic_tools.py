""" 仿真启动后可以使用的一些配置工具 """
import random
import numpy as np
from typing import Tuple

# The following items must be imported after Simulation is initialized
from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.objects import DynamicCuboid
import carb


class IsaacTools(object):
    """ IsaacSim常用接口工具类 """
    _stage = None
    _scene = None
    _inited = False

    @classmethod
    def init(cls):
        """ 必须先进行初始化该类然后才能进行类方法的使用 """
        cls._stage = get_current_stage()
        cls._scene = Scene()
        cls._inited = True

    @classmethod
    def add_usd_to_stage(cls,usd_path: str, prim_path: str):
        """ 向当前stage添加USD对象，使其成为prim """
        add_reference_to_stage(usd_path, prim_path)

    @classmethod
    def get_prim_from_stage(cls, prim_path: str):
        """ 从当前stage中获取prim对象 """
        return cls._stage.GetPrimAtPath(prim_path)

    @classmethod
    def check_prim(cls, prim_path):
        """ 获得机器人prim对象 """
        prim = cls.get_prim_from_stage(prim_path)
        if not prim.IsValid():
            carb.log_error("Invalid Robot Prim Path.")
            return None
        else:
            return prim

    @classmethod
    def add_prim_to_scene(cls, prim_path:str):
        """ 向当前scene添加prim(USD对象) """
        cls._scene.add(prim_path)

    @classmethod
    def get_object_from_scene(cls,prim_path:str):
        """ 从当前scene中获取prim对象 """
        return cls._scene.get_object(prim_path)

    @classmethod
    def generate_random_cubes(cls,num):
        """ 随机生成物块参考函数 """
        # 随机生产一些物块  # TODO：在一定空间内随机生成若干物块
        for i in range(num):
            cls._scene.add(
                DynamicCuboid(
                    prim_path="/random_cubes/cube{}".format(i), # The prim path of the cube in the USD stage
                    name="fancy_cube{}".format(i), # The unique name used to retrieve the object from the scene later on
                    translation=np.array([random.random()*0.12-0.03, random.random()*0.12-0.06, 0.0125]), # x和y是随机的，z是固定的0.0125。# most arguments accept mainly numpy arrays.
                    size=0.025, 
                    color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1。这里设置为纯蓝。
                    mass=0.005  # 质量设置为5克
                ))


class Camera(object):
    """ 简化版的仿真相机操作类 """
    def __init__(self,path:str) -> None:
        from omni.isaac.sensor import Camera
        self.camera = Camera(path)
        self.camera.initialize()

    def get_rgba_image(self) -> np.ndarray:
        return self.camera.get_rgba()

    def get_resolution(self) -> Tuple[int, int]:
        return self.camera.get_resolution()

    def set_default_resolution(self):
        self.camera.set_resolution((1280,720))

    def set_resolution(self,res:tuple):
        self.camera.set_resolution(res)
    # camera.get_current_frame()
    # camera.resume()
    # camera.pause()
    # camera.get_resolution()
    # camera.set_clipping_range()
    # camera.get_aspect_ratio()
    # camera.get_dt()
    # camera.get_focal_length()
    # camera.get_frequency()
    # camera.get_focus_distance()
    # camera.get_horizontal_aperture()
    # camera.set_horizontal_aperture()
    # camera.get_intrinsics_matrix()
    # camera.get_lens_aperture()
    # camera.get_projection_mode()
    # camera.get_vertical_aperture()
    # camera.get_vertical_fov()
    # camera.initialize()
    # camera.set_clipping_range()
