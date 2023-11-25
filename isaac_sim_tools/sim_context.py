""" 仿真启动后可以使用的一些配置工具 """
# The following items must be imported after Simulation is initialized
import omni
import carb
from omni.isaac.core import World,SimulationContext
from omni.isaac.core.utils.stage import is_stage_loading,get_current_stage,open_stage
from omni.isaac.core.scenes.scene import Scene
import omni.timeline
from omni.isaac.core.utils.nucleus import is_file
import omni.kit.app
from omni.isaac.core.physics_context import PhysicsContext


class IsaacContext(object):
    """ IsaacSim常用接口工具类 """
    __app = None
    __stage = None
    __world = None
    __scene = None
    __reset_handler = None

    def __init__(self,app, world, reset_handler) -> None:
        self.__app = app
        self.__world = world
        self.__reset_handler = reset_handler

    @classmethod
    def context_sense(cls,sense_world=False):
        """ 获得仿真上下文 """
        cls.__app = omni.kit.app.get_app()
        cls.__stage = get_current_stage()
        cls.__scene = Scene()
        if sense_world:
            cls.__world = cls.get_current_sim_context()

    @classmethod
    def set_sim_app(cls, isaac_app):
        """ 设置仿真app对象 """
        cls.__app = isaac_app

    @classmethod
    def set_world_context(cls,world_context):
        """ 设置仿真上下文 """
        cls.__world = world_context

    @classmethod
    def set_reset_handler(cls,handler):
        """ 设置仿真重置函数 """
        cls.__reset_handler = handler

    @classmethod
    def init_stage(cls,usd_path=None):
        """ 初始化stage """
        if usd_path is None:
            cls.__stage = get_current_stage()
            return cls.__stage
        # make sure the file exists before we try to open it
        try: result = is_file(usd_path)
        except: result = False
        if result: open_stage(usd_path)  # 开启stage
        else:
            carb.log_error(f"the usd path {usd_path} could not be opened, please make sure that it is a valid usd file.")
            cls.__app.close()
            exit()
        # Wait two frames so that stage starts loading
        cls.__app.update(),cls.__app.update()
        print("Loading stage...")
        while is_stage_loading(): cls.__app.update()
        print("Loading Complete")
        # disable joystick in isaac sim to avoid conflict
        carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)
        cls.__stage = get_current_stage()
        return cls.__stage

    @classmethod
    def get_current_sim_context(cls):
        """ 得到当前的仿真场景或World """
        if cls.__world is not None:
            return cls.__world
        elif World._world_initialized:
            print("World is initialed, will return world istead.")
            return World()
        else: return SimulationContext.instance()

    @classmethod
    def world_init(cls,physics_dt=None,rendering_dt=None,meters_unit=None):
        """ 初始化世界 """
        # use exist world
        if World._world_initialized:
            print("World is initialized and it will be used instead of creating a new one.")
            cls.__world = World()
        else:  # create a new world
            from torch._C import NoneType
            # from omni.isaac.core.utils.stage import get_stage_units
            cls.__world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=meters_unit)
            if(cls.__world is NoneType): carb.log_error("Failed to set world."),exit(0)
            elif not cls.__world._world_initialized:
                carb.log_error("The simulation context is initialed before world, hence world will be the same.")
            else: cls.__app.update()
        cls.__scene = Scene()
        return cls.__world,cls.__scene

    @classmethod
    def world_reset(cls,times=2):
        if cls.__reset_handler is not None: cls.__reset_handler()
        elif isinstance(cls.__world,World):
            for _ in range(times): cls.__world.reset() # reset twice after callback setup
        else:
            context = SimulationContext()
            for _ in range(times):
                context.reset()
                cls.__scene._finalize(context.physics_sim_view)
                cls.__scene.post_reset()

    @classmethod
    def add_physics_callback(cls,call_back_name:str,callback_fn,reset=False):
        """ 添加回调函数 """
        PhysicsContext(set_defaults=False)._physx_interface.subscribe_physics_step_events(callback_fn)
        if reset: cls.add_physics_callback_done()

    @classmethod
    def add_physics_callback_done(cls):
        """ 表明回调函数设置完全 """
        if isinstance(cls.__world,World):
            cls.__world.reset(),cls.__world.reset()  # reset twice after callback setup
        else: cls.world_reset()

    @classmethod
    def run_sim_loop(cls,render=True,robot_and_func=None):
        """ 循环运行仿真 """
        omni.timeline.get_timeline_interface().play()
        while cls.__app.is_running():
            cls.__world.step(render=render)
            if robot_and_func is not None:
                return robot_and_func[1](robot_and_func[0])

    @classmethod
    def run_sim_once(cls,render=True):
        """ 该函数总是与sim_is_running()结合使用，以实现与run_sim_loop一样的效果；必须首先执行一遍run_sim_once """
        if not hasattr(cls.sim_is_running,'first'):
            raise Exception("You must use sim_is_running before run_sim_once!")
        cls.__world.step(render=render)

    @classmethod
    def sim_is_running(cls):
        if not hasattr(cls.sim_is_running,'first'):
            cls.sim_is_running.__dict__['first'] = False
            omni.timeline.get_timeline_interface().play()
        return cls.__app.is_running()

    @classmethod
    def exit_sim(cls,exit_ros=True):
        """ 退出仿真 """
        if exit_ros:
            import rospy
            rospy.signal_shutdown("Sim complete!")
        omni.timeline.get_timeline_interface().stop()
        cls.__app.close()

    @classmethod
    def close_app(cls):
        cls.__app.close(),exit()
