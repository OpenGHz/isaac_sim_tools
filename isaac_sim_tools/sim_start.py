""" 仿真启动时用 """
from omni.isaac.kit import SimulationApp
import atexit,sys


class IsaacApp(object):

    # simulation default config
    ISAACSIM_CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}
    SIM_CONFIG = {}

    @classmethod
    def config_sim(cls,headless=False,window_size=(1280,720),sync_loads=True,renderer="RayTracedLighting",articulation=True,use_ros=True):
        """ 仿真启动前先进行配置(必须在启动app之前使用)  """
        cls.ISAACSIM_CONFIG['width']       = window_size[0]
        cls.ISAACSIM_CONFIG['height']      = window_size[1]
        cls.ISAACSIM_CONFIG['headless']    = headless  # headless为true则不会启动Isaac Sim软件的GUI界面，可用于无图形化的服务器情况使用。
        cls.ISAACSIM_CONFIG["sync_loads"]  = sync_loads
        cls.ISAACSIM_CONFIG["renderer"]    = renderer  # RTX光线追踪
        cls.SIM_CONFIG["articulation"]= articulation  # 关节机器人仿真通常需要设置为true
        cls.SIM_CONFIG["use_ros"]     = use_ros  # 为true时将自动检查是否已经开启roscore，然后自动启用rosbridge插件

    @classmethod
    def start_sim(cls):
        """ 根据上述配置启动仿真和插件，并进行检查 """
        # Start the omniverse application
        app = SimulationApp(launch_config=cls.ISAACSIM_CONFIG)  # Set up the simulation
        cls.enable_sim_extensions()
        app.update()
        def exit_and_close_app(): app.close()
        atexit.register(exit_and_close_app)
        cls.sim_init_check()
        return app

    @classmethod
    def enable_sim_extensions(cls):
        """ 根据config使能一些插件(必须在启动app之后使用) """
        from omni.isaac.core.utils.extensions import enable_extension
        if cls.ISAACSIM_CONFIG["headless"]: enable_extension("omni.kit.livestream.native")  # enable livestream
        if cls.SIM_CONFIG['articulation']: enable_extension("omni.isaac.articulation_inspector")  # enable articulation inspector to check articulation tree
        if cls.SIM_CONFIG["use_ros"]: enable_extension("omni.isaac.ros_bridge")  # enable ROS bridge extension
        enable_extension("omni.isaac.physics_inspector")  # 始终启用

    @classmethod
    def sim_init_check(cls):
        """ sim启动后进行一些检查 """
        # check if rosmaster node is running
        if cls.SIM_CONFIG["use_ros"] and not ROS_Tools.check_roscore():
            import carb
            carb.log_error("Please run roscore before executing this script.")
            sys.exit()


class ROS_Tools(object):
    """ ROS与Isaac Sim结合的相关工具包 """

    @staticmethod
    def check_roscore()->bool:
        import rosgraph
        return rosgraph.is_master_online()

    @staticmethod
    def init_rosnode(name,use_sim_time=False,anonymous=False,disable_signals=True,start_roscore=False):
        """
            name用于指定ROS节点名，若为''，则不初始化ROS节点，用户可自行在函数外部初始化。若为None，则认为用户不需ROS，不启动rosbridge
            use_sim_time为true则使用仿真时间。
            start_roscore为true时自动启动roscore(勿用，测试不通过)。
        """
        if name not in [None,'']:
            if start_roscore: ROS_Tools._start_roscore()
            import rospy
            if use_sim_time: rospy.set_param("/use_sim_time", True)  # 必须在节点启动前设置该参数
            rospy.init_node(name, anonymous=anonymous, disable_signals=disable_signals,log_level=rospy.ERROR)

    @staticmethod
    def _start_roscore()->None:
        """ 脚本中启动roscore """
        import subprocess
        # 通过roscore启动rosmaster
        p = subprocess.getoutput("pgrep rosmaster")
        if p == '': subprocess.Popen('roscore',start_new_session=True)
        def safe_exit(): subprocess.getoutput("kill %s " % subprocess.getoutput("pgrep rosmaster"))
        atexit.register(safe_exit)

    @staticmethod
    def _use_sim_time():
        import rospy
        rospy.set_param("/use_sim_time", True)  # 必须在节点启动前设置该参数