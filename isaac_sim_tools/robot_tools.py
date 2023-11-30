from .basic_tools import IsaacTools
from omni.isaac.core.robots import RobotView, Robot
from omni.isaac.dynamic_control import _dynamic_control

import numpy as np
from copy import copy, deepcopy


class RobotTools(object):
    """ 机器人工具类 """

    @classmethod
    def init(cls):
        """ 需首先初始化IsaacTools类 """
        if not IsaacTools._inited: IsaacTools.init()

    @classmethod
    def add_robot_to_scene(cls,usd_path,prim_path,own_name,position=(0,0,0),orientation=(0,0,0,1)) -> RobotView:
        """ 首先将robot（关节对象）添加到stage中，然后将其加入到scene中(通过创建RobotView) """
        IsaacTools.add_usd_to_stage(usd_path, prim_path)
        return cls.create_robot_view(prim_path, own_name, position, orientation)

    @classmethod
    def create_robot_view(cls,prim_path,own_name,position,orientation) -> RobotView:
        """ 将已经存在于stage中的机器人prim对象转换为RobotView对象，加入到scene中 """
        robot_view = RobotView(prim_path,own_name,np.array([list(position)]),orientations=np.array([list(orientation)]))
        IsaacTools.add_prim_to_scene(robot_view)
        return robot_view

    @classmethod
    def add_robot_to_world(cls, usd_path, prim_path, own_name, position=(0,0,0)):
        """ 与add_robot_to_scene()类似，但是不会创建RobotView对象，而是创建Robot对象 """
        IsaacTools.add_usd_to_stage(usd_path, prim_path)
        IsaacTools.add_prim_to_scene(Robot(prim_path,own_name,list(position)))
        return IsaacTools.get_object_from_scene(own_name)

    @classmethod
    def set_robot_world_position(cls, robot_view:RobotView, position=(0,0,0)):
        robot_view.set_world_poses(np.array([list(position)]))

    @classmethod
    def set_robot_world_orientation(cls, robot_view:RobotView, orientation=(0,0,0,1)):
        robot_view.set_world_poses(orientations=np.array([list(orientation)]))

    @classmethod
    def get_surface_gripper(cls,gripper_path):
        """ 获得机器人末端吸盘节点 """
        if not isinstance(gripper_path,str): return gripper_path
        elif '/' in gripper_path: surface_gripper_node = IsaacTools.get_prim_from_stage(gripper_path)
        else: print('错误的夹爪参数输入')
        # 检查节点是否存在
        if not surface_gripper_node.IsValid():
            exit(f"Surface gripper node not found: {gripper_path}")
        else: return surface_gripper_node


class DynamicController(object):
    """ 动力学控制器类 """
    inited = False
    _dc_interface = None
    def __init__(self, robot_prim_path) -> None:
        self.init()
        self.articulation_handle = self._dc_interface.get_articulation(robot_prim_path)
        self._articulations = {}
        # Get information about the structure of the articulation
        self.num_joints = self._dc_interface.get_articulation_joint_count(self.articulation_handle)
        self.num_dofs = self._dc_interface.get_articulation_dof_count(self.articulation_handle)
        self.num_bodies = self._dc_interface.get_articulation_body_count(self.articulation_handle)

    @classmethod
    def init(cls):
        """ 初始化动力学控制器 """
        if not cls.inited:
            cls._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
            cls.inited = True

    def wake_up(self):
        self._dc_interface.wake_up_articulation(self.articulation_handle)

    def add_joint_names(self, joint_names):
        articulation_list = []
        for name in joint_names:
            self._articulations[name] = self._dc_interface.find_articulation_dof(self.articulation_handle, name)
            articulation_list.append(self._articulations[name])
        return articulation_list


class JointGroup(object):
    """ 关节控制类 """

    def __init__(self, group_name, joint_names = None) -> None:
        self.group_name = group_name
        self.joint_names = joint_names
        self._joint_properties = {'drive_mode':[],'stiffness':[],'damping':[],
                                  'max_effort':[],'max_velocity':[],
                                  'has_limits':[],'upper':[],'lower':[]}
        self._default_position = None
        self._control_mode = None

    @property
    def joint_names(self):
        return copy(self._joint_names)

    @joint_names.setter
    def joint_names(self, joint_names):
        """ 需要首先设置 """
        if joint_names is None: return
        self._joint_names = copy(joint_names)
        self._joint_num = len(joint_names)
        self._default_position = [0] * self._joint_num
        self._control_mode = ['position'] * self._joint_num

    @property
    def joint_num(self):
        return self._joint_num

    @property
    def control_mode(self):
        return copy(self._control_mode)

    @control_mode.setter
    def control_mode(self, control_mode):
        if isinstance(control_mode, str):
            control_mode = [control_mode] * self._joint_num
        for mode in control_mode:
            if mode not in ['force','velocity','position']:
                raise Exception('错误的控制模式输入')
        self._control_mode = copy(control_mode)

    @property
    def default_positions(self):
        return deepcopy(self._default_position)

    @default_positions.setter
    def default_positions(self, default_positions):
        if isinstance(default_positions, int) or isinstance(default_positions, float):
            default_positions = [default_positions] * self._joint_num
        self._default_position = deepcopy(default_positions)

    @property
    def joint_properties(self):
        return deepcopy(self._joint_properties)

    @joint_properties.setter
    def joint_properties(self, joint_properties):
        self._joint_properties = deepcopy(joint_properties)

    @property
    def drive_mode(self):
        return copy(self.joint_properties['drive_mode'])

    @drive_mode.setter
    def drive_mode(self, drive_mode):
        if isinstance(drive_mode, str):
            drive_mode = [drive_mode] * self._joint_num
        self._joint_properties['drive_mode'] = copy(drive_mode)

    @property
    def stiffness(self):
        return copy(self.joint_properties['stiffness'])

    @stiffness.setter
    def stiffness(self, stiffness):
        if isinstance(stiffness, int) or isinstance(stiffness, float):
            stiffness = [stiffness] * self._joint_num
        self._joint_properties['stiffness'] = copy(stiffness)

    @property
    def damping(self):
        return copy(self.joint_properties['damping'])
    
    @damping.setter
    def damping(self, damping):
        if isinstance(damping, int) or isinstance(damping, float):
            damping = [damping] * self._joint_num
        self._joint_properties['damping'] = copy(damping)

    @property
    def max_effort(self):
        return copy(self.joint_properties['max_effort'])
    
    @max_effort.setter
    def max_effort(self, max_effort):
        if isinstance(max_effort, int) or isinstance(max_effort, float):
            max_effort = [max_effort] * self._joint_num
        self._joint_properties['max_effort'] = copy(max_effort)

    @property
    def max_velocity(self):
        return copy(self.joint_properties['max_velocity'])

    @max_velocity.setter
    def max_velocity(self, max_velocity):
        if isinstance(max_velocity, int) or isinstance(max_velocity, float):
            max_velocity = [max_velocity] * self._joint_num
        self._joint_properties['max_velocity'] = copy(max_velocity)

    @property
    def has_limits(self):
        return copy(self.joint_properties['has_limits'])

    @has_limits.setter
    def has_limits(self, has_limits):
        if isinstance(has_limits, bool):
            has_limits = [has_limits] * self._joint_num
        self._joint_properties['has_limits'] = copy(has_limits)

    @property
    def upper(self):
        return copy(self.joint_properties['upper'])

    @upper.setter
    def upper(self, upper):
        self._joint_properties['upper'] = copy(upper)

    @property
    def lower(self):
        return copy(self.joint_properties['lower'])

    @lower.setter
    def lower(self, lower):
        self._joint_properties['lower'] = copy(lower)


class RobotInfo(object):
    """ 机器人信息类 """

    def __init__(self,usd_path,prim_path,own_name) -> None:
        self.usd_path = usd_path
        self.prim_path = prim_path
        self.own_name = own_name
        self._position = (0,0,0)
        self._orientation = (0,0,0,1)

    @property
    def init_world_pose(self):
        return self._position, self._orientation

    @init_world_pose.setter
    def init_world_pose(self, pose):
        self._position = pose[0]
        self._orientation = pose[1]


class MoveGroup(object):
    """ MoveGroup """
    def __init__(self, joint_group:JointGroup, robot_prim_path:str) -> None:
        # initialize dynamic control, must be done after world.reset() in which articulations are initialized
        self._robot_prim_path = robot_prim_path
        self.joint_num = joint_group.joint_num
        self.group_name = joint_group.group_name
        self.control_mode = joint_group.control_mode
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self._articulation_handle = self._dc_interface.get_articulation(robot_prim_path)
        # Sanity check -- make sure handle is not invalid handle -- it should only ever be None or a valid integer
        assert self._articulation_handle != _dynamic_control.INVALID_HANDLE, \
            f"Got invalid articulation handle for entity at {self.articulation_root_path}"
        self._wake_up()
        self._Joint_ptr_X = [self._dc_interface.find_articulation_dof(self._articulation_handle, joint) for joint in joint_group.joint_names]
        self._Joint_properties_X = [_dynamic_control.DofProperties()] * self.joint_num
        self.set_default_properties()
        self._properties = self._process_empty_properties(joint_group.joint_properties)
        self.set_properties(self._properties)
        if joint_group.default_positions is not None:
            self.set_position_target(joint_group.default_positions)
        self._current_positions = joint_group.default_positions
        self._current_velocities = [0] * self.joint_num
        self._current_efforts = [0] * self.joint_num

    def get_name(self):
        return self.group_name

    def set_default_properties(self, default_properties=None):
        self._mode_map = {'force':_dynamic_control.DRIVE_FORCE,
                        'acceleration':_dynamic_control.DRIVE_ACCELERATION}
        if default_properties is None:
            self._default_properties = {'drive_mode':'force',
                                    'stiffness':15000,'damping':1400,
                                    'max_effort':5000,'max_velocity':100,
                                    'has_limits':None,'upper':np.pi,'lower':-np.pi}
        else: self._default_properties = default_properties

    def get_default_properties(self):
        return self._default_properties

    def _wake_up(self):
        self._dc_interface.wake_up_articulation(self._articulation_handle)

    def _process_empty_properties(self, joint_properties:dict):
        """ 处理关节属性 """
        for key in joint_properties.keys():
            if len(joint_properties[key]) == 0:
                joint_properties[key] = [self._default_properties[key]] * self.joint_num
            elif len(joint_properties[key]) != self.joint_num:
                raise Exception('关节属性输入错误')
        return joint_properties

    def _convert_drive_mode(self, joint_properties:dict):
        """ 将drive_mode转换为Isaac类型 """
        drive_mode = joint_properties['drive_mode']
        for i in range(self.joint_num):
            drive_mode[i] = self._mode_map[drive_mode[i]]
        joint_properties['drive_mode'] = drive_mode
        return joint_properties

    def set_properties(self, joint_properties):
        """ 设置关节属性 """
        joint_properties = self._convert_drive_mode(deepcopy(joint_properties))
        self._wake_up()
        for j in range(self.joint_num):
            # 速度控制模式下，关节的刚度必须为0
            if self.control_mode[j] == 'velocity':
                joint_properties['stiffness'][j] = 0
            self._Joint_properties_X[j].drive_mode = joint_properties['drive_mode'][j]
            self._Joint_properties_X[j].stiffness = joint_properties['stiffness'][j]
            self._Joint_properties_X[j].damping = joint_properties['damping'][j]
            self._Joint_properties_X[j].max_effort = joint_properties['max_effort'][j]
            self._Joint_properties_X[j].max_velocity = joint_properties['max_velocity'][j]
            # 为None时不额外配置，使用默认值（经测试，这部分的设置没有作用，以USD中设置为准）
            if joint_properties['has_limits'][j] is not None:
                self._Joint_properties_X[j].has_limits = joint_properties['has_limits'][j]
                if joint_properties['has_limits'][j]:
                    self._Joint_properties_X[j].upper = joint_properties['upper'][j]
                    self._Joint_properties_X[j].lower = joint_properties['lower'][j]
            self._dc_interface.set_dof_properties(self._Joint_ptr_X[j], self._Joint_properties_X[j])

    def set_position_target(self, positions):
        """ 设置关节位置目标，无关位置值可以任意设置，如设置为None """
        self._wake_up()
        for j in range(self.joint_num):
            if self.control_mode[j] == 'position':
                self._dc_interface.set_dof_position_target(self._Joint_ptr_X[j], positions[j])

    def set_velocity_target(self, velocities):
        """ 设置关节速度目标，无关速度值可以任意设置，如设置为None """
        self._wake_up()
        for j in range(self.joint_num):
            if self.control_mode[j] == 'velocity':
                self._dc_interface.set_dof_velocity_target(self._Joint_ptr_X[j], velocities[j])

    def update_current_states(self):
        """ 更新并获取当前关节状态 """
        all_joint_states = [None] * self.joint_num
        for j in range(self.joint_num):
            all_joint_states[j] = self._dc_interface.get_dof_state(self._Joint_ptr_X[j], _dynamic_control.STATE_ALL)
            self._current_positions[j] = np.float32(all_joint_states[j].pos)
            self._current_velocities[j] = np.float32(all_joint_states[j].vel)
            self._current_efforts[j]   = np.float32(all_joint_states[j].effort)
        return self.current_positions, self.current_velocities, self.current_efforts

    def get_current_positions(self):
        """ 获取当前关节位置 """
        for j in range(self.joint_num):
            self._current_positions[j] = self._dc_interface.get_dof_state(self._Joint_ptr_X[j],
                                                                          _dynamic_control.STATE_POS)["pos"]
        return self._current_positions

    def get_current_velocities(self):
        """ 获取当前关节速度 """
        for j in range(self.joint_num):
            self._current_velocities[j] = self._dc_interface.get_dof_state(self._Joint_ptr_X[j],
                                                                           _dynamic_control.STATE_VEL)["vel"]
        return self._current_velocities

    def get_current_efforts(self):
        """ 获取当前关节力 """
        for j in range(self.joint_num):
            self._current_efforts[j] = self._dc_interface.get_dof_state(self._Joint_ptr_X[j],
                                                                        _dynamic_control.STATE_EFFORT)["effort"]
        return self._current_efforts

    def get_properties(self):
        """ 获取关节属性 """
        properties = deepcopy(self._properties)
        for j in range(self.joint_num):
            joint_properties = self._dc_interface.get_dof_properties(self._Joint_ptr_X[j])
            properties['drive_mode'][j] = joint_properties.drive_mode
            properties['stiffness'][j] = joint_properties.stiffness
            properties['damping'][j] = joint_properties.damping
            properties['max_effort'][j] = joint_properties.max_effort
            properties['max_velocity'][j] = joint_properties.max_velocity
            properties['has_limits'][j] = joint_properties.has_limits
            properties['upper'][j] = joint_properties.upper
            properties['lower'][j] = joint_properties.lower
        return properties

    @property
    def current_positions(self):
        return self._current_positions

    @property
    def current_velocities(self):
        return self._current_velocities

    @property
    def current_efforts(self):
        return self._current_efforts


class RobotControl(object):
    """ 机器人整体管理与控制类 """

    def __init__(self, info:RobotInfo, joint_groups=None) -> None:
        RobotTools.init()
        self.info = info
        self.robot = RobotTools.add_robot_to_scene(info.usd_path, info.prim_path, info.own_name, *info.init_world_pose)
        self.move_groups = {}
        if joint_groups is not None:
            self.create_move_groups(joint_groups)
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self._articulation_handle = self._dc_interface.get_articulation(info.prim_path)

    def wake_up(self):
        """ 每次仿真循环需先唤醒机器人，然后才能进行控制 """
        self._dc_interface.wake_up_articulation(self._articulation_handle)

    def create_move_group(self, group:JointGroup):
        """ 构建MoveGroup对象 """
        move_group = MoveGroup(group, self.info.prim_path)
        self.move_groups[group.group_name] = move_group

    def create_move_groups(self, groups:list):
        """ 构建多个MoveGroup对象;按照groups中的顺序返回MoveGroup对象 """
        for group in groups:
            self.create_move_group(group)
        return self.move_groups.values()

    def get_move_group(self, group_name) -> MoveGroup:
        """ 根据name获取MoveGroup对象 """
        return self.move_groups[group_name]

    def move_group_num(self):
        """ 机器人全部的MoveGroup数量 """
        return len(self.move_groups)
