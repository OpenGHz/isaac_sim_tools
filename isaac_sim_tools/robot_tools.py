from .basic_tools import IsaacTools
from omni.isaac.core.robots import RobotView, Robot
import numpy as np
from omni.isaac.dynamic_control import _dynamic_control
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


class JointGroup(object):
    """ 关节控制类 """

    def __init__(self, group_name) -> None:
        self.group_name = group_name
        self._joint_names = None
        self._joint_properties = {'drive_mode':[],'stiffness':[],'damping':[],'max_effort':[],'max_velocity':[]}
        self._default_position = None

    @property
    def joint_names(self):
        return copy(self._joint_names)

    @joint_names.setter
    def joint_names(self, joint_names):
        self._joint_names = copy(joint_names)
        self._joint_num = len(joint_names)
        self._default_position = [0] * self._joint_num

    @property
    def joint_num(self):
        return self._joint_num

    @property
    def default_positions(self):
        return deepcopy(self._default_position)

    @default_positions.setter
    def default_positions(self, default_positions):
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
        self.joint_properties['drive_mode'] = copy(drive_mode)

    @property
    def stiffness(self):
        return copy(self.joint_properties['stiffness'])

    @stiffness.setter
    def stiffness(self, stiffness):
        self.joint_properties['stiffness'] = copy(stiffness)

    @property
    def damping(self):
        return copy(self.joint_properties['damping'])
    
    @damping.setter
    def damping(self, damping):
        self.joint_properties['damping'] = copy(damping)

    @property
    def max_effort(self):
        return copy(self.joint_properties['max_effort'])
    
    @max_effort.setter
    def max_effort(self, max_effort):
        self.joint_properties['max_effort'] = copy(max_effort)

    @property
    def max_velocity(self):
        return copy(self.joint_properties['max_velocity'])

    @max_velocity.setter
    def max_velocity(self, max_velocity):
        self.joint_properties['max_velocity'] = copy(max_velocity)


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
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self._articulation_handle = self._dc_interface.get_articulation(robot_prim_path)
        self._wake_up()
        self.joint_num = joint_group.joint_num
        self.group_name = joint_group.group_name
        self._Joint_ptr_X = [self._dc_interface.find_articulation_dof(self._articulation_handle, joint) for joint in joint_group.joint_names]
        self._Joint_properties_X = [_dynamic_control.DofProperties()] * self.joint_num
        self.default_properties()
        self.set_properties(self._process_empty_properties(joint_group.joint_properties))
        self.set_position_target(joint_group.default_positions)
        self._current_positions = joint_group.default_positions
        self._current_velocities = [0] * self.joint_num
        self._current_efforts = [0] * self.joint_num

    def get_name(self):
        return self.group_name

    def default_properties(self):
        self._default_properties = {'drive_mode':'force',
                                   'stiffness':15000,'damping':1400,
                                   'max_effort':5000,'max_velocity':100}
        self._mode_map = {'force':_dynamic_control.DRIVE_FORCE,
                        'velocity':None,
                        'position':None}
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
            self._Joint_properties_X[j].drive_mode = joint_properties['drive_mode'][j]
            self._Joint_properties_X[j].stiffness = joint_properties['stiffness'][j]
            self._Joint_properties_X[j].damping = joint_properties['damping'][j]
            self._Joint_properties_X[j].max_effort = joint_properties['max_effort'][j]
            self._Joint_properties_X[j].max_velocity = joint_properties['max_velocity'][j]
            self._dc_interface.set_dof_properties(self._Joint_ptr_X[j], self._Joint_properties_X[j])

    def set_position_target(self, positions):
        """ 设置关节位置 """
        self._wake_up()
        for j in range(self.joint_num):
            self._dc_interface.set_dof_position_target(self._Joint_ptr_X[j], positions[j])

    def set_velocity_target(self, velocities):
        """ 设置关节速度 """
        self._wake_up()
        for j in range(self.joint_num):
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
