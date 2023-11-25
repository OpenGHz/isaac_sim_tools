# Isaac原生平台使用示例

# 命令行参数初始化
import argparse
parser = argparse.ArgumentParser("AIRbotPlay Isaac Sim Config")
parser.add_argument("-hl","--headless",action="store_true",help="Run stage headless")
parser.add_argument('-ust','--use_sim_time',action='store_true',help='Use sim time instead of ROS time')
args, unknown = parser.parse_known_args()

""" *******************************IsaacSimAPP初始化 *******************************"""
from isaac_sim_tools.sim_start import IsaacApp, ROS_Tools
IsaacApp.config_sim(args.headless)  # 先进行配置
app = IsaacApp.start_sim()  # 然后启动仿真
ROS_Tools.init_rosnode("AIRbotPlay_SimROS",use_sim_time=args.use_sim_time)  # 然后配置ROS
from isaac_sim_tools import IsaacContext  # 必须在start_sim()之后导入
IsaacContext.set_sim_app(app)  # 必须set后之后的tools才能正确使用

"""******************************* 仿真场景初始化 ******************************* """
scene_usd_path = "/home/ghz/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/AirbotPlaySim/USD/localhost/NVIDIA/Assets/Isaac/2022.2.0/Isaac/Environments/Simple_Room/simple_room.usd"
IsaacContext.init_stage(scene_usd_path)
world, _ = IsaacContext.world_init(physics_dt=1/200.0, rendering_dt=1/30.0, meters_unit=1.0)  # 200 30 ROG Z14；meters_unit=1.0表示单位为1m，与ROS一致。

from isaac_sim_tools import JointGroup, RobotInfo, RobotControl

# 关节组信息配置
airbot_play_arm = JointGroup('airbot_play_arm')
airbot_play_arm.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
airbot_play_gripper = JointGroup('airbot_play_gripper')
airbot_play_gripper.joint_names = ['joint_gripper10','joint_gripper11','joint_gripper20','joint_gripper21']

# 机器人信息配置
robot_usd_path = "/home/ghz/Work/AIRbotPlay/airbot_play_isaacsim/sim/USD/airbot_play_inspire_eg2_4c_v2.usd"
robot_path = '/World/airbot_play'
robot_info = RobotInfo(robot_usd_path, robot_path, 'airbot_player_1')
robot_info.init_world_pose = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])  # 默认即是这个配置

# 机器人控制器创建
airbot_player = RobotControl(robot_info, joint_groups=None)
airbot_player.create_move_groups([airbot_play_arm, airbot_play_gripper])

# 机器人运动组获取
arm_group = airbot_player.get_move_group(airbot_play_arm.group_name)
gripper_group = airbot_player.get_move_group(airbot_play_gripper.group_name)

# 关节运动控制
arm_group.set_position_target([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
gripper_group.set_position_target([0.0, 0.0, 0.0, 0.0])

# 关节状态获取
current_arm_states = arm_group.update_current_states()
print(arm_group.current_positions)
print(arm_group.current_velocities)
print(arm_group.current_efforts)
current_gripper_states = gripper_group.update_current_states()
print(gripper_group.current_positions)
print(gripper_group.current_velocities)
print(gripper_group.current_efforts)

# 信息提示
print('*******************CONFIGS*******************',flush=True)
print(f'  headless = {args.headless}',flush=True)
print(f'  use_sim_time = {args.use_sim_time}',flush=True)
print(f'  target_scene = {args.target_scene}',flush=True)
print('********************END*********************',flush=True)

# 循环仿真
while IsaacContext.sim_is_running():
    IsaacContext.run_sim_once()  # 更新仿真
# 结束仿真
IsaacContext.exit_sim()
