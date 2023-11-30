import numpy as np
import omnigibson as og
from omnigibson.scenes import Scene
from isaac_sim_tools import JointGroup, RobotInfo, RobotControl, MoveGroup

joint_cmd_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def main():
    """
    This example showcases how to construct your own IK functionality using omniverse's native lula library
    without explicitly utilizing all of OmniGibson's class abstractions, and also showcases how to manipulate
    the simulator at a lower-level than the main Environment entry point.
    """
    og.log.info(f"Demo {__file__}\n    " + "*" * 80 + "\n    Description:\n" + main.__doc__ + "*" * 80)


    scene = Scene()
    og.sim.import_scene(scene)
    # env = og.Environment(cfg)

    # Update the viewer camera's pose so that it points towards the robot
    og.sim.viewer_camera.set_position_orientation(
        position=np.array([4.32248, -5.74338, 6.85436]),
        orientation=np.array([0.39592, 0.13485, 0.29286, 0.85982]),
    )

    # 关节组信息配置
    airbot_play_arm = JointGroup('airbot_play_arm')
    airbot_play_arm.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    airbot_play_arm.stiffness = 300000
    airbot_play_arm.damping = 3000
    airbot_play_arm.max_effort = 10000000
    # airbot_play_arm.control_mode = ['velocity'] + ['position'] * 5
    # airbot_play_arm.has_limits = True
    airbot_play_gripper = JointGroup('airbot_play_gripper')
    airbot_play_gripper.joint_names = ['joint_gripper10','joint_gripper11','joint_gripper20','joint_gripper21']
    first_2_stif = 30000
    last_2_stif  = 300000000
    first_2_damp = 300
    last_2_damp  = 300
    first_2_eff  = 100000
    last_2_eff   = 100000000000
    airbot_play_gripper.stiffness = [first_2_stif,last_2_stif,first_2_stif,last_2_stif]
    airbot_play_gripper.damping = [first_2_damp,last_2_damp,first_2_damp,last_2_damp]
    airbot_play_gripper.max_effort = [first_2_eff,last_2_eff,first_2_eff,last_2_eff]
    # 机器人信息配置
    # robot_usd_path = "/home/ghz/.local/share/ov/pkg/isaac_sim-2022.2.0/standalone_examples/api/AirbotPlaySim/USD/airbot_play_v3_inspire_eg2_4c_v1.usd"
    robot_usd_path = "/home/ghz/.local/share/ov/pkg/OmniGibson/omnigibson/data/assets/models/airbot_play_with_rm2/urdf/AIRBOT_V3_v2-1/AIRBOT_V3_v2-1.usd"
    robot_prim_path = '/World/airbot_play'
    robot_info = RobotInfo(robot_usd_path, robot_prim_path, 'airbot_player_1')
    robot_info.init_world_pose = ([0, 0, 0.85], [0.0, 0.0, 0.0, 1.0])  # 默认即是这个配置

    # 机器人控制器创建
    airbot_player = RobotControl(robot_info)

    # At least one simulation step while the simulator is playing must occur for the robot (or in general, any object)
    # to be fully initialized after it is imported into the simulator
    og.sim.play()
    og.sim.step()

    # 机器人运动组获取
    airbot_player.create_move_groups([airbot_play_arm, airbot_play_gripper])
    arm_group = airbot_player.get_move_group(airbot_play_arm.group_name)
    gripper_group = airbot_player.get_move_group(airbot_play_gripper.group_name)

    og.sim.step()

    """加入ROS接口"""
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node('test')

    def cmd_callback(joint_cmd:JointState):
        global joint_cmd_position
        joint_cmd_position = list(joint_cmd.position)
        # print(joint_cmd_position)

    name_space = '/airbot_play'
    joint_cmd_sub = rospy.Subscriber(name_space+"/joint_cmd", JointState, cmd_callback, queue_size=10)
    joint_states_pub = rospy.Publisher(name_space+"/joint_states", JointState, queue_size=10)
    joint_states = JointState()
    joint_states.name = airbot_play_arm.joint_names
    joint_states.header.frame_id = 'airbot_play'
    while not rospy.is_shutdown():
        pp_flag = rospy.get_param("/pick_place_flag", default=None)
        if pp_flag == 'pick':
            gripper_pos = 0.6
        else:
            gripper_pos = 0.0
        og.sim.step()
        arm_group.set_position_target(joint_cmd_position)
        gripper_group.set_position_target([-gripper_pos, gripper_pos,
                                           gripper_pos, -gripper_pos])
        joint_position, joint_velocity, joint_effort = arm_group.update_current_states()
        """ 发布ROS话题 """
        joint_states.position = joint_position
        joint_states.velocity = joint_velocity
        joint_states.effort = joint_effort
        joint_states.header.stamp = rospy.Time.now()
        joint_states_pub.publish(joint_states)
        # print("arm_joint_position:",joint_position)
        # print("arm_joint_velocity:",joint_velocity)
        # print("arm_joint_effort:",joint_effort)
        # joint_position, joint_velocity, joint_effort = gripper_group.update_current_states()
        # print("gripper_joint_position:",joint_position)
        # print("gripper_joint_velocity:",joint_velocity)
        # print("gripper_joint_effort:",joint_effort)

    # Always shut the simulation down cleanly at the end
    og.app.close()

def gripper_joint_config():
    gripper_joint_group = JointGroup('airbot_play_gripper')
    gripper_joint_group.joint_names = ['joint_gripper10','joint_gripper11','joint_gripper20','joint_gripper21']
    first_2_stif = 30000
    last_2_stif  = 300000000
    first_2_damp = 300
    last_2_damp  = 300
    first_2_eff  = 100000
    last_2_eff   = 100000000000
    gripper_joint_group.stiffness = [first_2_stif,last_2_stif,first_2_stif,last_2_stif]
    gripper_joint_group.damping = [first_2_damp,last_2_damp,first_2_damp,last_2_damp]
    gripper_joint_group.max_effort = [first_2_eff,last_2_eff,first_2_eff,last_2_eff]
    gripper_move_group = MoveGroup(gripper_joint_group, '/World/robot0')  # 根据JointGroup创建MoveGroup(完成仿真机器人关节参数配置)

if __name__ == "__main__":
    main()
