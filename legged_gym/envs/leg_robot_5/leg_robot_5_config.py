from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class LegRobot5Cfg(LeggedRobotCfg):
    class env:
        num_envs = 4096
        num_observations = 235
        num_privileged_obs = None # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 12
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds

    class commands:
        curriculum = False
        max_curriculum = 1.
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = True # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [-1.0, 1.0] # min max [m/s]
            lin_vel_y = [-1.0, 1.0]   # min max [m/s]
            ang_vel_yaw = [-1, 1]    # min max [rad/s]
            heading = [-3.14, 3.14]

    class init_state:
        pos = [0.1, 0.1, 1.] # x,y,z [m]
        rot = [0.1, 0.1, 0.1, 1.0] # x,y,z,w [quat]
        lin_vel = [0.1, 0.1, 0.1]  # x,y,z [m/s]
        ang_vel = [0.1, 0.1, 0.1]  # x,y,z [rad/s]
        default_joint_angles = { # target angles when action = 0.0
            "left_joint1": 0.1, 
            "left_joint2": 0.1,
            "left_joint3": 0.1,
            "left_joint4": 0.1,
            "left_joint5": 0.1,
            "left_joint6": 0.1,

            "right_joint1": 0.1,
            "right_joint2": 0.1,
            "right_joint3": 0.1,
            "right_joint4": 0.1,
            "right_joint5": 0.1,
            "right_joint6": 0.1
            }

    class control:
        control_type = 'P' # P: position, V: velocity, T: torques
        # PD Drive parameters:
        stiffness = {
            "left_joint1": 10., 
            "left_joint2": 8.,
            "left_joint3": 7.,
            "left_joint4": 5.,
            "left_joint5": 3.,
            "left_joint6": 1.,

            "right_joint1": 10.,
            "right_joint2": 8.,
            "right_joint3": 7.,
            "right_joint4": 5.,
            "right_joint5": 3.,
            "right_joint6": 1.}  # [N*m/rad]
        damping = {
            "left_joint1": 1., 
            "left_joint2": 1.,
            "left_joint3": 1.,
            "left_joint4": 1.,
            "left_joint5": 1.,
            "left_joint6": 1.,

            "right_joint1": 1.,
            "right_joint2": 1.,
            "right_joint3": 1.,
            "right_joint4": 1.,
            "right_joint5": 1.,
            "right_joint6": 1.}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset:
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/leg_robot_5/urdf/leg_robot_5.urdf'
        name = "leg_robot__5"  # actor name
        foot_name = "6" # name of the feet bodies, used to index body state and contact force tensors
        penalize_contacts_on = ["left_Link6", "right_Link6","left_Link1", "right_Link1"]
        terminate_after_contacts_on = []
        default_dof_drive_mode = 3
        collapse_fixed_joints = True
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        replace_cylinder_with_capsule = True # replace collision cylinders with capsules, leads to faster/more stable simulation
        flip_visual_attachments = False # Some .obj meshes must be flipped from y-up to z-up
        fix_base_link = True
        disable_gravity = True

        density = 0.001
        angular_damping = 0.
        linear_damping = 0.
        max_angular_velocity = 1000.
        max_linear_velocity = 1000.
        armature = 0.
        thickness = 0.01

        


    class sim:
        dt =  0.005
        substeps = 1
        gravity = [0., 0. ,0.]  # [m/s^2] -9.81
        up_axis = 1  # 0 is y, 1 is z

        class physx:
            num_threads = 10
            solver_type = 1  # 0: pgs, 1: tgs
            num_position_iterations = 4
            num_velocity_iterations = 0
            contact_offset = 0.01  # [m]
            rest_offset = 0.0   # [m]
            bounce_threshold_velocity = 0.5 #0.5 [m/s]
            max_depenetration_velocity = 1.0
            max_gpu_contact_pairs = 2**23 #2**24 -> needed for 8000 envs and more
            default_buffer_size_multiplier = 5
            contact_collection = 2 # 0: never, 1: last sub-step, 2: all sub-steps (default=2)

class LegRobot5CfgPPO(LeggedRobotCfgPPO):
    class runner:
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 24 # per iteration
        max_iterations = 1500 # number of policy updates

        # logging
        save_interval = 50 # check for potential saves every this many iterations
        experiment_name = 'leg_robot__5'
        resume = False  # 是否从检查点恢复训练
        run_name = ''
        