import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import math
import os
import psutil
import signal
import sys
from gym import utils, spaces
from gym_gazebo2.utils import ut_generic, ut_launch, ut_mara, ut_math, ut_gazebo, tree_urdf, general_utils
from gym.utils import seeding
from gazebo_msgs.srv import SpawnEntity
import subprocess
import argparse
import transforms3d as tf3d

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing mara joint angles.
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ContactState
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from ros2pkg.api import get_prefix_path
from builtin_interfaces.msg import Duration
from hrim_actuator_gripper_srvs.srv import ControlFinger

from ctypes import *

# Algorithm specific
from PyKDL import ChainJntToJacSolver # For KDL Jacobians
import pdb

class MARAEnv(gym.Env):
    """
    TODO. Define the environment.
    """

    def __init__(self):
        """
        Initialize the Delta MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsParserMARA().parse_args()
        self.gzclient = args.gzclient
        self.realSpeed = args.realSpeed
        self.velocity = args.velocity
        self.multiInstance = args.multiInstance
        self.port = args.port

        # Set the path of the corresponding URDF file
        if self.realSpeed:
            urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/" + urdf
        else:
            urdf = "reinforcement_learning/arc_delta_onTable.urdf"
            urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/" + urdf

        # Launch mara in a new Process
        self.launch_subp = ut_launch.startLaunchServiceProcess(
            ut_launch.generateLaunchDescriptionMara(
                self.gzclient, self.realSpeed, self.multiInstance, self.port, urdfPath))

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.max_episode_steps = 1024 #default value, can be updated from baselines
        self.iterator = 0
        self.reset_jnts = True
        self._collision_msg = None

        # Initial joint position
        INITIAL_JOINTS = np.array([0., 0., 0.])

        # # Topics for the robot publisher and subscriber.
        JOINT_PUBLISHER = '/mara_controller/command'
        JOINT_SUBSCRIBER = '/mara_controller/state'

        # joint names:
        MOTOR1_JOINT = 'motor1'
        MOTOR2_JOINT = 'motor2'
        MOTOR3_JOINT = 'motor3'

        # Set constants for links
        WORLD = 'world'
        BASE = 'link_0'  #Added BASE
        MARA_MOTOR1_LINK = 'uleg_1'
        MARA_MOTOR2_LINK = 'uleg_2'
        MARA_MOTOR3_LINK = 'uleg_3'

        JOINT_ORDER = [MOTOR1_JOINT,MOTOR2_JOINT, MOTOR3_JOINT]
        LINK_NAMES = [ WORLD,BASE,
                        MARA_MOTOR1_LINK, MARA_MOTOR2_LINK,
                        MARA_MOTOR3_LINK]

        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
             'initial_velocities': []
        }
        #############################

        m_jointOrder = copy.deepcopy(JOINT_ORDER)
        m_linkNames = copy.deepcopy(LINK_NAMES)

        # Initialize target end effector position
        self.environment = {
            'jointOrder': m_jointOrder,
            'linkNames': m_linkNames,
            'reset_conditions': reset_condition,
            }


        self.cli = self.node.create_client(ControlFinger, '/hrim_actuation_gripper_000000000004/goal')
        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(JointTrajectory, JOINT_PUBLISHER, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointTrajectoryControllerState, JOINT_SUBSCRIBER, self.observation_callback, qos_profile=qos_profile_sensor_data)
        self._sub_coll = self.node.create_subscription(ContactState, '/gazebo_contacts', self.collision_callback, qos_profile=qos_profile_sensor_data)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        self.numJoints = 3
        self.obs_dim = self.numJoints

        low = -np.pi * np.ones(self.numJoints)
        high = np.pi * np.ones(self.numJoints)

        self.action_space = spaces.Box(low, high)

        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        # Spawn Target element in gazebo.
        # node & spawn_cli initialized in super class
        spawn_cli = self.node.create_client(SpawnEntity, '/spawn_entity')

        while not spawn_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/spawn_entity service not available, waiting again...')

    def moveGripper(self,vel,pos,effort):
        self.vel = vel
        self.pos = pos
        self.effort = effort
        print('moveGripper', self.vel , self.pos, self.effort)
        req = ControlFinger.Request()
        req.goal_velocity = self.vel         # velocity range: 30 -  250 mm/s
        req.goal_effort = self.effort        # forces range:   10 -  125 N
        req.goal_linearposition = self.pos  # position range   0 - 0.87 rad

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info('Goal accepted: %d: ' % future.result().goal_accepted)
        else:
            self.node.get_logger().error('Exception while calling service: %r' % future.exception())

        done = True

        return done

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg = message

    def collision_callback(self, message):
        """
        Callback method for the subscriber of Collision data
        """
        if message.collision1_name != message.collision2_name:
            self._collision_msg = message

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def take_observation(self,action):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # # Take an observation
        rclpy.spin_once(self.node)
        obs_message = self._observation_msg

        # Collect the present joint angles and velocities from ROS for the state.
        motor_flag = True
        cnt = 0
        while (motor_flag):
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg
            lastObservations = ut_mara.processObservations(obs_message, self.environment)
            motor1_err = abs(lastObservations[0] - action[0])
            motor2_err = abs(lastObservations[1] - action[1])
            motor3_err = abs(lastObservations[2] - action[2])

            if motor1_err < 0.01 and motor2_err < 0.01 and motor3_err < 0.01:
                motor_flag = False
            #print ('Motor observation:', lastObservations)
            cnt+=1
            #print ("Counter Value: ", cnt)
            if cnt == 1000:
                break
        #print ('Motor observation:', lastObservations)

        #Set observation to None after it has been read.
        self._observation_msg = None

        state = np.r_[np.reshape(lastObservations, -1),]
        return state

    def collision(self):
        # Reset if there is a collision
        if self._collision_msg is not None:
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)
            self._collision_msg = None
            self.collided += 1
            return True
        else:
            return False

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        
        done = False
        self.iterator+=1
        # Execute "action"
        self._pub.publish(ut_mara.getTrajectoryMessage(
            action[:self.numJoints],
            self.environment['jointOrder'],
            self.velocity))

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        obs = self.take_observation(action)
        done = True

        return obs, done

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0

        if self.reset_jnts is True:
            # reset simulation
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        obs = self.take_observation()

        # Return the corresponding observation
        return obs

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()
