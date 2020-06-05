#!/usr/bin

import gym
import time
import numpy as np
import time
from gym import utils, spaces
import rospy
from gym.utils import seeding
from gym.envs.registration import register
from geometry_msgs.msg import TwistStamped,PoseStamped,Pose
from gazebo_connection import GazeboConnection
from arm import armtakeoff

# register the training environment in the gym as an available one
reg = register(
    id='QuadcopterLiveShow-v0',
    entry_point='myquadcopter_env:QuadCopterEnv',
    max_episode_steps=300,
    )
# env = gym.make('QuadcopterLiveShow-v0')

class QuadCopterEnv(gym.Env):

    def __init__(self):

        self.running_step = 0.3
        self.mode = ''
        self.count =0
        self.current_init_pose = PoseStamped()

        self.desired_pose=PoseStamped()
        self.desired_pose.pose.position.x = 6
        self.desired_pose.pose.position.y = 10
        self.desired_pose.pose.position.z = 5

        self.gazebo = GazeboConnection()
        self.arming = armtakeoff()
        self.data_pose = PoseStamped()

        self.vel_pub=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=20)
        self.action_space = spaces.Discrete(5)
        self.reward_range = (-np.inf, np.inf)
        self.seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def _reset(self):
        #self.control_snake.respawn()
        self.gazebo.resetSim()

        self.gazebo.unpauseSim()        

        self.arming.disarm()
        

        self.gazebo.resetSim()

        self.init_desired_pose()

        self.arming.arm()
        self.take_observation()
        observation = np.array([self.data_pose.pose.position.x,self.data_pose.pose.position.y,self.data_pose.pose.position.z,self.calculate_dist_between_two_Points()])
        observation = np.float32(observation)
        self.gazebo.unpauseSim() 
        return observation

    def _step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot
        
        # 1st, we decide which velocity command corresponds
        vel_cmd = TwistStamped()
        if action == 0: #FORWARD
            vel_cmd.twist.linear.x = 3
        elif action == 1: #Back
            vel_cmd.twist.linear.x = -3    
        elif action == 2: #LEFT
            vel_cmd.twist.linear.y = 3
        elif action == 3: #RIGHT
            vel_cmd.twist.linear.y = -3
        elif action == 4: #Up
            vel_cmd.twist.linear.z = 3
        elif action == 5: #Down
            vel_cmd.twist.linear.z = -3



        self.gazebo.unpauseSim()
        self.vel_pub.publish(vel_cmd) 
        self.take_observation()

        reward,done = self.process_data(self)
        reward =(reward - 100)*2
        state = np.array([self.data_pose.pose.position.x,self.data_pose.pose.position.y,self.data_pose.pose.position.z,self.calculate_dist_between_two_Points()])
        state = np.float32(state)
        return state, reward, done, {}

    def take_observation (self):

        def current_pos_callback(position):
            self.data_pose = position


        rospy.Subscriber('mavros/local_position/pose',PoseStamped,current_pos_callback)
   

    def calculate_dist_between_two_Points(self):
        s1x=self.data_pose.pose.position.x
        s1y=self.data_pose.pose.position.y
        s1z=self.data_pose.pose.position.z
        p2x=self.desired_pose.pose.position.x
        p2y=self.desired_pose.pose.position.y
        p2z=self.desired_pose.pose.position.z
        a = np.array((s1x ,s1y,s1z))
        b = np.array((p2x ,p2y,p2z))
        
        dist = np.linalg.norm(a-b)  
        return dist


    def init_desired_pose(self):

        self.current_init_pose.pose.position.x = self.data_pose.pose.position.x
        self.current_init_pose.pose.position.y = self.data_pose.pose.position.y
        self.current_init_pose.pose.position.z = self.data_pose.pose.position.z
    

    def improved_distance_reward(self,current_pose):

        current_dist = self.calculate_dist_between_two_Points()

        reward = -1*current_dist + 20           
        return reward


    def process_data(self,data_position):

        done = False

           
        if self.desired_pose == self.data_pose:
            reward = 10000
        else:
            reward = self.improved_distance_reward(data_position)

        return reward,done