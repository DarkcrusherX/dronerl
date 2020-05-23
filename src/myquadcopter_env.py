#!/usr/bin/env python

import gym
import rospy
import time
import numpy as np
import time
from gym import utils, spaces
from geometry_msgs.msg import TwistStamped,PoseStamped,Pose

from sensor_msgs.msg import LaserScan

from gym.utils import seeding
from gym.envs.registration import register

from gazebo_connection import GazeboConnection
from arm import armtakeoff

#register the training environment in the gym as an available one
reg = register(
    id='QuadcopterLiveShow-v0',
    entry_point='myquadcopter_env:QuadCopterEnv',
    timestep_limit=100,
    )

#rospy.init_node('environment _definition', anonymous=True)

class QuadCopterEnv(gym.Env):

    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment
        
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=5)
        
        # gets training parameters from param server
        self.speed_value = rospy.get_param("/speed_value")
        self.desired_pose = Pose()
        self.desired_pose.position.z = rospy.get_param("/desired_pose/z")
        self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
        self.running_step = rospy.get_param("/running_step")
        self.max_incl = rospy.get_param("/max_incl")
        self.max_altitude = rospy.get_param("/max_altitude")
        
        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        self.arming = armtakeoff()
        self.data_pose = PoseStamped()
        self.data_lidar = LaserScan()
        self.action_space = spaces.Discrete(5) #Forward,Left,Right,Up,Down
        self.reward_range = (-np.inf, np.inf)

        self.seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def _reset(self):
        
        # 1st disarm
        self.arming.disarm()
        
        # 2nd: resets the simulation to initial values
        self.gazebo.resetSim()

        # 3rd: Unpauses simulation
        self.gazebo.unpauseSim()

        # 3rd: resets the robot to initial conditions
        # self.init_desired_pose()
        # self.takeoff_sequence()

        # 5th: resets the robot to initial conditions
        self.init_desired_pose()


        # 4th arming and takeoff
        self.arming.arm()


        # 4th: takes an observation of the initial condition of the robot
        self.take_observation()
        observation = [self.data_pose.pose.position.x]

        # 5th: pauses simulation
        #self.gazebo.pauseSim()

        return observation

    def _step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot
        
        # 1st, we decide which velocity command corresponds
        vel_cmd = TwistStamped()
        if action == 0: #FORWARD
            vel_cmd.twist.linear.x = self.speed_value
            vel_cmd.twist.angular.z = 0.0
        elif action == 1: #LEFT
            vel_cmd.twist.linear.x = 0.05
            vel_cmd.twist.angular.z = self.speed_value
        elif action == 2: #RIGHT
            vel_cmd.twist.linear.x = 0.05
            vel_cmd.twist.angular.z = -1*self.speed_value
        elif action == 3: #Up
            vel_cmd.twist.linear.z = self.speed_value
            vel_cmd.twist.angular.z = 0.0
        elif action == 4: #Down
            vel_cmd.twist.linear.z = -1*self.speed_value
            vel_cmd.twist.angular.z = 0.0

        # Then we send the command to the robot and let it go
        # for running_step seconds
        self.gazebo.unpauseSim()
        self.vel_pub.publish(vel_cmd)
        time.sleep(self.running_step)
        self.take_observation()
        self.gazebo.pauseSim()

        # finally we get an evaluation based on what happened in the sim
        reward,done = self.process_data(self)

        # Promote going forwards instead if turning
        if action == 0:
            reward += 100
        elif action == 1 or action == 2:
            reward -= 50
        elif action == 3:
            reward -= 150
        else:
            reward -= 50

        state = [self.data_pose.pose.position.x]
        return state, reward, done, {}


    def take_observation (self):

        def current_pos_callback(position):
            #print("position data obtained")
            self.data_pose = position


        rospy.Subscriber('mavros/local_position/pose',PoseStamped,current_pos_callback)

        def lidar_callback(lidar_msg):
            #print("lidar data obtained")
            self.data_lidar=lidar_msg.ranges


        rospy.Subscriber("laser/scan", LaserScan, lidar_callback)



    def calculate_dist_between_two_Points(self,p_init,p_end):
        a = np.array((p_init.x ,p_init.y, p_init.z))
        b = np.array((p_end.x ,p_end.y, p_end.z))
        
        dist = np.linalg.norm(a-b)
        
        return dist


    def init_desired_pose(self):
        
        self.take_observation()
        current_init_pose = PoseStamped()
        current_init_pose = self.data_pose
        lidar = self.data_lidar
        self.best_dist = self.calculate_dist_between_two_Points(current_init_pose.pose.position, self.desired_pose.position)
    

    def reset_cmd_vel_commands(self):
        # We send an empty null Twist
        vel_cmd = TwistStamped()
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)


    def improved_distance_reward(self, current_pose):
        current_pose = PoseStamped()
        current_dist = self.calculate_dist_between_two_Points(current_pose.pose.position, self.desired_pose.position)
        #rospy.loginfo("Calculated Distance = "+str(current_dist))
        
        if current_dist < self.best_dist:
            reward = 100
            self.best_dist = current_dist
        elif current_dist == self.best_dist:
            reward = 0
        else:
            reward = -100
            #print "Made Distance bigger= "+str(self.best_dist)
        
        return reward
        


    def process_data(self, data_position):

        done = False
        c=0
        lidar_bad = False
        # Setting lidar obstacle range as 1m
        for i in range(len(self.data_lidar)):
            if self.data_lidar[i] < 1:                   
                c=c+1

        #  minimum number of lidar line  crossings        
        if c > len(self.data_lidar)*0.1:                   
            lidar_bad = True        

        if lidar_bad == True:
            done = True 
            reward = -200
        else:
            reward = self.improved_distance_reward(data_position)

        return reward,done