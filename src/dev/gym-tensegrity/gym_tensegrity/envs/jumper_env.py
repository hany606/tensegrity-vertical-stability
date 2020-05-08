"""jumper_env.py: Create the gym custom environment of tensegrity one legged jumpeing robot"""
__author__ = "Hany Hamed"
__credits__ = ["Hany Hamed", "Vlad Kurenkov", "Sergie Savin"]
__version__ = "1.0.0"
__email__ = "h.hamed.elanwar@gmail.com / h.hamed@innopolis.university"
__status__ = "Paper Results"

# This file will contain all the information about the agent and the environment starting from the rendering of the GUI to the rewards,... etc.
import os
import time
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import sys
import signal
from math import floor,log2
import logging
from random import randint,uniform


import numpy as np
import math
from gym_tensegrity.envs.jumper_model import JumperModel

path_to_model = os.path.join(os.environ["TENSEGRITY_HOME"], "build/dev/jumper/AppJumperModel")

# Machine with Xscreen
# sim_exec = "gnome-terminal -e {}".format(path_to_model)

#Headless
sim_exec = "{}".format(path_to_model)

class JumperEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, config=None):
        if(config is not None):    
            self.config =  {
                            'host_name': 'localhost' if 'host_name' not in config.keys() else config['host_name'],
                            'port_num':None if 'port_num' not in config.keys() else config['port_num'],
                            'sim_exec':sim_exec if 'sim_exec' not in config.keys() else config['sim_exec'],
                            'dl':0.1 if 'dl' not in config.keys() else config['dl'],
                            'observation': ['end_points', 'end_points_velocities'] if 'observation' not in config.keys() else config['observation'],
                            'control_type': 'rest_length_mod' if 'control_type' not in config.keys() else config['control_type'],
                            'num_repeated_action': 1 if 'num_repeated_action' not in config.keys() else config['num_repeated_action'],
                            'max_num_steps': 20000 if 'max_num_steps' not in config.keys() else config['max_num_steps'],
                            'starting_coordinates': [0,100,0] if 'starting_coordinates' not in config.keys() else config['starting_coordinates'],
                            'starting_angle': [0,0] if 'starting_angle' not in config.keys() else config['starting_angle'],
                            'randomized_starting': {"angle":[[False,False], [0,0], [0,0]], "height":[False, 100,100]} if 'randomized_starting' not in config.keys() else config['randomized_starting'],
                            'starting_leg_angle' : [0,0] if 'starting_leg_angle' not in config.keys() else config['starting_leg_angle'],
                            'observation_noise': None if 'observation_noise' not in config.keys() else config['observation_noise']
                            }
        else:
            self.config =  {
                            'host_name': 'localhost',
                            'port_num':None,
                            'sim_exec':sim_exec,
                            'dl':0.1,
                            'observation': ['end_points', 'end_points_velocities'],
                            'control_type': 'rest_length_mod',
                            'num_repeated_action': 1,
                            'max_num_steps': 20000,
                            'starting_coordinates': [0,100,0],
                            'starting_angle': [0,0],
                            'randomized_starting': {"angle":[[False,False], [0,0], [0,0]], "height":[False, 100,100]},
                            'starting_leg_angle' : [0,0],
                            'observation_noise': None
                            }
        super(JumperEnv, self).__init__()

        if('end_points' not in self.config['observation'] and 'rest_length' not in self.config['observation'] and 'current_length'  not in self.config['observation'] and 'end_points_velocities' not in self.config['observation']):
            raise Exception("Wrong choice for the type of the observation, you should choose one of these [end_points, rest_length, current_length, end_points_velocities] or any option from them together in a form of list")

        if('rest_length' not in self.config['control_type'] and 'current_length' not in self.config['control_type'] and 'rest_length_mod'  not in self.config['control_type'] and 'current_length_mod'  not in self.config['control_type']):
            raise Exception("Wrong choice for the type of the control_type, you should choose one of these [rest_length, current_length, rest_length_mod, current_length_mod]")

        # Agent self variables
        self.max_cable_length = 50
        self.min_leg_angle = -np.pi/2 # in radian
        self.max_leg_angle =  np.pi/2 # in radian
        self.end_points_num = 6
        self.min_coordinate = -200
        self.max_coordinate = -self.min_coordinate
        self.dl = self.config['dl'] # This were used for discrete action space
        self.count_rewards_flag = False
        self.starting_coordinates = self.config['starting_coordinates']    # starting_coordinates: (y,z,x)
        self.starting_angle = self.config['starting_angle'] # (angle around x-axis, angle around y-axis) in degree as in parsing the angles in radian to cmd command as parameter is giving errors
        self.starting_leg_angle = self.config['starting_leg_angle']
        self.num_steps = 0
        self.max_num_steps = self.config['max_num_steps']
        self.observation_noise = self.config['observation_noise']


        # The angles for min and max here for the randomization in degree
        self.min_starting_angle = [-3, -3] if len(self.config["randomized_starting"]["angle"]) < 2 else self.config["randomized_starting"]["angle"][1]
        self.max_starting_angle = -1*self.min_starting_angle if len(self.config["randomized_starting"]["angle"]) < 3 else self.config["randomized_starting"]["angle"][2]
        
        self.min_starting_coordinates = 10 if len(self.config["randomized_starting"]["height"]) < 2 else self.config["randomized_starting"]["height"][1]
        self.max_starting_coordinates = 100 if len(self.config["randomized_starting"]["height"]) < 3 else self.config["randomized_starting"]["height"][2]
        
        random_flag, random_starting_conditions = self.randomizStartingConditions()
        if(random_flag > 0):
            # self.starting_angle = random_starting_conditions["starting_angle"]
            self.starting_leg_angle = self.starting_leg_angle if "starting_angle" not in random_starting_conditions.keys() else random_starting_conditions["starting_angle"]  # This will randomize the leg_angle not the whole structure(model) angle
            self.starting_height = self.starting_coordinates[1] if "starting_height" not in random_starting_conditions.keys() else random_starting_conditions["starting_height"]
            self.starting_coordinates[1] = self.starting_height # starting_coordinates [y,z,x]

        # self.starting_coordinates[1] = 10.535 # starting_coordinates [y,z,x]
        self.env = JumperModel(host_name=self.config['host_name'], port_num=self.config['port_num'],
                               sim_exec=self.config['sim_exec'], dl=self.config['dl'], 
                               control_type= self.config['control_type'], starting_coordinates=self.starting_coordinates,
                               starting_angle=self.starting_angle, starting_leg_angle= self.starting_leg_angle)
        self.env.startSimulator()

        # Continuous Action space for the delta lengths
        self.delta_length = 2

        self.min_end_point_velocity = -2000
        self.max_end_point_velocity = -self.min_end_point_velocity

        low = np.array([-1*self.delta_length for i in range(self.env.controllers_num)])
        high = np.array([self.delta_length for i in range(self.env.controllers_num)])
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)

        low = np.empty((1,0))
        high = np.empty((1,0))

        if('end_points' in self.config['observation']):
            low = np.append(low, np.full((1,self.end_points_num*3), self.min_coordinate))

            high = np.append(high, np.full((1,self.end_points_num*3), self.max_coordinate))

        if('end_points_velocities' in self.config['observation']):
            low = np.append(low, np.full((1,self.end_points_num*3), self.min_end_point_velocity))

            high = np.append(high, np.full((1,self.end_points_num*3), self.max_end_point_velocity))

        
        if('rest_length' in self.config['observation']):
            # low = np.append(low, self.min_leg_angle)
            low = np.append(low, np.zeros(self.env.controllers_num))

            # high = np.append(high, self.max_leg_angle)
            high = np.append(high, np.full((1,self.env.controllers_num), self.max_cable_length))

        if('current_length' in self.config['observation']):
            # low = np.append(low, self.min_leg_angle)
            low = np.append(low, np.zeros(self.env.controllers_num))

            # high = np.append(high, self.max_leg_angle)
            high = np.append(high, np.full((1,self.env.controllers_num), self.max_cable_length))            
        


        self.observation_space = spaces.Box(low= low, high= high, dtype=np.float32)
        self.uncorrelated_noise = [0 for i in range(self.observation_space.shape[0])]
        self.correlated_noise = [0 for i in range(self.observation_space.shape[0])]
        # To randomize the initial state of the strings
        # random_init_lengths = [((1 if randint(1,10)%2 else -1)*uniform(self.delta_length-1, self.delta_length)) for i in range(self.env.controllers_num)]
        # self.env.actions_json["Controllers_val"][:] = random_init_lengths
        # self.env.step()

    def __del__(self):
        self.env.closeSimulator()
    
    
    def step(self, action):
        self.num_steps += 1
        # This modification of multiple steps of actions was adapted from Atari environment: https://github.com/openai/gym/blob/master/gym/envs/atari/atari_env.py
        num_steps = 0
        num_repeated_action = self.config['num_repeated_action']
        rewards = 0
        
        if isinstance(num_repeated_action, int):
            num_steps = num_repeated_action
        else:
            num_steps = randint(num_repeated_action[0], num_repeated_action[1])
        
        
        for _ in range(num_steps):    
            self._takeAction(action)
            observation = self._getObservation()
            rewards += self._getReward(observation)


        if(self.observation_noise is not None):
            self.uncorrelated_noise = np.random.normal(self.observation_noise["uncorrelated"]["mean"],
                                                       self.observation_noise["uncorrelated"]["stdev"],
                                                       self.observation_space.shape[0])
        observation = self._getObservation()
        noisy_observation = observation + self.uncorrelated_noise + self.correlated_noise
        reward = rewards
        done = self._isDone()
        return observation, reward, done, {}

    # Continuous delta length
    # action is number that represents the length and the index of the controller
    # For example imaging that the delta_length = 10
    # Then if the action belongs to (-10,0]U[0,10) -- controller 0
    # action belongs to (-50,-40]U[40,50) -- controller 1
    def _takeAction(self, action):
        if (not isinstance(action, np.ndarray)):
            raise Exception("The action space should be an np.array")
        if action.shape != self.action_space.shape:
            raise Exception("The shape of the provided action does not match")

        action = np.clip(action,-self.delta_length, self.delta_length)
        # This is now useless after the value clipping
        if not self.action_space.contains(action):
            raise Exception("The provided action is out of allowed space.")

        self.env.actions_json["Controllers_val"][:] = action.tolist()
        self.env.step()

    # Observations:
    #   - The dimensions is specified above and their min. and max. values
    def _getObservation(self):
        observation = np.empty((1,0))

        if('end_points' in self.config['observation']):
            for i in self.env.getEndPoints():
                observation = np.append(observation, i)

        if('end_points_velocities' in self.config['observation']):
            observation = np.append(observation, self.env.getEndPointsVelocities())

        if('rest_length' in self.config['observation']):
            # observation = np.append(observation, self.env.getLegAngle())
            observation = np.append(observation, self.env.getRestCablesLengths())

        if('current_length' in self.config['observation']):
            # observation = np.append(observation, self.env.getLegAngle())
            observation = np.append(observation, self.env.getCurrentCablesLengths())

        return np.array(observation)

    def _getReward(self, observation):
        # Reward Criteria will depend on:
        # Survival rewards with the time

        leg_end_points_lower_z = self.env.getLegEndPoints()[0][1]
        if(leg_end_points_lower_z < 2):
                self.count_rewards_flag = True

        # Due to problem in counting the rewards while dropping from the sky, it is better to start
        #   giving rewards when it lands to the ground
        if(self.count_rewards_flag):
            # Positive survival rewards
            reward = 1
        else:
            reward = 0

        return reward

    def _isDone(self):
        #  The criteria for finish will be either
        #   - Fall "The angle is more than theta_max"
        # if the angle is greater than 20 degrees this will mean that the episode is done and the agent failed to balance
        squre_sides_angles = self.env.getSquareSidesAngles()
        if abs(self.env.getLegAngle()) > np.pi/9 or abs(squre_sides_angles[0]) > np.pi/4 or abs(squre_sides_angles[1]) > np.pi/4 or self.num_steps > self.max_num_steps:
                self.num_steps = 0
                #print(self.env.getLegAngle())
                return True
        return False

    def reset(self):
        # Reset the state of the environment to an initial state, and the self vars to the initial values
        self.num_steps = 0
        random_flag, random_starting_conditions = self.randomizStartingConditions()
        if(random_flag > 0):
            # self.starting_angle = random_starting_conditions["starting_angle"]
            # self.setStartingAngle(self.starting_angle)
            self.starting_leg_angle = self.starting_leg_angle if "starting_angle" not in random_starting_conditions.keys() else random_starting_conditions["starting_angle"]  # This will randomize the leg_angle not the whole structure(model) angle
            self.starting_height = self.starting_coordinates[1] if "starting_height" not in random_starting_conditions.keys() else random_starting_conditions["starting_height"]
            self.setStartingLegAngle(self.starting_leg_angle)
            self.setStartingHeight(self.starting_height)
        
        if(self.observation_noise is not None):
            self.correlated_noise = np.random.normal(self.observation_noise["correlated"]["mean"],
                                                     self.observation_noise["correlated"]["stdev"],
                                                     self.observation_space.shape[0])

        # Reset the environment and the simulator
        self.env.reset()
        self.env.step()
        # Not necessary as long as we didn't comment it in the _takeAction above
        for i in range(self.env.controllers_num):
            self.env.actions_json["Controllers_val"][i] = 0

        # get the observations after the resetting of the environment
        return self._getObservation()

    def render(self, mode='human'):
        self.env.render()

    def close(self):
        self.env.closeSimulator()

    def randomizStartingConditions(self, min_angle=None, max_angle=None, min_coordinates=None, max_coordinates=None):
        random_starting_conditions = {}
        flag = 0

        if(min_angle is None):
            min_angle = self.min_starting_angle
        if(max_angle is None):
            max_angle = self.max_starting_angle

        if(min_coordinates is None):
            min_coordinates = self.min_starting_coordinates
        if(max_coordinates is None):
            max_coordinates = self.max_starting_coordinates


        if(type(min_angle) is list and len(min_angle) < 2):
            min_angle.append(min_angle[0])
        if(type(max_angle) is list and len(max_angle) < 2):
            max_angle.append(max_angle[0])

        if(type(min_angle) is not list):
            min_angle = [min_angle, min_angle]
        if(type(max_angle) is not list):
            max_angle = [max_angle, max_angle]

        # The argument is list of True/False of 2 componenets for the angles or it is just True
        if((type(self.config["randomized_starting"]["angle"][0]) is not list and self.config["randomized_starting"]["angle"][0] == True)
             or True in self.config["randomized_starting"]["angle"][0]):
            flag += 1
            floating_precision = 0
            starting_angle = [0,0]
            starting_angle[0] = np.random.uniform(min_angle[0], max_angle[0]) # in degree
            starting_angle[1] = np.random.uniform(min_angle[1], max_angle[1]) # in degree
            
            if(type(self.config["randomized_starting"]["angle"][0]) is list and self.config["randomized_starting"]["angle"][0][0] == False):
                starting_angle[0] = self.starting_leg_angle[0]

            if(type(self.config["randomized_starting"]["angle"][0]) is list and self.config["randomized_starting"]["angle"][0][1] == False):
                starting_angle[1] = self.starting_leg_angle[1]

            if(floating_precision > 0):
                for i in range(len(starting_angle)):
                     starting_angle[i] = int(starting_angle[i]*(10**floating_precision))/(10**floating_precision) 
            print("Starting Angle: {:} in degree".format(starting_angle))
            random_starting_conditions["starting_angle"] = starting_angle
        
        if(self.config["randomized_starting"]["height"][0] != False):
            flag += 1
            floating_precision = 0
            starting_height = np.random.uniform(min_coordinates, max_coordinates,1) # in degree
            if(floating_precision > 0):
                starting_height[0] = int(starting_height[0]*(10**floating_precision))/(10**floating_precision) 
            print("Starting Height: {:}".format(starting_height))
            random_starting_conditions["starting_height"] = int(starting_height)

        return flag, random_starting_conditions

    def setStartingCoordinates(self, coordinates):
        self.env.starting_coordinates = coordinates

    def setStartingAngle(self, angle):
        self.env.setStartingAngle(angle)

    def setStartingLegAngle(self, angle):
        self.env.setStartingLegAngle(angle)

    def setStartingHeight(self, height):
        self.env.setStartingHeight(height)
    
    def setConfig(self, config):
        if "starting_leg_angle" in config.keys():
            self.setStartingLegAngle(config["starting_leg_angle"])
        if "height" in config.keys():
            self.setStartingHeight(config["height"])
