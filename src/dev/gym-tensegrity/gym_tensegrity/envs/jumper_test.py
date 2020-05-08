import gym
import gym_tensegrity
import numpy as np
import os
from time import sleep

def test(config=None):
    def print_observation(obs):
        # This printing for the default observation
        print("Observations: ")
        for i in range(6):
            print("#{:} End point: {:}".format(i+1, [obs[3*i:3*(i+1)]]))
        print("---")
        for i in range(6):
            print("#{:} End point velocity: {:}".format(i+1, [obs[3*(i+6):3*(i+1+6)]]))

        print("Leg angle:{:}".format(env.env.getLegAngle()*180/np.pi))
        squre_sides_angles = env.env.getSquareSidesAngles()
        print("Square side angle1:{:}".format(squre_sides_angles[0]*180/np.pi))
        print("Square side angle2:{:}".format(squre_sides_angles[1]*180/np.pi))
        print("----------------------------------")
    if(config is not None):
        env = gym.make('gym_tensegrity:jumper-v0', config=config)
    if(config is None):
        env = gym.make('gym_tensegrity:jumper-v0') 
           
    observation = env.reset()
    print_observation(observation)
    tot_reward = 0
    action = np.array([0. for i in range(8)])
    done = False
    input("-> check point: WAIT for INPUT !!!!")

    while not done:
        #inp = input("INPUT")
        # action = env.action_space.sample()
        print("Action: {:}".format(action))
        observation, reward, done, _= env.step(action)
        tot_reward += reward
        print("Reward: {:}, Done: {:}".format(reward,done))
        print("Time: {:}".format(env.env.getTime()))
        print_observation(observation)
        print("angle:{:}".format(env.env.getLegAngle()*180/np.pi))
        print("Total Reward: {:}".format(tot_reward))
        # input("-> check point: WAIT for INPUT !!!!")

        # sleep(0.01)
    input("-> check point: WAIT for INPUT !!!!")

    flag = 0
    while True:
        inp = 'd'
        # inp = input("~~~~~~input: ")
        #action = env.action_space.sample()
        #observation, reward, done, _= env.step(action)

        if(inp == "w"):
            flag = 1
        elif(inp == "s"):
            flag = -1
        elif(inp == "d"):
            flag = 0

        if(flag < 0):
            action[0] = -0.1
            
        if(flag > 0):
            action[0] = 0.1

        if(flag == 0):
            action[0] = 0

        observation, reward, done, _= env.step(action)
        print(action)
        print_observation(observation)


if __name__ == "__main__":
    test({'starting_coordinates':[0,100,0], "max_num_steps":10000})

