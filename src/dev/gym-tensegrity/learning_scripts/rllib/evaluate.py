# This code is being adapted from: https://github.com/ray-project/ray/blob/master/rllib/train.py
import argparse
import yaml
import json
from ray.tune.config_parser import make_parser
import gym
import gym_tensegrity
import ray
from ray import tune
import ray.rllib.agents.ars as ars
from ray.tune.logger import pretty_print
import numpy as np
from random import randint
from time import sleep

# Usage example via RLlib CLI:
# python3 evaluate.py --evaluation-file=trained_agents/train_default/ARS_jumper_3ba03d10_2020-01-17_19-40-36wgy3fy8a/checkpoint_15/checkpoint-15 --agent-config-file=trained_agents/train_default/ARS_jumper_3ba03d10_2020-01-17_19-40-36wgy3fy8a/params.json
# OR python3 evaluate.py --agent-path=trained_agents/train_default/ARS_jumper_3ba03d10_2020-01-17_19-40-36wgy3fy8a/ --checkpoint-num=120


# ```bash
# python3 evaluate.py --evaluation-file=<path-to-trained-agent-training-checkpoint-xx> --agent-config-file=<path-to-config-file-for-trained-agent.json>
# ```

# --evaluation-file: is the path to the checkpoint for the trained agent that will be used to restore the trained model to be evaluated

# --agent-config-file: is the path to the json file which include all the configurations and parameters to the trained agent

# Example:
# ```bash
# python3 evaluate.py --evaluation-file=trained_agents/train_default/ARS_jumper_3ba03d10_2020-01-17_19-40-36wgy3fy8a/checkpoint_60/checkpoint-60 --agent-config-file=trained_agents/train_default/ARS_jumper_3ba03d10_2020-01-17_19-40-36wgy3fy8a/params.json
# ```
  
def create_environment(env_config):
    print("Creation Envirnoment...")
    print("Environment Configuration: {:}".format(env_config))
    import gym_tensegrity
    return gym.make('gym_tensegrity:jumper-v0', config=env_config)

class Printer:
    def __init__(self,debug=1):
        self.debug_flag = debug

    def all(self, returns, num_episdoes=50):
        if(self.debug_flag):
            if(returns["observation"] is not None):
                self.observation(returns["observation"])
            if(returns["action"] is not None):
                self.action(returns["action"])
            if(returns["done"] is not None):
                self.done(returns["done"])
            if(returns["reward"] is not None):
                self.reward(returns["reward"])
            if(returns["history"] is not None):
                self.history(returns["history"])
            if(returns["mean"] is not None):
                self.mean(returns["mean"], num_episodes=num_episdoes)
    
    def observation(self, observation):
        if(self.debug_flag):
            print("Observations:")
            for obs in observation:
                print("{:}".format(obs),end="\t")
    
    def action(self, action):
        if(self.debug_flag):
            print("Actions:")
            for act in action:
                print("{:}".format(act),end="\t")
    
    def reward(self, reward):
        if(self.debug_flag):
            print("Reward: {:}".format(reward))

    def done(self, done):
        if(self.debug_flag):
            print("Termination: {:}".format(done))

    def history(self, history):
        if(self.debug_flag):
            print("History for {:} episodes:".format(len(history)))
            rewards = 0
            for i in range(len(history)):
                rewards += history[i]["reward"]
                print("#Episode {:} -> ".format(i+1),end="")
                for key in history[i].keys():
                    print("{:}: {:}".format(key, history[i][key]), end="\t")
                print("")
                #print("#Episode {:} -> {:}".format(i+1, history[i]),end="\n")
            self.mean(rewards/len(history),num_episodes=len(history))

    def mean(self, mean, num_episodes=None):
        if(self.debug_flag):
            print("The mean for the rewards for {:} episodes is {:}".format(num_episodes, mean))

    def config(self, data, tag=None):
        if(tag is None):
            print("Configuration: {:}".format(data))
        else:
            print("{:} Configuration: {:}".format(tag, data))

    def separator(self, separator=None):
        if(separator is None):
            print("--------------------------------------")
        else:
            print(separator)

class Evaluater:
    def __init__(self):
        self.printer = Printer()
        self.env_config = {}
        self.agent_config = {}
        self.evaluation_config = {}
        self.EXAMPLE_USAGE = """
            Usage example via RLlib CLI:
            python3 evaluate.py --evaluation-file=trained_agents/train_default/ARS_jumper_3ba03d10_2020-01-17_19-40-36wgy3fy8a/checkpoint_15/checkpoint-15 --agent-config-file=trained_agents/train_default/ARS_jumper_3ba03d10_2020-01-17_19-40-36wgy3fy8a/params.json
            """

    def create_parser(self, parser_creator=None):
        parser = make_parser(
            parser_creator=parser_creator,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description="Evaluate a trained RL agent.",
            epilog=self.EXAMPLE_USAGE)

        parser.add_argument(
            "--random-agent",
            default=False,
            type=bool,
            help="Evaluate and test an agent with random policy")

        parser.add_argument(
            "--evaluation-file",
            default=None,
            type=str,
            help="Path to the object file of the trained model." 
            "The file should be in the following format: checkpoint_<NUM>/checkpoint-<NUM>")
        
        parser.add_argument(
            "--agent-path",
            default=None,
            type=str,
            help="Path to the agent folder")

        parser.add_argument(
            "--checkpoint-num",
            default=None,
            type=int,
            help="Number of the checkpoint")

        parser.add_argument(
            "--agent-config-file",
            default=None,
            type=str,
            help="Path to the json configuration file of the trained model." 
            "The file should be in the following format: __.json")

        parser.add_argument(
            "--observation-space-type",
            default=["end_points", "end_points_velocities"],
            type=list,
            help="Set the observation space type to be one of those: [end_points, rest_length, current_length, end_points_velocities] or any option from them together in a form of a list")

        parser.add_argument(
            "--controller-type",
            default="rest_length_mod",
            type=str,
            help="Set the controller type to be one of those: [rest_length, current_length, rest_length_mod, current_length_mod]")
        
        parser.add_argument(
            "--num-episodes",
            default="10",
            type=int,
            help="Set the number of the episodes for the evaluation and running the model on")

        parser.add_argument(
            "-f",
            "--config-file",
            default=None,
            type=str,
            help="If specified, use config options from this file. Note that this "
            "overrides any trial-specific options set via flags above.")

        return parser
        

    def run_episode(self, env, agent, random=False):
        observation = env.reset()
        #self.printer.observation(observation)
        cumulative_reward = 0
        done  = False
        #input("Check-point: Press Enter!!!!!!!!!!!!!")
        while not done:
            #sleep(0.01)
            if(not random):
                action = agent.compute_action(observation)
            else:
                action = env.action_space.sample()
                #action = np.zeros(8)
            observation, reward, done, _ = env.step(action)
            #sleep(0.1)
            #self.printer.observation(observation)
            #self.printer.reward(reward)
            #self.printer.action(action)
            #self.printer.done(done)
            cumulative_reward += reward

        return cumulative_reward


    def evaluate(self, evaluation_config, agent_config, env_config, random=False):
        config = ars.DEFAULT_CONFIG.copy()
        for key in agent_config.keys():
            config[key] = agent_config[key]
        config["num_workers"] = 1
        config["noise_size"] = 250000
        trained_agent = ars.ARSTrainer(config, env="jumper")
        trained_agent.restore(evaluation_config["evaluation_file"])
        num_episodes = evaluation_config["num_episodes"]
        env = create_environment(env_config)
        cumulative_reward = 0
        history = []
        for i in range(num_episodes):
            reward = self.run_episode(env, trained_agent, random=random)
            self.printer.reward(reward)
            min_history_dict = {"reward":reward}
            history.append(min_history_dict)
            cumulative_reward += reward

 
        self.printer.history(history)
        # self.printer.mean(cumulative_reward/evaluation_config["num_episodes"])

    def run(self, args, parser):
        if args.config_file:
            with open(args.config_file) as f:
                self.evaluation_config = yaml.safe_load(f)
        else:
            self.evaluation_config = {
                "random_agent": args.random_agent,
                "evaluation_file": args.evaluation_file if args.evaluation_file is not None else args.agent_path+"checkpoint_{:}/checkpoint-{:}".format(args.checkpoint_num, args.checkpoint_num),	
                "agent_config_file": args.agent_config_file if args.evaluation_file is not None else args.agent_path+"params.json",
                "observation_space_type": args.observation_space_type,
                "controller_type": args.controller_type,
                "config_file": args.config_file,
                "num_episodes": args.num_episodes,
                "domain_buckets":None,
            }
        
        self.env_config = {"observation": self.evaluation_config["observation_space_type"], "control_type": self.evaluation_config["controller_type"]}
        
        if(self.evaluation_config["agent_config_file"] is None):
            raise Exception("The agent config file should be defined.\n--Hint: it is params.json")
        with open(self.evaluation_config["agent_config_file"]) as json_file:
            self.agent_config = json.load(json_file)
            if("env_config" in self.agent_config.keys()):
                self.env_config = self.agent_config["env_config"]
        
        self.printer.separator()
        self.printer.config(self.env_config, tag="Environment")
        self.printer.config(self.agent_config, tag="Agent")
        self.printer.config(self.evaluation_config, tag="Evaluation")
        self.printer.separator()
        self.env_config["max_num_steps"] = 20000
        self.evaluation_config["num_episodes"] = 50
        tune.register_env("jumper", create_environment)
        ray.init()
        self.evaluate(self.evaluation_config, self.agent_config, self.env_config, random=self.evaluation_config["random_agent"])

if __name__ == "__main__":
    evaluate = Evaluater()
    parser = evaluate.create_parser()
    args = parser.parse_args()
    evaluate.run(args, parser)
