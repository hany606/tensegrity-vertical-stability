"""Jumper_model.py: Create the agent of model of the one legged jumping tensegrity robot and provide an easy interface to be used with RL algorithms"""
__author__ = "Hany Hamed"
__credits__ = ["Hany Hamed", "Vlad Kurenkov", "Sergie Savin"]
__version__ = "1.0.0"
__email__ = "h.hamed.elanwar@gmail.com / h.hamed@innopolis.university"
__status__ = "Paper Result"


import socket
import sys
import signal
import json
from time import *
import os
import subprocess
import numpy as np

path_to_model = os.path.join(os.environ["TENSEGRITY_HOME"], "build/dev/jumper/AppJumperModel")
sim_exec = "gnome-terminal -e {}".format(path_to_model)

class JumperModel():
    def __init__(self, host_name='localhost', port_num=10040, packet_size=5000,
                 sim_exec=sim_exec, dl=0.1, controllers_num=8, control_type="rest_length", 
                 starting_coordinates=[0,100,0], starting_angle=[0,0], starting_leg_angle=[0,0]):
        self.starting_coordinates = starting_coordinates
        self.starting_angle = starting_angle
        self.starting_leg_angle = starting_leg_angle
        self.host_name = host_name
        self.port_num = port_num
        self.packet_size = packet_size
        self.actions_json = {
            'Controllers_val': [0,0,0,0,0,0,0,0],
            'Reset': 0
        }
        self.sim_json = {"Rest_cables_lengths":
                        [0,0,0,0,0,0,0,0],
                        "Current_cables_lengths":
                        [0,0,0,0,0,0,0,0],
                        "End_points":
                        [[0.,0.,0.], [0.,0.,0.], [0.,0.,0.], [0.,0.,0.], 
                         [0.,0.,0.],[0.,0.,0.]],
                        "End_points_velocities":
                        [[0.,0.,0.], [0.,0.,0.], [0.,0.,0.], [0.,0.,0.], 
                         [0.,0.,0.],[0.,0.,0.]],
                        "Leg_end_points_world":
                        [[0.,0.,0.], [0.,0.,0.]],
                        "Time": 0.,
                        "ZFinished": 1,
                        "Flags":[1,0,0]}

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # print(self.port_num)
        if(self.port_num is None):
            self.port_num = 0
        self.server_address = [self.host_name, self.port_num] # Bind the socket to the port

        self.connection = None
        self.client_address = None
        self.child_process = None
        print('#########\nstarting up on {}\n#########'.format(self.server_address))
        try:
            self.sock.bind(tuple(self.server_address))

        except socket.error as exc:
            self.port_num += 1
            self.server_address[1] = self.port_num
            print('#########\nstarting up on {} after getting an error of busy port first\n#########'.format(self.server_address))
            self.sock.bind(tuple(self.server_address))
        print('#########\nConnected to port: {:}\n#########'.format(self.sock.getsockname()[1]))
        print('#########\nServer binding is finished\n#########')
        self.sock.listen(1)  # Listen for incoming connections
        self.reset_flag = False
        self.close_flag = False
        self.dl = dl                            # Self modified parameter
        self.end_points_num = 6
        self.controllers_num = controllers_num    # Self modified parameter
        self.leg_end_points = [4,5]
        self.leg_length = 20
        self.port_num = self.sock.getsockname()[1]
        self.control_type = control_type
        self.control_type_index = {"rest_length": 0, "current_length": 1, "rest_length_mod": 2, "current_length_mod": 3}
        self.orginal_sim_exec = sim_exec
        self.set_sim_exec(self.orginal_sim_exec)


    def set_sim_exec(self, sim_exec):
        self.sim_exec = sim_exec + ' {:} {:} {:} {:} {:} {:} {:} {:} {:} {:}'.format(self.host_name, self.port_num, self.control_type_index[self.control_type], self.starting_coordinates[0], self.starting_coordinates[1], self.starting_coordinates[2] , self.starting_angle[0], self.starting_angle[1], self.starting_leg_angle[0], self.starting_leg_angle[1])
        print("EXEC: {:}".format(self.sim_exec))

    def __del__(self):
        self.closeSimulator()
        # sys.exit(0)

    # function for writing data into TCP connection
    def write(self, data):
        try:
            self.connection.sendall(data.encode())
        except Exception as e:
            print("$$$$$$$$$$$$ ERROR in Writing $$$$$$$$$$$$")
            print("Error: " + str(e))

    # function for reading data from TCP connection
    def read(self):
        try:
            data = []
            counter = 1
            # Receive the data in small chunks and retransmit it
            while True:
                data.append(self.connection.recv(self.packet_size).decode("utf-8"))         #reading part
                if 'ZFinished' in str(data[-1][-14:-1]):
                    break
                counter += 1
            return "".join(data)
        except ValueError:
            print(ValueError)
            print("$$$$$$$$$$$$ ERROR in Reading $$$$$$$$$$$$")
            # sleep(2)
            return None

    def startSimulator(self):
        self.close_flag = False
        self.reset_flag = False
        if(self.sim_exec == sim_exec):
            print("#Warning: Starting an old version")
        #Headless
        self.child_process = subprocess.Popen(self.sim_exec, shell=True)  
        #print('#########\nwaiting for a connection\n#########')
        self.connection, self.clientAddress = self.sock.accept()  #wait until it get a client
        #print('connection from', self.clientAddress)

    def closeSimulator(self):
        self.close_flag = True
        # kill the shell script of the simulator
        if self.connection is not None:
            self.connection.close()
        if self.child_process is not None:
            os.kill(self.child_process.pid, signal.SIGKILL)

    def render(self):
        pass
    
    def reset(self):
        self.reset_flag = True
        self.closeSimulator()
        self.startSimulator()

    def step(self):
        if (self.close_flag == False):
            if (self.reset_flag == True):
                self.reset()
            
            self.write(json.dumps(self.actions_json))   # Write to the simulator module the json object with the required info
            sim_raw_data = self.read()
            if(sim_raw_data is not None):
                self.sim_json = json.loads(sim_raw_data)  # Parse the data from string to json
        else:
            self.closeSimulator()

    def getRestCablesLengths(self, i=None):
        if(i is None):
            return self.sim_json["Rest_cables_lengths"]
        return self.sim_json["Rest_cables_lengths"][i]
        
    
    def getCurrentCablesLengths(self, i=None):
        if(i is None):
            return self.sim_json["Current_cables_lengths"]
        return self.sim_json["Current_cables_lengths"][i]

    def getEndPoints(self):
        end_points = []
        # Notice that the end_points are in the form (y,z,x) as it is coming from the simulator like this
        for i in range(self.end_points_num):
            end_points.append(self.sim_json["End_points"][i])
        return end_points

    def getEndPointsVelocities(self):
        end_points_velocities = []
        # Notice that the end_points are in the form (y,z,x) as it is coming from the simulator like this
        for i in range(self.end_points_num):
            end_points_velocities.append(self.sim_json["End_points_velocities"][i])
        return end_points_velocities

    def getLegEndPoints(self):
        return [self.sim_json["Leg_end_points_world"][0], self.sim_json["Leg_end_points_world"][1]]

    # point_a: is the end point of the leg from down
    # point_b: is the end point of the virtual horizontal leg from up
    # point_c: is the end point of the actual leg from up
    def getLegAngle(self):
        point_a = np.array(self.sim_json["End_points"][self.leg_end_points[0]])
        point_b = [0,0,0]
        point_b[:] = point_a[:]
        point_b[1] += self.leg_length
        point_c = np.array(self.sim_json["End_points"][self.leg_end_points[1]])
        v1 = point_b - point_a
        v2 = point_c - point_a
        dot_product = np.dot(v1,v2)
        v1_mag = np.linalg.norm(v1)
        v2_mag = np.linalg.norm(v2)
        angle = np.arccos(dot_product/(v1_mag*v2_mag))
        return angle

    """
    (b)|
       |
    (a)|_____(c)
    """
    def getSquareSidesAngles(self):
        point_a = np.array(self.sim_json["End_points"][1])
        point_b = np.array(self.sim_json["End_points"][2])
        point_c = np.array(self.sim_json["End_points"][0])

        point_d = [0,0,0]
        point_e = [0,0,0]

        point_d[:] = point_b[:]
        point_e[:] = point_c[:]

        point_d[1] = point_a[1]
        point_e[1] = point_a[1]

        v_ab = point_b - point_a
        v_ac = point_c - point_a
        v_ad = point_d - point_a
        v_ae = point_e - point_a

        dot_v_ad_v_ab = np.dot(v_ad, v_ab)
        dot_v_ae_v_ac = np.dot(v_ae, v_ac)

        mag_v_ab = np.linalg.norm(v_ab)
        mag_v_ac = np.linalg.norm(v_ac)
        mag_v_ad = np.linalg.norm(v_ad)
        mag_v_ae = np.linalg.norm(v_ae)

        angle_x = np.arccos(dot_v_ad_v_ab/(mag_v_ad*mag_v_ab))
        angle_y = np.arccos(dot_v_ae_v_ac/(mag_v_ae*mag_v_ac))
        
        return [angle_x, angle_y]
    
    def getTime(self):
        return self.sim_json["Time"]
    
    def setStartingAngle(self, angle):
        self.starting_angle = angle
        self.set_sim_exec(self.orginal_sim_exec)
        
    def setStartingLegAngle(self, angle):
        self.starting_leg_angle = angle
        self.set_sim_exec(self.orginal_sim_exec)

    def setStartingHeight(self, height):
        self.starting_coordinates[1] = height
        self.set_sim_exec(self.orginal_sim_exec)