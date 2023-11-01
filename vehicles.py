#!/usr/bin/env python3

import airsim
import rospy
import os
import random
import numpy as np

from std_msgs.msg import String


class Ue4Briedge:
    def __init__(self, host= '172.20.0.2') -> None:
        '''
        For set ip address from host, use ipconfig result's on host
        For set ip address from docker network, use a ip from ping result's between containers on host
        For set ip address from WSL, os.environ['WSL_HOST_IP'] on host
        '''
        self.__client = airsim.MultirotorClient(host)
        self.__client.confirmConnection()
        rospy.logwarn(f"\nConnection: {self.__client.ping()}")
        
    
    def restart_connection(self) -> None: 
        "Reset the ue4 client conection."
        self.__client.reset()

class Simulation(Ue4Briedge):
    def __init__(self, start_position):
        ConnectionUe4.__init__(self)
        self.__start_position = start_position
        self.__modes = [] 
        self.__pub_info = rospy.Publisher("simulation_info", String, queue_size=10)

    def set_vehicle_pose(self, vehicle_name='', px=float(), py=float(), pz=float(), yaw=float()):
        #All elements in pose need to have the type float if is None
        pose = airsim.Pose() 
        pose.orientation = airsim.utils.to_quaternion(0, 0, yaw= np.deg2rad(yaw))
        pose.position.x_val = px
        pose.position.y_val = py
        pose.position.z_val = pz
        self.__client.simSetVehiclePose(pose, True, vehicle_name)

        info = "New Vehicle pose are setted: {} - ({:.2f} {:.2f} {:.2f})\n".format(vehicle_name, px, py, pz) 
        self.__pub_info.publish(info)