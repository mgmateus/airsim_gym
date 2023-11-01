#!/usr/bin/env python3

import rospy
import os
import random

import numpy as np

from airsim import MultirotorClient
from airsim.types import Pose
from airsim.utils import to_quaternion

from std_msgs.msg import String
        
HOST = '172.18.0.2'

        
class Resources:
    
    class Ue4Briedge:
        """Starts communication's Engine.

        Args:
            HOST: 
                -For set ip address from host, use ipconfig result's on host
                -For set ip address from docker network, use a ip from ping result's between containers on host
                -For set ip address from WSL, os.environ['WSL_HOST_IP'] on host.
        """        
        client = MultirotorClient(HOST)
        
        @classmethod            
        def restart(cls) -> None:
            """
            Reset the ue4 client conection.
            """        
            rospy.logwarn(f"\nRestart Connection: {cls.client.ping()}")
            cls.client.reset()
        
        def __init__(cls) -> None:
            cls.client.confirmConnection()
            rospy.logwarn(f"\nConnection: {cls.client.ping()}")
    
    @staticmethod
    def pose(position : tuple, orientation : tuple) -> Pose:
        """_summary_

        Args:
            position (tuple): position (x, y ,z).
            orientation (tuple): orientation in degrees (roll, pitch, yaw).

        Returns:
            Pose: AirSim pose type.
        """        
        x, y, z = position
        roll, pitch, yaw =  np.deg2rad(orientation[0]),\
                            np.deg2rad(orientation[1]),\
                            np.deg2rad(orientation[2])
        pose_ = Pose()
        pose_.position.x_val = x
        pose_.position.y_val = y
        pose_.position.z_val = z
        
        pose_.orientation = to_quaternion(roll, pitch, yaw)
        
        return pose_
    
    def __init__(self):
        self.ue4 = self.Ue4Briedge()
        self.__pub_info = rospy.Publisher("simulation_info", String, queue_size=10)
        
    def get_object_pose(self, object_name : str) -> Pose:
        """Returns a scene element pose.

        Args:
            object_name (str): Element's name.

        Returns:
            Pose: AirSim native pose.
        """        
        pose = self.ue4.client.simGetObjectPose(object_name)
        return pose
    
    def get_vehicle_pose(self, vehicle_name : str) -> Pose:
        """Returns a scene element pose.

        Args:
            vehicle_name (str): Vehicle's name.

        Returns:
            Pose: AirSim native pose.
        """        
        pose = self.ue4.client.simGetVehiclePose(vehicle_name)
        return pose
        
        
    def set_vehicle_pose(self, vehicle_name : str, position : tuple, orientation : tuple, debbug : bool = False) -> None:
        """Define a new pose at one vehicle.

        Args:
            vehicle_name (str): The same was defined on settings.json .
            position (tuple): (x,y,z).
            orientation (tuple): (roll, pitch, yaw).
        """           
        pose = self.pose(position, orientation)
        self.ue4.client.simSetVehiclePose(pose, True, vehicle_name)
        
        if debbug:
            info = f"New Vehicle pose was defined: {vehicle_name} - [{position}, {orientation}]"
            self.__pub_info.publish(info) 
            
    def set_object_pose(self, object_name : str, position : tuple, orientation : tuple, debbug : bool = False) -> None:
        """Define a new pose at one scene object.

        Args:
            object_name (str): Element's name.
            position (tuple): (x,y,z).
            orientation (tuple): (roll, pitch, yaw).
        """           
        pose = self.pose(position, orientation)
        self.ue4.client.simSetObjectPose(pose, True, object_name)
        
        if debbug:
            info = f"New Object pose was defined: {object_name} - [{position}, {orientation}]"
            self.__pub_info.publish(info) 
            
            
class Coverage(Resources):
    def __init__(self, vehicle_name : str, init_position : tuple, init_orientation : tuple) -> None:
        Resources.__init__(self)
        self.__vehicle_name = vehicle_name
        self.__init_position = init_position
        self.__init_orientation = init_orientation
        
    @property
    def vehicle_name(self):
        return self.__vehicle_name
    
    @vehicle_name.setter
    def vehicle_name(self, new_name):
        self.__vehicle_name = new_name
        
    @property
    def init_position(self):
        return self.__init_position
    
    @init_position.setter
    def init_position(self, new_position):
        self.__init_position = new_position
        
    @property
    def init_orientation(self):
        return self.__init_orientation
    
    @init_orientation.setter
    def init_orientation(self, new_orientation):
        self.__init_orientation = new_orientation
        
    def air_pose(self, radius : float, dist : float, target : str) -> None:
        pose = self.get_object_pose(target)
        
        x = pose.position.x_val
        y = pose.position.y_val
        
        b_supx = x - dist
        b_infx = b_supx - radius
        
        a_supx = x + dist
        a_infx = b_supx + radius
        
        b_supy = y - dist
        b_infy = b_supy - radius
        
        a_supy = y + dist
        a_infy = a_supy + radius
        
        nx = random.choice(range(b_infx, b_supx), range(a_infx, a_supx))
        ny = random.choice(range(b_infy, b_supy), range(a_infy, a_supy))
        
        

        
if __name__ == "__main__":
    
    r = Coverage("Hydrone")
    print(r.get_object_pose('eolic'))   
        
        
    
        
        