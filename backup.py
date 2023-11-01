#!/usr/bin/env python3

import rospy
import os
import random

import numpy as np

from airsim.type import Pose
from airsim.utils import to_quaternion

from std_msgs.msg import String

class Resources(Ue4Briedge):
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
    
    def __init__(self, host : str = '172.20.0.2'):
        super.__init__(self, host)
        self.__pub_info = rospy.Publisher("simulation_info", String, queue_size=10)
        
    def set_vehicle_pose(self, vehicle_name : str, position : tuple, orientation : tuple, debbug : bool = False) -> None:
        """Define a new pose at one vehicle.

        Args:
            vehicle_name (str): The same was defined on settings.json .
            position (tuple): (x,y,z).
            orientation (tuple): (roll, pitch, yaw).
        """           
        pose = self.pose(position, orientation)
        self.__client.simSetVehiclePose(pose, True, vehicle_name)
        
        if debbug:
            info = f"New Vehicle pose was defined: {vehicle_name} - [{position}, {orientation}]"
            self.__pub_info.publish(info) 
        
    def set_object_pose(self, object_name : str, position : tuple, orientation : tuple) -> None:
        """Define a new pose at one scene object.

        Args:
            vehicle_name (str): _description_
            position (tuple): _description_
            orientation (tuple): _description_
        """           
        pose = self.pose(position, orientation)
        self.__client.simSetObjectPose(pose, True, object_name)
        
        if debbug:
            info = f"New Object pose was defined: {object_name} - [{position}, {orientation}]"
            self.__pub_info.publish(info) 
            
            
            
            
            
            
            
            
class Resources(Ue4Briedge):
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
    
    def __init__(self, host : str):
        super.__init__(self, host)
        self.__pub_info = rospy.Publisher("simulation_info", String, queue_size=10)
        
        
    def set_vehicle_pose(self, vehicle_name : str, position : tuple, orientation : tuple, debbug : bool = False) -> None:
        """Define a new pose at one vehicle.

        Args:
            vehicle_name (str): The same was defined on settings.json .
            position (tuple): (x,y,z).
            orientation (tuple): (roll, pitch, yaw).
        """           
        pose = self.pose(position, orientation)
        self.client.simSetVehiclePose(pose, True, vehicle_name)
        
        if debbug:
            info = f"New Vehicle pose was defined: {vehicle_name} - [{position}, {orientation}]"
            self.__pub_info.publish(info) 
        
    def set_object_pose(self, object_name : str, position : tuple, orientation : tuple) -> None:
        """Define a new pose at one scene object.

        Args:
            vehicle_name (str): _description_
            position (tuple): _description_
            orientation (tuple): _description_
        """           
        pose = self.pose(position, orientation)
        self.client.simSetObjectPose(pose, True, object_name)
        
        if debbug:
            info = f"New Object pose was defined: {object_name} - [{position}, {orientation}]"
            self.__pub_info.publish(info) 
        

@Resources(HOST)
def air_coverage(radius : float) -> None:
     