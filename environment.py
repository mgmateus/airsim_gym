#!/usr/bin/env python3

import rospy

from .airsim_simulation_resources.simulation import Ue4Briedge


class Env(Ue4Briedge):
    def __init__(self) -> None:
        Ue4Briedge.__init__(self)