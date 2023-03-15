# -*- coding: utf-8 -*-
"""
Environment for Reinforcement Learning Algorithm 

V1.0 

Milosz Placzek


Custom set-up to work alongise API_MAIN.py and the V-REP Model

"""

from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
import random 


class VrepEnvironment(Env):
    def __init__(self):
        super(VrepEnvironment,self).__init__()
        