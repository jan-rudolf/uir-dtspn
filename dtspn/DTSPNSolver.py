# -*- coding: utf-8 -*-

import sys
import os
import numpy as np
import math
import matplotlib.pyplot as plt

from DubinsManeuver import DubinsManeuver as dubins

def dist_euclidean_squared(coord1, coord2):
    (x1, y1) = coord1
    (x2, y2) = coord2
    return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)

def dist_euclidean(coord1, coord2):
    return math.sqrt(dist_euclidean_squared(coord1, coord2))

class DTSPNSolver:
    def __init__(self):
        pass

    
    def plan_tour_decoupled(self, goals, sensing_radius, turning_radius):
        """
        Compute a DTSPN tour.  
 
        Parameters
        ----------
        goals: list (float, float)
            list of the TSP goal coordinates  
        sensing_radius: float
            neighborhood of TSP goals  
        turning_radius: float
            turning radius for the Dubins vehicle model  
 
        Returns
        -------
        list (float,float,float)
            tour as a list of robot configurations (x,y,phi), each corresponding to a TSP goal
        """

        return None

    def configurations_to_path(self, configurations, turning_radius):    

        n = len(configurations)

        path = []
        path_len = 0.
        for a in range(n):
            b = (a+1) % n
            start = configurations[a]
            end = configurations[b]
            step_size = 0.01 * turning_radius
            dubins_path = dubins.shortest_path(start, end, turning_radius)
            segment_len = dubins_path.get_length()
            step_configurations, _ = dubins_path.sample_many(step_size)
            path = path + step_configurations
            path_len += segment_len

        return path,path_len
