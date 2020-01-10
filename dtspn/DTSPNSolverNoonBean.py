# -*- coding: utf-8 -*-

import sys
import os
import numpy as np
import math
import matplotlib.pyplot as plt

from invoke_LKH import *
from DubinsManeuver import DubinsManeuver as dubins

import DTSPNSolver

class DTSPNSolverNoonBean(DTSPNSolver.DTSPNSolver):

    # compute the shortest sequence based on the distance matrix (self.distances)
    def compute_TSP_sequence(self, start_idx = None, end_idx = None):
        n = len(self.distances)
        
        if start_idx != None and end_idx != None:
            M = n * np.max(np.max(self.distances))
            for i in range(n):
                self.distances[i, start_idx] = M
                self.distances[end_idx, i] = M
            self.distances[end_idx, start_idx] = 0
        
        fname_tsp = "problem"
        user_comment = "a comment by the user"
        writeTSPLIBfile_FE(fname_tsp, self.distances,user_comment)
        run_LKHsolver_cmd(fname_tsp)
        sequence = read_LKHresult_cmd(fname_tsp)
        return sequence
    
    def plan_tour(self, goals, sensing_radius, turning_radius):
        """
        Compute a DTSPN tour using the NoonBean approach.  
 
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

        n = len(goals)
        self.distances = np.zeros((n,n))	
        
        '''
        TODO - Noon-Bean approach
        '''

        print ('TODO - Noon-Bean')

        configurations = []
        for goal in goals:
            configurations.append((goal[0],goal[1],math.pi))
            
        return configurations

