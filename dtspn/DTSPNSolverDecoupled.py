# -*- coding: utf-8 -*-

import sys
import os
import numpy as np
import math
import matplotlib.pyplot as plt

from invoke_LKH import *
from DubinsManeuver import DubinsManeuver as dubins

import DTSPNSolver

class DTSPNSolverDecoupled(DTSPNSolver.DTSPNSolver):

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

    def m_generate_angles(self, samples_count):
        """ Generate linearly degrees from 0 to 360 with samples_count samples."""
        samples = np.linscape(0, 360, num=samples_count + 1)
        return samples[:samples_count]
    
    def plan_tour(self, goals, sensing_radius, turning_radius):
        """
        Compute a DTSPN tour using the decoupled approach.  
 
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
        self.distances = np.zeros((n, n))
        self.paths = {}

        '''
        TODO - homework
            - Compute the distance between the individual goals  
        '''
        print ('TODO distances')
        self.distances = np.ones(self.distances.shape)


        sequence = self.compute_TSP_sequence()
        
        '''
        TODO - homework
            - Sample the configurations ih the goal areas
            - Find the shortest tour
        '''
        print ('TODO sampling')
        selected_configurations = []
        for a in range(n):
            selected_configurations.append ( ( goals[sequence[a]][0], goals[sequence[a]][1], math.pi ) )


        return selected_configurations

