# -*- coding: utf-8 -*-

import sys
import os
import numpy as np
import math
import matplotlib.pyplot as plt

from scipy.spatial import distance_matrix

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

    def m_generate_radians(self, samples_count):
        """ Generate linearly radians from 0 to 2 Pi with samples_count samples."""
        samples = np.linspace(0, 2*np.pi, num=samples_count + 1)
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
        self.distances = distance_matrix(goals, goals)  # np.ones(self.distances.shape)

        sequence = self.compute_TSP_sequence()
        
        '''
        print('TODO sampling')
        TODO - homework
            - Sample the configurations ih the goal areas
            - Find the shortest tour
        '''
        number_border_points = 8
        number_border_points_angles = 8

        border_points_rads = self.m_generate_radians(number_border_points)
        border_points_angles_rads = self.m_generate_radians(number_border_points_angles)

        # compute configurations
        m_configurations = []

        for seq_idx in sequence:
            new_configurations_per_goal = list()
            goal = goals[seq_idx]
            goal_x = goal[0]
            goal_y = goal[1]

            for border_point_rad in border_points_rads:
                new_configuration = list()
                new_x = goal_x + sensing_radius * np.cos(border_point_rad)
                new_y = goal_y + sensing_radius * np.sin(border_point_rad)

                for border_points_angle_rads in border_points_angles_rads:
                    new_point = (new_x, new_y, border_points_angle_rads)
                    new_configuration.append(new_point)

                new_configurations_per_goal.append(new_configuration)

            m_configurations.append(new_configurations_per_goal)

        m_configurations.append(m_configurations[0])  # add the first configuration as a goal

        selected_configurations = []
        for a in range(n):
            selected_configurations.append((goals[sequence[a]][0], goals[sequence[a]][1], math.pi))

        return selected_configurations
