# -*- coding: utf-8 -*-

import sys
import os
import numpy as np
import math
from copy import deepcopy
import matplotlib.pyplot as plt
import heapq

from scipy.spatial import distance_matrix

from invoke_LKH import *
from DubinsManeuver import DubinsManeuver as dubins

import DTSPNSolver


class MNodeState:
    NOT_FOUND = 0
    OPEN = 1
    CLOSE = 2


class MNode:
    def __init__(self, data):
        self.data = data  # tuple (x, y, angle)

        self.parent = None
        self.h = sys.maxsize
        self.state = MNodeState.NOT_FOUND

        self.neighbors = list()
        self.neighbors_costs = list()


class MGraph:
    def __init__(self, graph, graph_nodes):
        self.graph = graph

        self.graph_nodes = graph_nodes

    def _init_search(self):
        for node in self.graph_nodes:
            node.parent = None
            node.h = sys.maxsize
            self.state = MNodeState.NOT_FOUND

    def _reconstruct_path(self, goal_node_idx):
        path = list()
        node = self.graph[len(self.graph) - 1][goal_node_idx]
        path_cost = node.h
        while node:
            path.insert(0, node.data)
            node = node.parent
        return path, path_cost

    def plan(self, start_node_idx):
        """ Searchs the graph. Starts and ends in a graph node on index start_node_idx. """

        self._init_search()

        start_node = self.graph[0][start_node_idx]
        start_node.h = 0
        start_node.state = MNodeState.OPEN

        m_heap = []
        heapq.heappush(m_heap, (start_node.h, start_node))

        while len(m_heap) > 0:
            _, v = heapq.heappop(m_heap)
            for w_idx, w in enumerate(v.neighbors):
                if w.state == MNodeState.CLOSE:
                    continue
                edge_len = v.neighbors_costs[w_idx]
                if w.h > v.h + edge_len:
                    w.h = v.h + edge_len
                    w.state = MNodeState.OPEN
                    w.parent = v
                    heapq.heappush(m_heap, (w.h, w))
            v.state = MNodeState.CLOSE

        path, path_cost = self._reconstruct_path(start_node_idx)

        return path, path_cost


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
        return np.linspace((2*np.pi)/samples_count, 2*np.pi, num=samples_count)
    
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
        number_border_points = 15
        number_border_points_angles = 8

        border_points_rads = self.m_generate_radians(number_border_points)
        border_points_angles_rads = self.m_generate_radians(number_border_points_angles)

        # compute configurations and graph nodes
        m_graph = []
        m_graph_nodes = []

        tmp_sequence = deepcopy(sequence)
        tmp_sequence.append(sequence[0])

        for seq_idx in tmp_sequence:
            new_configuration_level = list()
            goal = goals[seq_idx]
            goal_x = goal[0]
            goal_y = goal[1]

            for border_point_rad in border_points_rads:
                new_x = goal_x + sensing_radius * np.cos(border_point_rad)
                new_y = goal_y + sensing_radius * np.sin(border_point_rad)

                for border_points_angle_rads in border_points_angles_rads:
                    new_point = (new_x, new_y, border_points_angle_rads)
                    new_graph_node = MNode(new_point)

                    m_graph_nodes.append(new_graph_node)

                    new_configuration_level.append(new_graph_node)

            m_graph.append(new_configuration_level)

        # connecting graph nodes and computing edges costs

        for configuration_level_idx, configuration_level in enumerate(m_graph):
            if configuration_level_idx == len(m_graph) - 1:
                break
            for configuration in configuration_level:
                configuration_level_next = m_graph[configuration_level_idx + 1]

                for configuration_next in configuration_level_next:
                    edge_cost = dubins.shortest_path(configuration, configuration_next, turning_radius).get_length()

                    configuration.neighbors.append(configuration_next)
                    configuration.neighbors_costs.append(edge_cost)

        # search in the graph
        m_graph_planner = MGraph(m_graph, m_graph_nodes)

        best_path = None
        best_path_cost = sys.maxsize

        for start_node_idx, start_node in enumerate(m_graph[0]):
            path, path_cost = m_graph_planner.plan(start_node_idx)

            if path_cost < best_path_cost:
                best_path = path
                best_path_cost = path_cost

        print('Best path ', best_path_cost, best_path)

        selected_configurations = []
        for a in range(n):
            selected_configurations.append((goals[sequence[a]][0], goals[sequence[a]][1], math.pi))

        return selected_configurations
