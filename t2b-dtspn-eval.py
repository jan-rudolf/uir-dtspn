#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import re
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, PathPatch

sys.path.append('dtspn')
sys.path.append('lkh')
sys.path.append('gdip')

import DTSPNSolverDecoupled as decoupled
import DTSPNSolverNoonBean as noonbean

##################################################
# Helper functions
##################################################
def pause(time = 1):
    plt.pause(time)

def plot_points(points, specs = 'b'):
    x_val = [x[0] for x in points]
    y_val = [x[1] for x in points]
    plt.plot(x_val, y_val, specs)

def plot_circle(xy, radius):
    ax = plt.gca()
    circle = Circle(xy, radius, facecolor='yellow',edgecolor="orange", linewidth=1, alpha=0.2)
    ax.add_patch(circle)

def plot_map():
    plt.clf()
    plt.axis('equal')
    plot_points(goals, 'ro')
    if sensing_radius != None:
        for goal in goals:
            plot_circle(goal, sensing_radius)

def save_figure(filename):
    plt.gcf().savefig(filename)

################################################
# Testing
################################################
#define planning problems:
    #  map file 
    #  minimum turning radius
    #  sensing radius
    #  solver type
scenarios = [
    ("./problems/mini3.txt", 0.5, 0.5, 'DTSPN-decoupled'),
    ("./problems/burma14.txt", 0.5, 0.5, 'DTSPN-decoupled'),
    ("./problems/gauntlets16.txt", 0.6, 0.6, 'DTSPN-decoupled'),
    ("./problems/att48.txt", 300, 200, 'DTSPN-decoupled'),
    ("./problems/mini3.txt", 0.5, 0.5, 'DTSPN-NoonBean'),
    ("./problems/burma14.txt", 0.5, 0.5, 'DTSPN-NoonBean'),
    ("./problems/gauntlets16.txt", 0.6, 0.6, 'DTSPN-NoonBean'),
]


for scenario in scenarios:
    # Fix the problems
    random.seed(42)

    # read config with goal positions
    goals = []
    with open(scenario[0]) as fp:
        for line in fp:
            label, x, y = line.split()
            goals.append((float(x), float(y)))

    radius = scenario[1]
    sensing_radius = scenario[2]
    solver_type = scenario[3]

    ######################################
    # plot arena and goals (and their neighborhoods)
    ######################################
    plot_map()
    plt.pause(0.1)

    ######################################
    #tour planning
    ######################################

    if solver_type == 'DTSPN-NoonBean':
        solver = noonbean.DTSPNSolverNoonBean()
    elif solver_type == 'DTSPN-decoupled':
        solver = decoupled.DTSPNSolverDecoupled()
    configurations = solver.plan_tour(goals, sensing_radius, radius)
    path,path_len = solver.configurations_to_path(configurations, radius)

    ######################################
    # plot result
    ######################################
    plot_points(path, 'b-')
    pause(2)
