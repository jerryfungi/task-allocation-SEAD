import random
import time
import math
import numpy as np
import threading
import queue
from matplotlib import pyplot as plt
from dubins import Dubins


# sub thread class
class decentralized_GA_VRP(threading.Thread):
    def __init__(self, uav_id, uav_site, targets_sites, uav_specification):
        threading.Thread.__init__(self)
        self.uav_id = uav_id
        self.uav_site = uav_site
        self.targets_sites = targets_sites
        self.targets_num = len(self.targets_sites)
        self.uav_velocity = uav_specification
        self.cost_matrix = np.zeros((self.targets_num, self.targets_num))
        self.uav2targets = np.zeros(self.targets_num)
        def time_cost(node1, node2):
            return np.sqrt((node2[0] - node1[0])**2 + (node2[1] - node1[1])**2) / self.uav_velocity
        for i in range(self.targets_num):
            self.uav2targets[i] = time_cost(self.uav_site, self.targets_sites[i])
            for j in range(self.targets_num):
                self.cost_matrix[i][j] = time_cost(self.targets_sites[i], self.targets_sites[j])
        # self.pop_size = 100
        self.crossover_prob = 0.8
        self.mutation_prob = 0.1

    def fitness(self, chromosome):
        def fitness_function(cost_value):
            return 1/cost_value
        decision_variables = np.zeros((self.targets_num, self.targets_num))
        cost = self.uav2targets[chromosome[0] - 1]
        pre = chromosome[0] - 1
        for i in range(1, self.targets_num):
            decision_variables[pre][chromosome[i] - 1] = 1
            pre = chromosome[i] - 1
        cost = cost + np.sum(self.cost_matrix * decision_variables) + self.uav2targets[pre]
        return cost

    def initiate_population(self):

    def crossover(self):

    def mutation(self):

    def elitism(self):

    def task_assignment(self, broadcast):

    def motion_planning(self):


if __name__ == '__main__':
    agents = []
    uav_num = 3
    for a in range(uav_num):
        agents.append()
