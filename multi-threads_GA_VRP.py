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
    def __init__(self, uav_id, uav_site, targets_sites, uav_specification, communication):
        threading.Thread.__init__(self)
        # configuration
        self.uav_id = uav_id
        self.uav_site = uav_site
        self.targets_sites = targets_sites
        self.targets_num = len(self.targets_sites)
        self.uav_velocity = uav_specification
        # initial mapping
        self.cost_matrix = [(lambda x: [self.time_cost(self.targets_sites[x], self.targets_sites[y])
                                        for y in range(self.targets_num)])(z) for z in range(self.targets_num)]
        self.uav2targets = [self.time_cost(self.targets_sites[_], self.uav_site) for _ in range(self.targets_num)]
        # communication between agents
        self.queue = communication
        # GA parameters
        self.pop_size = 100
        self.crossover_prob = 0.8
        self.mutation_prob = 0.1

    def time_cost(self, node1, node2):
        return np.sqrt((node2[0] - node1[0]) ** 2 + (node2[1] - node1[1]) ** 2) / self.uav_velocity

    def cost_matrix_adjust(self, uav_id):
        if len(self.cost_matrix) < len(uav_id):

    def fitness_evaluate(self, population, num, fitness):
        def fitness_function(total_cost):
            return 1/(max(total_cost) + 0.01 * sum(total_cost))
        for i, chromosome in enumerate(population):
            decision_variables = np.zeros((num, self.targets_num+1, self.targets_num+1))
            pre = 0
            for j in range(len(chromosome)):
                decision_variables[chromosome[1][i]][pre][chromosome[0][j]-1] = 1
                pre = chromosome[0][j]-1
            cost = [np.sum(c) for c in np.multiply(self.cost_matrix, decision_variables)]
            fitness[i] = fitness_function(cost)
        roulette_wheel = fitness / np.sum(fitness)
        return roulette_wheel

    def generate_population(self, num, pop_size):
        def generate_chromosome():
            chromosome = [[i+1 for i in range(self.targets_num)],
                          [random.randint(1, num) for _ in range(self.targets_num)]]
            random.shuffle(chromosome[0])
            return chromosome
        return [ generate_chromosome() for _ in range(pop_size) ]

    def selection(self, population, roulette_wheel):
        return np.random.choice(np.arange(len(population)), 2, replace=False, p=roulette_wheel)

    def crossover(self):


    def mutation(self):


    def motion_planning(self):


    def run(self):
        agents_matrix = [[], [], [], []] # id, position, velocity, current beat chromosome
        # self-uav setting
        agents_matrix[0].append(self.uav_id)
        agents_matrix[1].append(self.uav_site)
        agents_matrix[2].append(self.uav_velocity)
        agents_matrix[3].append([])
        population = []
        pop_size = 100
        uav_number = 0
        broadcast_list = [i for i in range(len(self.queue)) if i != self.uav_id-1]
        # broadcast initial state
        print(f'UAV0{self.uav_id} first broadcast')
        for q in broadcast_list:
            self.queue[q].put([self.uav_id, self.uav_site[0], self.uav_site[1], self.uav_velocity])
        time.sleep(3)
        print(f'UAV0{self.uav_id} start')
        while True:
            while self.queue[self.uav_id-1].qsize() > 0:
                packet = self.queue[self.uav_id - 1].get(timeout=1)
                if packet[0] not in agents_matrix[0]:
                    agents_matrix[0].append(packet[0])
                    agents_matrix[1].append(packet[1:3])
                    agents_matrix[2].append(packet[3])
                    agents_matrix[3].append([])
                    self.cost_matrix_adjust()
                else:
                    agent_index = agents_matrix[0].index(packet[0])
                    agents_matrix[1][agent_index] = packet[1:3]
                    agents_matrix[3][agent_index] = packet[4:]
            if uav_number != len(agents_matrix[0]):
                uav_number = len(agents_matrix[0])
                population.clear()
                pp = math.ceil(100/uav_number)
                pop_size = pp if pp % 2 == 0 else pp+1
                fitness = np.zeros(pop_size)
                self.generate_population(uav_number, pop_size)
                roulette_wheel = self.fitness_evaluate(population, uav_number, fitness)
            for k in range(pop_size):
                parents = self.selection(population, roulette_wheel)
                self.crossover()
                self.mutation()


if __name__ == '__main__':
    agents = []
    uav_position = [[0, 0], [10, 5], [-10, 10]]
    uav_configuration = [1, 1, 1]
    uav_num = len(uav_configuration)
    targets = []
    broadcast = [queue.Queue() for _ in range(uav_num)]
    # broadcast packets
    for a in range(uav_num):
        agents.append(decentralized_GA_VRP(a+1, uav_position[a], targets, uav_configuration[a], broadcast))
        agents[a].start()
    for a in range(uav_num):
        agents[a].join()
    print('Done')
