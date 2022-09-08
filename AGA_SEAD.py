import random
import time
import math
import numpy as np
import copy
from matplotlib import pyplot as plt
from dubins_path import *
from dubins_length import *
from dubins import Dubins


class GA_task_allocation(object):

    def __init__(self, target_sites, uav_sites, mission_amount):
        # input data
        self.target_sites = target_sites
        self.uav_sites = uav_sites
        self.target_num = len(target_sites)
        self.uav_num = len(uav_sites)
        self.mission_num = mission_amount
        # GA parameters
        self.population_size = 100
        self.crossover_num = 66
        self.mutation_num = 30
        self.elitism_num = 4
        self.iteration = 300
        # mapping
        # self.node = [[] for _ in range(self.uav_num)]
        # self.cost_matrix = [[] for _ in range(self.uav_num)]
        # for i in range(self.uav_num):
        #     self.node[i].extend([self.uav_sites[i]])
        #     self.node[i].extend(self.target_sites)
        # for i in range(self.uav_num):
        #     self.cost_matrix[i] = [(lambda x: [math.sqrt(math.pow(self.node[i][x][0] - self.node[i][y][0], 2) +
        #                                                   math.pow(self.node[i][x][1] - self.node[i][y][1], 2))
        #                                         for y in range(self.target_num + 1)])(j) for j in
        #                             range(self.target_num + 1)]
        #     for j in range(self.target_num+1):
        #         self.cost_matrix[i][j][j] = 2*math.pi*self.Rmax
        # motion
        self.uan_velocity = 10
        self.Rmax = 6

    def dubins_path(self, state_list):
        kappa_ = 1./self.Rmax
        dubins = Dubins()
        distance = 0
        route_state = [[] for _ in range(3)]
        for i in range(len(state_list)):
            state_list[i][2] *= 2*np.pi/180
        for i in range(len(state_list)-1):
            start_state = state_list[i]
            goal_state = state_list[i+1] if state_list[i] != state_list[i+1] \
                else [state_list[i+1][0], state_list[i+1][1], abs(state_list[i+1][2]-1e-5)]
            cartesian_path, controls, dubins_path = dubins.plan(start_state, goal_state, kappa_)
            path_x, path_y, path_yaw = cartesian_path
            route_state[0].extend(path_x)
            route_state[1].extend(path_y)
            route_state[2].extend(path_yaw)
            distance += dubins_path.length()
        return distance, route_state

    def dubins(self, state_list, local_planner):
        distance = 0
        # route_state = [[] for _ in range(2)]
        for i in range(len(state_list)):
            state_list[i][2] *= 2 * np.pi / 180
        for i in range(len(state_list) - 1):
            start_state = state_list[i]
            goal_state = state_list[i + 1] if state_list[i] != state_list[i + 1] \
                else [state_list[i + 1][0], state_list[i + 1][1], abs(state_list[i + 1][2] - 1e-3)]
            path, length = local_planner.dubins_path(start_state, goal_state)
            # route_state[0].extend([x[0] for x in path])
            # route_state[1].extend([y[0] for y in path])
            # print(route_state)
            distance += length
        return distance

    def dubins_length(self, state_list):
        distance = 0
        # for i in range(len(state_list)):
        #     state_list[i][2] *= 2 * np.pi / 180
        for i in range(len(state_list) - 1):
            start_state = state_list[i][:]
            goal_state = state_list[i+1][:]
            Ti = TangentVector(numpy.array(start_state[0:2]), start_state[2])
            Tf = TangentVector(numpy.array(goal_state[0:2]), goal_state[2])
            distance += calcDubinsLength(Ti, Tf, self.Rmax)
        return distance

    def fitness(self, population, local_planner):
        fitness_value = []
        # for k in range(self.population_size):
        #     pre = 0
        #     binary_decision = np.zeros((self.uav_num, self.target_num + 1, self.target_num + 1))
        #     dist = np.zeros((self.uav_num))
        #     for j in range(self.target_num*2):
        #         binary_decision[population[k][3][j]][pre][population[k][1][j]] = 1
        #         pre = population[k][1][j]
        #     for i in range(self.uav_num):
        #         dist[i] = np.sum(np.multiply(self.cost_matrix[i], binary_decision[i]))
        #     fitness_value.extend([1 / (np.max(dist) + 0.01 * np.sum(dist))])
        for i in range(self.population_size):
            dist = np.zeros(self.uav_num)
            task_sequence_state = [[] for _ in range(self.uav_num)]
            for j in range(self.target_num*self.mission_num):
                task_sequence_state[population[i][3][j]-1].append([
                    self.target_sites[population[i][1][j]-1][0], self.target_sites[population[i][1][j]-1][1], population[i][4][j]])
            for j in range(self.uav_num):
                task_sequence_state[j] = [self.uav_sites[j]] + task_sequence_state[j][:] + [self.uav_sites[j]]
                dist[j] = self.dubins(task_sequence_state[j], local_planner)
            fitness_value.extend([1 / (np.max(dist) + 0.01 * np.sum(dist))])
        roulette_wheel = np.array(fitness_value)/np.sum(fitness_value)
        return fitness_value, list(roulette_wheel)

    def initiate_population(self):
        def generate_chromosome():
            chromosome = [[] for _ in range(5)]
            for j in range(self.mission_num):
                for i in range(self.target_num):
                    chromosome[0].append(i+1+self.target_num*j)  # order
                    chromosome[1].append(i+1)  # target id
                    chromosome[2].append(j+1)  # mission type
                    chromosome[3].append(random.randint(1, self.uav_num))  # uav id
                    chromosome[4].append(random.randint(0, 360))  # heading angle
            relate_type = list(zip(chromosome[1], chromosome[2]))
            random.shuffle(relate_type)
            chromosome[1] = [shuffle[0] for shuffle in relate_type]
            chromosome[2] = [shuffle[1] for shuffle in relate_type]
            return chromosome

        def generate__chromosome():
            chromosome = np.zeros((5, self.target_num*self.mission_num), dtype=int)

            for i in range(chromosome.shape[1]):
                chromosome[0][i] = i+1  # order
                chromosome[1][i] = random.choice([i for i in range(1, self.target_num+1)  # target id
                                                  if np.count_nonzero(chromosome[1] == i) < self.mission_num])
            # turn to target-based
            zipped_gene = [list(g) for g in zip(chromosome[0], chromosome[1], chromosome[2],
                                                chromosome[3], chromosome[4])]
            target_based_gene = np.array(sorted(zipped_gene, key=lambda u: u[1]))
            for i in range(target_based_gene.shape[0]):
                target_based_gene[i][2] = (i % self.mission_num)+1  # mission type
                target_based_gene[i][3] = random.randint(1, self.uav_num)  # uav id
                target_based_gene[i][4] = random.randint(0, 360)  # heading angle
            # back to order-based
            chromosome = [[] for _ in range(5)]
            order_based_gene = (sorted(target_based_gene, key=lambda u: u[0]))
            for i in range(5):
                chromosome[i] = [g[i] for g in order_based_gene]
            return chromosome
        return [generate__chromosome() for _ in range(self.population_size)]

    def selection(self, roulette_wheel):
        parent = random.choices(range(self.population_size), weights=roulette_wheel)[0]
        return parent

    def crossover(self, parent_1, parent_2):
        # turn to target-based
        target_based_gene, order_based_gene = [], []
        for parents in [parent_1, parent_2]:
            zipped_gene = [list(g) for g in zip(parents[0], parents[1], parents[2], parents[3], parents[4])]
            target_based_gene.append(sorted(zipped_gene, key=lambda u: u[1]))
        # choose cut point
        cutpoint = random.sample(range(self.target_num), 2)
        cutpoint_1, cutpoint_2 = min(cutpoint), max(cutpoint)
        child_1, child_2 = [], []
        # 2 point crossover
        target_based_gene[0][cutpoint_1:cutpoint_2], target_based_gene[1][cutpoint_1:cutpoint_2] = \
            [[b[:3] for b in target_based_gene[0][cutpoint_1:cutpoint_2]][i]+[a[3:] for a in target_based_gene[1][cutpoint_1:cutpoint_2]][i] for i in range(cutpoint_2-cutpoint_1)], \
            [[b[:3] for b in target_based_gene[1][cutpoint_1:cutpoint_2]][i]+[a[3:] for a in target_based_gene[0][cutpoint_1:cutpoint_2]][i] for i in range(cutpoint_2-cutpoint_1)]
        # back to order-based
        for gene in target_based_gene:
            order_based_gene.append(sorted(gene, key=lambda u: u[0]))
        for i in range(5):
            child_1.append([g[i] for g in order_based_gene[0]])
            child_2.append([g[i] for g in order_based_gene[1]])
        return child_1, child_2

    def mutation(self, chromosome):
        def point_mutation():
            # choose mutate point
            mutpoint = random.randint(0, len(chromosome)-1)
            # mutate assign uav or heading angle
            new_gene = [[] for _ in range(5)]
            for i in range(len(chromosome)):
                new_gene[i] = chromosome[i][:]
            if random.random() < 0.5:
                new_gene[3][mutpoint] = random.choice([i for i in range(1, self.uav_num + 1) if
                                                       i != chromosome[3][mutpoint]])
            else:
                new_gene[4][mutpoint] = random.choice([i for i in range(0, 360) if i != chromosome[4][mutpoint]])
            return new_gene

        def state_mutation():
            # turn to target-based
            zipped_gene = [list(g) for g in zip(chromosome[0], chromosome[1], chromosome[2],
                                                chromosome[3], chromosome[4])]
            target_based_gene = (sorted(zipped_gene, key=lambda u: u[1]))
            # shuffle the state
            target_sequence = list(range(self.target_num))
            random.shuffle(target_sequence)
            mutate_target_based = [[] for _ in range(self.target_num*self.mission_num)]
            for n in range(self.target_num):
                mutate_target_based[self.mission_num*n:self.mission_num*(n+1)] = \
                    [[b[:1] for b in target_based_gene[self.mission_num*n:self.mission_num*(n+1)]][i] +
                     [a[1:] for a in target_based_gene[self.mission_num*target_sequence[n]:
                                                       self.mission_num*(target_sequence[n]+1)]][i]
                     for i in range(self.mission_num)]
            # back to order-based
            new_gene = [[] for _ in range(5)]
            order_based_gene = (sorted(mutate_target_based, key=lambda u: u[0]))
            for i in range(5):
                 new_gene[i] = [g[i] for g in order_based_gene]
            return new_gene
        mut_gene = point_mutation() if random.random() < 0.7 else state_mutation()
        return mut_gene

    def elitism(self, fitness_value):
        fitness_ranking = sorted(range(len(fitness_value)), key=lambda u: fitness_value[u], reverse=True)
        elitism_id = fitness_ranking[:self.elitism_num]
        return elitism_id

    def adaptive_setting(self, Nit):
        Ncr = round((self.population_size-self.elitism_num)*math.exp(-Nit/self.iteration))
        Nmu = self.population_size - self.elitism_num - Ncr
        return Ncr, Nmu

    def GA_SEAD(self):
        start = time.time()
        fitness_curve = []
        population = self.initiate_population()
        local_planner = Dubins(radius=self.Rmax, point_separation=.5)
        fitness, wheel = self.fitness(population, local_planner)
        fitness_curve.append(1/max(fitness))
        for i in range(self.iteration):
            print(i)
            new_population = []
            crossover_num, mutation_num = self.adaptive_setting(i+1)
            for j in range(0, self.crossover_num, 2):
                parent_1, parent_2 = [self.selection(wheel) for k in range(2)]
                child_1, child_2 = self.crossover(population[parent_1], population[parent_2])
                new_population.extend([child_1, child_2])
            if crossover_num % 2 == 1:
                parent_1, parent_2 = [self.selection(wheel) for k in range(2)]
                child_1, child_2 = self.crossover(population[parent_1], population[parent_2])
                new_population.extend([child_1])
            for j in range(self.mutation_num):
                mutate_gene = self.selection(wheel)
                new_population.append(self.mutation(population[mutate_gene]))
            elitism_gene = self.elitism(fitness)
            new_population.extend([population[k] for k in elitism_gene])
            fitness, wheel = self.fitness(new_population, local_planner)
            fitness_curve.append(1/max(fitness))
            population = new_population
        print(f'consume time:{time.time()-start}')
        self.plot_result(population[fitness.index(max(fitness))], fitness_curve, local_planner)

    def plot_result(self, best_solution, performance, local_planner):
        def dubins_plot(state_list):
            distance = 0
            route_state = [[] for _ in range(2)]
            for i in range(len(state_list)):
                state_list[i][2] *= 2 * np.pi / 180
            for i in range(len(state_list) - 1):
                start_state = state_list[i]
                goal_state = state_list[i + 1] if state_list[i] != state_list[i + 1] \
                    else [state_list[i + 1][0], state_list[i + 1][1], abs(state_list[i + 1][2] - 1e-3)]
                path, length = local_planner.dubins_path(start_state, goal_state)
                route_state[0].extend([x[0] for x in path])
                route_state[1].extend([y[1] for y in path])
                distance += length
            return distance, route_state
        print(f'best gene:{best_solution}')
        dist = np.zeros(self.uav_num)
        task_sequence_state = [[] for _ in range(self.uav_num)]
        task_route = [[] for _ in range(self.uav_num)]
        route_state = [[] for _ in range(self.uav_num)]
        for j in range(self.target_num * self.mission_num):
            task_sequence_state[best_solution[3][j] - 1].append([
                self.target_sites[best_solution[1][j] - 1][0], self.target_sites[best_solution[1][j] - 1][1],
                best_solution[4][j]])
            task_route[best_solution[3][j] - 1].extend([[best_solution[1][j], best_solution[2][j]]])
        print(task_sequence_state)
        for j in range(self.uav_num):
            task_sequence_state[j] = [self.uav_sites[j]] + task_sequence_state[j] + [self.uav_sites[j]]
            task_route[j] = [0] + task_route[j] + [0]
            dist[j], route_state[j] = dubins_plot(task_sequence_state[j])
        print(f'best route:{task_route}')
        plt.subplot(121)
        for i in range(self.uav_num):
            # l = 2.0
            # plt.plot([sx, sx + l * np.cos(stheta)], [sy, sy + l * np.sin(stheta)], 'r-')
            # plt.plot([gx, gx + l * np.cos(gtheta)], [gy, gy + l * np.sin(gtheta)], 'r-')
            plt.plot(route_state[i][0], route_state[i][1], '-')
            plt.plot(self.uav_sites[i][0], self.uav_sites[i][1], 'ro')
            plt.axis("equal")
        plt.plot([b[0] for b in self.target_sites], [b[1] for b in self.target_sites], 'bo')
        plt.subplot(122)
        plt.plot(range(self.iteration+1), performance)
        plt.title('cost = {:.3f}'.format(performance[-1]))
        plt.show()


if __name__ == '__main__' :
    targets = [[random.randint(-100, 100), random.randint(-100, 100)] for m in range(20)]
    uav = [[random.randint(-100, 100),
            random.randint(-100, 100),
            random.randint(0, 360)] for n in range(3)]
    ga_sead_ = GA_task_allocation(target_sites=targets, uav_sites=uav, mission_amount=2)
    ga_sead_.GA_SEAD()
