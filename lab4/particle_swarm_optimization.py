import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        self.x = np.random.uniform(lower_bound, upper_bound)
        delta = upper_bound - lower_bound
        self.v = np.random.uniform(-delta, delta)
        self.best_position = np.zeros(np.size(self.x))
        self.best_value = -inf
        self.isEvalueted = False

class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.w = hyperparams.inertia_weight
        self.phip = hyperparams.cognitive_parameter
        self.phig = hyperparams.social_parameter
        self.particles = [Particle(lower_bound, upper_bound) for _ in range(hyperparams.num_particles)]
        self.best_position = np.zeros(np.size(self.particles[0].x))
        self.best_value = -inf
        self.particle = None
    
    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        return self.best_position

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        return self.best_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        self.particle = self.particles.pop()
        return self.particle.x

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        for particle in self.particles:
            rp = random.uniform(0.0, 1.0)
            rg = random.uniform(0.0, 1.0)
            particle.v = self.w * particle.v + self.phip * rp * (particle.best_position - particle.x) + self.phig * rg * (self.best_position - particle.x)
            particle.x = particle.v + particle.x


    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        if value > self.particle.best_value:
            self.particle.best_value = value
            self.particle.best_position = self.particle.x
        if value > self.best_value:
            self.best_value = value
            self.best_position = self.particle.x
        self.particle.isEvalueted = True
        self.particles.insert(0,self.particle)
        for particle in self.particles:
            if not particle.isEvalueted:
                return
        self.advance_generation()

