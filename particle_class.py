from numpy.random import uniform
from filterpy.monte_carlo import systematic_resample
from numpy.linalg import norm
from numpy.random import randn
import scipy.stats
import numpy as np
import random
from scipy.stats import multivariate_normal


class ParticleFilter(object):
    def __init__(self, NUM_P, PART_DIM, x_range, y_range, z_range, cov_mat):
        self.NUM_P = NUM_P
        self.PART_DIM = PART_DIM
        self.particles = self.create_uniform_particles(x_range, y_range, z_range)
        p=1
        self.std_x = 0.05*p
        self.std_y = 0.05*p
        self.std_z = 0.01*p  # should be smaller than std_x and std_y
        self.cov_mat_resample = cov_mat  # tuning knob for resampling
        # print(self.particles)
        # exit()

    def create_uniform_particles(self, x_range, y_range, z_range):

        particles = np.empty((self.NUM_P, self.PART_DIM))
        particles[:, 0] = uniform(x_range[0], x_range[1], size=self.NUM_P)
        particles[:, 1] = uniform(y_range[0], y_range[1], size=self.NUM_P)
        particles[:, 2] = uniform(z_range[0], z_range[1], size=self.NUM_P)

        # print particles
        return particles

    def create_fake_particles(self):
        """creates fake particles for debugging"""
        self.particles = np.empty((self.NUM_P, self.PART_DIM))
        self.particles[:, 0] = 5
        self.particles[:, 1] = 6
        self.particles[:, 2] = 7

    def create_different_fake_particles(self):
        """creates fake particles for debugging, all particles at zero except first one"""
        self.particles = np.zeros((self.NUM_P, self.PART_DIM))
        self.particles[0, 0] = 2
        self.particles[0, 1] = 3
        self.particles[0, 2] = 0

    def predict(self):
        """Gaussian added noise to each particle std_x/y/z/ defined in init"""
        noise_x = np.random.normal(0, self.std_x, self.NUM_P)
        noise_y = np.random.normal(0, self.std_y, self.NUM_P)
        noise_z = np.random.normal(0, self.std_z, self.NUM_P)
        self.particles[:, 0] += noise_x
        self.particles[:, 1] += noise_y
        self.particles[:, 2] += noise_z

    def update(self, measurements):
        """"""

        # get number measurements (number of seen tags), dimension of each measurement is 1 -> distance to tag
        measurements=measurements.reshape((-1,4))
        num_meas = measurements.shape[0]
        distances_measured = measurements[:, 0]
        # measurements [ri,xi,yi,zi] i=0 to number of measurements
        distances_particles = np.zeros((self.NUM_P, num_meas))  # initialize distances
        for i in range(self.NUM_P):
            distances_particles[i, :] = np.linalg.norm(self.particles[i, :] * np.ones((num_meas, self.PART_DIM)) -
                                                       measurements[:, 1:4], axis=1)
        # print()
        # print "geschaetzte messungen: " + str(distances_particles)

        # covariance matrix (diagonal)
        m = np.zeros((num_meas, num_meas))  # if dimension of each measurement is > 1: num_meas*dim_meas
        for ind in range(num_meas):
            m[ind][ind] = self.cov_mat_resample * self.cov_mat_resample
        cov_matrix = m

        weights = np.zeros((self.NUM_P, 1))  # initialize weights
        # print weights

        for i in range(self.NUM_P):
            weights[i] = multivariate_normal.pdf(distances_particles[i], mean=distances_measured, cov=cov_matrix)
        weights += 1.e-300  # avoid round-off to zero
        weights /= sum(weights)  # normalize
        # print weights

        # print "before resampling: " + str(self.particles)
        self.resample(weights)
        # print "after resampling: " + str(self.particles)

    def estimate(self):

        position = np.array(([np.mean(self.particles[:, 0]), np.mean(self.particles[:, 1]),
                              np.mean(self.particles[:, 2])]))
        return position

    def resample(self, weights):
        """resample particles proportional to weights"""

        index = int(random.random() * self.NUM_P)
        beta = 0.0
        mw = np.amax(weights)
        for i in range(self.NUM_P):
            beta += random.random() * 2.0 * mw

            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % self.NUM_P

            self.particles[i, :] = self.particles[index, :]
