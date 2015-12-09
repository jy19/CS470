#!/usr/bin/python

import sys
import math
import time
import random
import numpy as np

from bzrc import BZRC, Command
from error_ellipse import plot_enemy
import matplotlib.pyplot as plt
from pd_controller import PDController

sigma = 5

# this covariance matrix reflects how noisy our sensor is
sigma_z = np.array([[sigma, 0],
                    [0, sigma]], dtype=float)

# this covariance matrix allows way more noise in
# acceleration than for position and velocity
sigma_x = np.array([[0.1, 0, 0, 0, 0, 0],
                    [0, 0.1, 0, 0, 0, 0],
                    [0, 0, 100, 0, 0, 0],
                    [0, 0, 0, 0.1, 0, 0],
                    [0, 0, 0, 0, 0.1, 0],
                    [0, 0, 0, 0, 0, 100]])
# sigma_x = np.array([[ 0.1, 0, 0, 0, 0, 0 ],
#                     [ 0, 0, 0, 0, 0, 0 ],
#                     [ 0, 0, 0, 0, 0, 0 ],
#                     [ 0, 0, 0, 0.1, 0, 0 ],
#                     [ 0, 0, 0, 0, 0, 0 ],
#                     [ 0, 0, 0, 0, 0, 0 ]])

h = np.array([[1, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0]])


def get_system_model(delta_t, c):
    return np.array([[1, delta_t, np.power(delta_t, 2) / 2, 0, 0, 0],
                     [0, 1, delta_t, 0, 0, 0],
                     [0, -c, 1, 0, 0, 0],
                     [0, 0, 0, 1, delta_t, np.power(delta_t, 2) / 2],
                     [0, 0, 0, 0, 1, delta_t],
                     [0, 0, 0, 0, -c, 1]], dtype=float)


class KalmanState(object):
    def __init__(self, (x, y)):
        self.mean = np.array([[x], [0], [0], [y], [0], [0]], dtype=float)  # mu_t
        self.covariance = np.array([[100, 0, 0, 0, 0, 0],  # sigma_t
                                    [0, 0.1, 0, 0, 0, 0],
                                    [0, 0, 0.1, 0, 0, 0],
                                    [0, 0, 0, 100, 0, 0],
                                    [0, 0, 0, 0, 0.1, 0],
                                    [0, 0, 0, 0, 0, 0.1]])

    def update(self, time_diff, (x, y)):
        system_model = get_system_model(time_diff, 0)

        common = np.dot(np.dot(system_model, self.covariance), system_model.transpose()) + sigma_x
        # print 'common', common.shape, common

        gain = np.dot(np.dot(common, h.transpose()), np.linalg.inv(np.dot(np.dot(h, common), h.transpose()) + sigma_z))
        # print 'k_t+1', gain.shape, gain

        observation = np.array([[x], [y]], dtype=float)
        self.mean = np.dot(system_model, self.mean) + np.dot(gain,
                                                             (observation - np.dot(np.dot(h, system_model), self.mean)))
        # print 'mu_t+1', newMu.shape, newMu

        self.covariance = np.dot(np.identity(6) - np.dot(gain, h), common)
        # print 'sigma_t+1', newSigma.shape, newSigma

    def predict(self, time_diff):
        system_model = get_system_model(time_diff, 0)
        prediction = np.dot(system_model, self.mean)
        # print prediction

        return (prediction[0][0], prediction[3][0])


class KalmanTank(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

        # set up our Kalman models
        self.kalman_models = {}
        self.othertanks = self.bzrc.get_othertanks()
        self.iterations = 0
        for enemy in self.othertanks:
            self.kalman_models[enemy.callsign] = KalmanState((enemy.x, enemy.y))
        self.tank = self.bzrc.get_mytanks()[0]
        self.curr_target = self.othertanks[0]

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        try:
            self.tank.controller
        except AttributeError:
            self.tank.controller = PDController()
        print 'current target: ', self.curr_target.callsign

        for enemy in self.othertanks:
            model = self.kalman_models[enemy.callsign]
            noisy_position = (enemy.x, enemy.y)
            if enemy.callsign == "green0":
                print "{0:10s} > {1}".format(enemy.callsign, noisy_position)
            model.update(time_diff, noisy_position)
            predicted_pos = model.predict(time_diff)
            if enemy.callsign == "green0":
                print "{0:10s} < {1}".format(enemy.callsign, predicted_pos)
            if enemy.callsign == self.curr_target.callsign:
                self.shoot_target(self.tank, predicted_pos[0], predicted_pos[1])

        self.iterations += 1

        if self.iterations % 100 == 0:
            # show plots of enemy tanks every 10 iterations
            for enemy in self.othertanks:
                self.plot_enemy_states(enemy)

    def shoot_target(self, tank, target_x, target_y):
        """Set command to shoot at given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        adjusted_angvel = tank.controller.calc_error(relative_angle)
        # set speed to 0 so don't move
        # for stationary target:
        # todo not sure how to get tank to stop turning unless somehow change angvel to 0 when facing right direction
        # tank.angle never seems to change
        print 'shoot target, relative angle..', relative_angle
        command = Command(tank.index, 0, adjusted_angvel, True)
        results = self.bzrc.do_commands([command])

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int(angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def plot_enemy_states(self, tank):
        kalman = self.kalman_models[tank.callsign]
        mean = (kalman.mean[0, 0], kalman.mean[3, 0])
        cov = [[kalman.covariance[0, 0], kalman.covariance[0, 3]], [kalman.covariance[3, 0], kalman.covariance[3, 3]]]
        print '----------------'
        print tank.callsign, mean, cov
        print '----------------'
        plot_enemy(mean, cov, '{0}-{1}'.format(tank.callsign, self.iterations))
        # plt.savefig('plots/{0}-{1}.png'.format(tank.callsign, self.iterations))


def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >> sys.stderr, '%s: incorrect number of arguments' % execname
        print >> sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    # bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    print 'Starting agent: Kalman Filter shooter'
    agent = KalmanTank(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            now = time.time()
            time_diff = now - prev_time
            agent.tick(time_diff)
            prev_time = now
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
