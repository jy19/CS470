#!/usr/bin/python

import sys
import math
import time
import random
import numpy as np

from bzrc import BZRC, Command

sigma = 5

# this covariance matrix reflects how noisy our sensor is
sigma_z = np.array([[ sigma, 0 ],
                    [ 0, sigma ]], dtype=float)

# this covariance matrix allows way more noise in
# acceleration than for position and velocity
sigma_x = np.array([[ 0.1, 0, 0, 0, 0, 0 ],
                    [ 0, 0.1, 0, 0, 0, 0 ],
                    [ 0, 0, 100, 0, 0, 0 ],
                    [ 0, 0, 0, 0.1, 0, 0 ],
                    [ 0, 0, 0, 0, 0.1, 0 ],
                    [ 0, 0, 0, 0, 0, 100 ]])

h = np.array([[1, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0]])

def get_system_model(delta_t, c):
    return np.array([[1, delta_t, np.power(delta_t, 2)/2, 0, 0, 0],
                     [0, 1, delta_t, 0, 0, 0],
                     [0, -c, 1, 0, 0, 0],
                     [0, 0, 0, 1, delta_t, np.power(delta_t, 2)/2],
                     [0, 0, 0, 0, 1, delta_t],
                     [0, 0, 0, 0, -c, 1] ], dtype=float)

class KalmanState(object):
    def __init__(self, (x, y)):
        self.mean = np.array([ x, 0, 0, y, 0, 0 ], dtype=float)
        self.covariance = np.array([[ 100, 0, 0, 0, 0, 0 ],
                                    [ 0, 0.1, 0, 0, 0, 0 ],
                                    [ 0, 0, 0.1, 0, 0, 0 ],
                                    [ 0, 0, 0, 100, 0, 0 ],
                                    [ 0, 0, 0, 0, 0.1, 0 ],
                                    [ 0, 0, 0, 0, 0, 0.1 ]])

    def update(self, time_diff, (x, y)):
        self.mean[0], self.mean[3] = x, y

        system_model = get_system_model(time_diff, 0)
        (x, _, _, y, _, _) = np.dot(system_model, self.mean)

        # estimate the location of the target tank at time t+1
        predicted_covariance = np.zeros(2, dtype=float)
        return (x, y), predicted_covariance

class KalmanTank(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

        # set up our Kalman models
        self.kalman_models = {}
        _, othertanks, _, _ = self.bzrc.get_lots_o_stuff()
        for enemy in othertanks:
            self.kalman_models[enemy.callsign] = KalmanState((enemy.x, enemy.y))

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        _, othertanks, _, _ = self.bzrc.get_lots_o_stuff()
        for enemy in othertanks:
            model = self.kalman_models[enemy.callsign]
            noisy_position = (enemy.x, enemy.y)
            predicted_pos, predicted_covariance = model.update(time_diff, noisy_position)
            print "{0:10s} {1}".format(enemy.callsign, predicted_pos)

        # try to shoot people
        pass

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    print 'Starting agent: Kalman Filter shooter'
    agent = KalmanTank(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()

if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
