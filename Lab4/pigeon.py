#!/usr/bin/python

import sys
import math
import time
import random

from bzrc import BZRC, Command


class PredictablePigeon(object):
    """Class handles all command and control logic for a teams tanks."""
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

        self.started = False

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""

        mytanks, _, _, _ = self.bzrc.get_lots_o_stuff()

        if not self.started:
            self.commands = []
            self.commands.append(Command(mytanks[0].index, 1, 0, 0))
            results = self.bzrc.do_commands(self.commands)
            self.started = True


class WildPigeon(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.last_change = None

    def tick(self, time_diff):
        mytanks, _, _, _ = self.bzrc.get_lots_o_stuff()
        if self.last_change == None or time.time() - self.last_change > 4:
            velocity = (random.random() * 2) - 1
            angvel = random.random() * 2 * math.pi - math.pi
            self.bzrc.do_commands([
                Command(mytanks[0].index, velocity, angvel, 0)
            ])
            self.last_change = time.time()

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int(angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle


def main():
    # Process CLI arguments.
    try:
        execname, host, port, pigeon_type = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >> sys.stderr, '%s: incorrect number of arguments' % execname
        print >> sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    # bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    if pigeon_type == 'constant':
        # constant pigeon
        print 'Starting agent: Constant-velocity Pigeon'
        agent = PredictablePigeon(bzrc)
    elif pigeon_type == 'wild':
        # wild pigeon
        print 'Starting agent: Wild Pigeon'
        agent = WildPigeon(bzrc)

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
