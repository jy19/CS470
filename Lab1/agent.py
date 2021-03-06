#!/usr/bin/python

import sys
import math
import time
import random

import show_field
from bzrc import BZRC, Command

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

        self.bases = self.bzrc.get_bases()
        for base in self.bases:
            if base.color == self.constants['team']:
                self.base = base
                break

        print "   My color: %s" % self.constants['team']

        self.obstacles = self.bzrc.get_obstacles()

    def get_flag_coords(self, flags, color):
        for flag in flags:
            if flag.color == color:
                return (flag.x, flag.y)
        return None

    def build_attractive_field(self, robot, goal, radius=10, spread=100):
        """Returns delta-x and delta-y at _robot_ given an attactive field at _goal_"""
        scaling_factor = 5
        # scaling_factor = 0.15
        distance = math.sqrt( math.pow(goal[0] - robot[0], 2) + math.pow(goal[1] - robot[1], 2) )
        angle = math.atan2(goal[1] - robot[1], goal[0] - robot[0])
        if radius <= distance and distance <= spread + radius:
            # print "Inside influence"
            return (scaling_factor * (distance - radius) * math.cos(angle), scaling_factor * (distance - radius) * math.sin(angle))
        elif distance >= spread + radius:
            # print "Far away"
            return (scaling_factor * spread * math.cos(angle), scaling_factor * spread * math.sin(angle))
        elif distance < radius:
            pass
        # print "Right on top!"
        return (0, 0)

    def build_repulsive_field(self, robot, obstacle, offset=0, radius=20, spread=100):
        scaling_factor = 0.5
        # scaling_factor = 0.25
        distance = math.sqrt( math.pow(obstacle[0] - robot[0], 2) + math.pow(obstacle[1] - robot[1], 2) )
        angle = self.normalize_angle(math.atan2(obstacle[1] - robot[1], obstacle[0] - robot[0]) + offset)
        if distance < radius:
            return ( math.copysign(float("inf"), math.cos(angle)), math.copysign(float("inf"), math.sin(angle)) )
        elif radius <= distance and distance <= radius + spread:
            return ( -1 * scaling_factor * (spread + radius - distance) * math.cos(angle), -1 * scaling_factor * (spread + radius - distance) * math.sin(angle) )
        elif distance < radius:
            pass
        return (0, 0)

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.enemy_color = []
        for enemy in self.enemies:
            if enemy.color not in self.enemy_color:
                self.enemy_color.append(enemy.color)

        self.commands = []

        for tank in self.mytanks:
            # split tanks up and send after different teams
            if tank.index < 3:
                self.control_tank(tank, self.enemy_color[0])
            elif 3 <= tank.index < 6:
                self.control_tank(tank, self.enemy_color[1])
            else:
                self.control_tank(tank, self.enemy_color[2])

        # show_field.plot_single(self.build_potential_field, [], 'attractive.png')

        results = self.bzrc.do_commands(self.commands)

    def control_tank(self, tank, enemy_color):
        # build a potential field
        robot = (tank.x, tank.y)
        pf = self.build_potential_field(robot[0], robot[1], tank.flag != '-', enemy_color)
        self.move_to_position(tank, tank.x + pf[0], tank.y + pf[1])

    def build_potential_field(self, x, y, has_flag, enemy_color):

        if has_flag:
            # print (self.base.corner1_x, self.base.corner1_y, self.base.corner2_x, self.base.corner2_y, self.base.corner3_x, self.base.corner3_y, self.base.corner4_x, self.base.corner4_y)
            base_midpoint_x = (self.base.corner1_x + self.base.corner3_x) / 2.0
            base_midpoint_y = (self.base.corner1_y + self.base.corner3_y) / 2.0
            goal = (base_midpoint_x, base_midpoint_y)
        else:
            goal = self.get_flag_coords(self.flags, enemy_color)

        # dx, dy = (0, 0)
        dx, dy = self.build_attractive_field((x, y), goal)

        # add a tangential field around each obstacle
        for obstacle in self.obstacles:
            for pt in obstacle:
                odx, ody = self.build_repulsive_field((x, y), pt, math.pi / 2)
                dx += odx
                dy += ody

        if math.isnan(dx):
            dx = 0
        if math.isnan(dy):
            dy = 0

        return (dx, dy)

    # def attack_enemies(self, tank):
    #     """Find the closest enemy and chase it, shooting as you go."""
    #     best_enemy = None
    #     best_dist = 2 * float(self.constants['worldsize'])
    #     for enemy in self.enemies:
    #         if enemy.status != 'alive':
    #             continue
    #         dist = math.sqrt((enemy.x - tank.x)**2 + (enemy.y - tank.y)**2)
    #         if dist < best_dist:
    #             best_dist = dist
    #             best_enemy = enemy
    #     if best_enemy is None:
    #         command = Command(tank.index, 0, 0, False)
    #         self.commands.append(command)
    #     else:
    #         self.move_to_position(tank, best_enemy.x, best_enemy.y)

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

    agent = Agent(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
            # break
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()

if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
