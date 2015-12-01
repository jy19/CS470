import sys
import time
import math
import random

import matplotlib.pyplot as plt
import matplotlib.image as image
import numpy as np

from bzrc import BZRC, Command
from pd_controller import PDController
from weighted_graph import Point

class GraphAgent(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        # self.obstacles = self.bzrc.get_obstacles()
        self.obstacles = []
        self.bases = self.bzrc.get_bases()
        for base in self.bases:
            if base.color == self.constants['team']:
                self.base = base
                break
        self.flags = bzrc.get_flags()
        # get first flag that's not own to go after
        for flag in self.flags:
            if flag.color != self.constants['team']:
                self.enemy_flag = flag
                break
        self.commands = []
        self.goal_spread = 50
        self.tank = self.bzrc.get_mytanks()[0]
        self.actual_goal_reached = False

    def set_goal(self, goal, actual_goal):
        self.goal = goal
        self.actual_goal = actual_goal

    def reached_goal(self, goal):
        # if self.tank within self.goal_spread
        dist = (goal.point[0] - self.tank.x) ** 2 + (goal.point[1] - self.tank.y) ** 2
        reached = dist < (self.goal_spread) ** 2
        if reached and self.actual_goal:
            self.actual_goal_reached = True
        return reached

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.flags = flags
        self.enemies = [tank for tank in othertanks if tank.color != self.constants['team']]

        self.enemy_color = []

        for enemy in self.enemies:
            if enemy.color not in self.enemy_color:
                self.enemy_color.append(enemy.color)

        self.commands = []
        tank = mytanks[0]
        self.tank = tank

        try:
            self.tank.controller
        except AttributeError:
            self.tank.controller = PDController()

        self.move_potential_field(self.tank)

        results = self.bzrc.do_commands(self.commands)

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        adjusted_angvel = tank.controller.calc_error(relative_angle)
        # command = Command(tank.index, 1, 2 * relative_angle, True)
        command = Command(tank.index, 1, adjusted_angvel, False)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int(angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def move_potential_field(self, tank):
        """ moves a tank based on the potential field exerted on that tank at the tick"""
        delta_x, delta_y = self.calc_potential_field(tank.x, tank.y, self.goal)
        self.move_to_position(tank, tank.x + delta_x, tank.y + delta_y)

    def calc_potential_field(self, tank_x, tank_y, goal):
        delta_x = 0
        delta_y = 0

        goal_x = goal.point[0]
        goal_y = goal.point[1]

        attractive_x, attractive_y = self.attractive_field(tank_x, tank_y, goal_x, goal_y, 10)

        delta_x += attractive_x
        delta_y += attractive_y

        if not self.actual_goal:
            # if this is an obstacle corner, add tangential field
            tangential_x, tangential_y = self.tangential_field(tank_x, tank_y, goal_x, goal_y, 5)
            delta_x += tangential_x
            delta_y += tangential_y
            if math.isnan(delta_x):
                delta_x = 0
            if math.isnan(delta_y):
                delta_y = 0

        return delta_x, delta_y

    def attractive_field(self, tank_x, tank_y, goal_x, goal_y, goal_radius, goal_spread=100, alpha_const=5.0):
        """ calculates the attractive field created by 'goals' """
        # implement the equations from that one reading..
        delta_x, delta_y = 0, 0

        # distance between agent and goal
        distance = calc_distance(goal_x, tank_x, tank_y, goal_y)
        # angle between agent and goal
        theta = calc_theta(tank_x, goal_x, tank_y, goal_y)

        if distance < goal_radius:
            delta_x = delta_y = 0
        elif goal_radius <= distance <= (goal_radius + goal_spread):
            delta_x = alpha_const * (distance - goal_radius) * math.cos(theta)
            delta_y = alpha_const * (distance - goal_radius) * math.sin(theta)
        elif distance > (goal_radius + goal_spread):
            delta_x = alpha_const * goal_spread * math.cos(theta)
            delta_y = alpha_const * goal_spread * math.sin(theta)
        return delta_x, delta_y

    def repulsive_field(self, tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius, obstacle_spread=100,
                        beta_const=1):
        """ calculates repulsive field created by obstacles """
        delta_x = delta_y = 0
        # distance = calc_distance(tank_x, obstacle_x, tank_y, obstacle_y)
        distance = calc_distance(obstacle_x, tank_x, tank_y, obstacle_y)
        theta = calc_theta(tank_x, obstacle_x, tank_y, obstacle_y)

        if distance < obstacle_radius:
            delta_x = -1.0 * math.copysign(float('inf'), math.cos(theta))
            delta_y = -1.0 * math.copysign(float('inf'), math.sin(theta))
        elif obstacle_radius <= distance <= (obstacle_radius + obstacle_spread):
            delta_x = -1.0 * beta_const * (obstacle_spread + obstacle_radius - distance) * math.cos(theta)
            delta_y = -1.0 * beta_const * (obstacle_spread + obstacle_radius - distance) * math.sin(theta)
        elif distance > (obstacle_radius + obstacle_spread):
            delta_x = delta_y = 0
        # repulsive_force = (1.0 / distance) * const
        return delta_x, delta_y

    def tangential_field(self, tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius, obstacle_spread=30,
                         beta_const=4):
        """ calculates tangential fields around obstacles """
        # could basically be repulsive field but with angle changed?
        delta_x = delta_y = 0
        distance = calc_distance(obstacle_x, tank_x, tank_y, obstacle_y)
        # add 90 degrees to theta
        theta = self.normalize_angle(calc_theta(tank_x, obstacle_x, tank_y, obstacle_y) + (math.pi / 2.0))
        if distance < obstacle_radius:
            delta_x = -1.0 * math.copysign(float('inf'), math.cos(theta))
            delta_y = -1.0 * math.copysign(float('inf'), math.sin(theta))
        elif obstacle_radius <= distance <= (obstacle_radius + obstacle_spread):
            delta_x = -1.0 * beta_const * (obstacle_spread + obstacle_radius - distance) * math.cos(theta)
            delta_y = -1.0 * beta_const * (obstacle_spread + obstacle_radius - distance) * math.sin(theta)
        elif distance > (obstacle_radius + obstacle_spread):
            delta_x = delta_y = 0
        return delta_x, delta_y


def calc_distance(x1, x2, y1, y2):
    distance = math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))
    return distance


def calc_theta(x1, x2, y1, y2):
    theta = math.atan2((y2 - y1), (x2 - x1))
    return theta

# denan = np.vectorize(lambda x : 0 if math.isnan(x) else x)

'''Grid Filter implementation is here'''
def update_occ(occ_prob, seen, worldsize, (true_pos, true_neg), ((pos_x, pos_y), grid)):

    # prepare some constants
    bx, by = (pos_x + (worldsize / 2)), (pos_y + (worldsize / 2))
    occ_subset = occ_prob[bx:bx+len(grid), by:by+len(grid[0])]
    observations = np.asarray(grid)

    # mark these pixels as seen
    seen[bx:bx+len(grid), by:by+len(grid[0])] += 1

    # evaluate Bayes Rule
    likelihoods = (observations * true_pos) + (np.abs(observations - 1) * (1 - true_pos))
    denom = (true_pos * occ_subset) + ((1 - true_neg) * (1 - occ_subset))
    occ_subset[:] = (likelihoods * occ_subset) / denom

'''Selects a new target from the yet-unexplored regions'''
def select_goal(seen, worldsize):
    unseen = []
    for x in xrange(seen.shape[1]):
        for y in xrange(seen.shape[0]):
            if seen[x,y] == 0:
                unseen.append( (x - worldsize / 2, y - worldsize / 2) )

    if len(unseen) == 0:
        return None

    return unseen[random.randint(0, len(unseen) - 1)]

def generate_waypoints(gridsize, worldsize):
    waypoints = []
    blocksize = worldsize / gridsize
    for x in xrange(gridsize):
        for y in xrange(gridsize):
            waypoints.append((x, y))
    return waypoints

def nearest_waypoint(waypoints, (cur_x, cur_y), gridsize, worldsize):
    nearest = None
    nearest_coord = None
    nearest_dist = 10000
    for x in xrange(waypoints.shape[1]):
        for y in xrange(waypoints.shape[0]):
            if waypoints[x,y] == False:

                # calc distance to this one
                (wp_x, wp_y) = waypoint_coord((x,y), gridsize, worldsize)
                distance = np.sqrt( np.power(wp_x - cur_x, 2) + np.power(wp_y - cur_y, 2) )

                # replace if nearer
                if distance < nearest_dist:
                    nearest_dist = distance
                    nearest = (x, y)
                    nearest_coord = (wp_x, wp_y)

    print "{0} ({2}) nearest {1}".format(nearest, (cur_x, cur_y), nearest_coord)
    return nearest

def waypoint_coord((x, y), gridsize, worldsize):
    blocksize = worldsize / gridsize
    return (
        x * blocksize + (blocksize / 2) - (worldsize / 2),
        y * blocksize + (blocksize / 2) - (worldsize / 2)
    )

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

    agent = GraphAgent(bzrc)
    # print "Tank located at ({0}, {1})".format(agent.tank.x, agent.tank.y)

    t0 = time.time()
    prior = 0.5

    worldsize = int(agent.constants['worldsize'])
    true_pos, true_neg = float(agent.constants['truepositive']), float(agent.constants['truenegative'])
    prob_occupied = np.ones([worldsize, worldsize]) * prior
    seen = np.zeros([worldsize, worldsize], dtype=int)
    # print 'True Positive: ', true_pos
    # print 'True Negative: ', true_neg

    cur_goal, cur_point = None, None
    iters_total = 0

    # break the world up into a grid
    gridsize = 16
    waypoints = np.zeros([ gridsize, gridsize ], dtype=bool)

    try:
        last_known, iters_here = None, 0
        while True:

            # pick a new goal randomly if we got stuck
            location = (agent.tank.x, agent.tank.y)
            if location == last_known:
                iters_here += 1
            if iters_here > 10:
                print "Got stuck, selecting new goal"
                iters_here = 0
                cur_goal = cur_point = None
            last_known = location

            if cur_goal is None or agent.reached_goal(cur_point):

                # mark waypoint as visited
                if cur_goal != None:
                    print "Reached waypoint: {0}".format(cur_goal)
                    waypoints[cur_goal] = True

                wp_nearest = nearest_waypoint(waypoints, location, gridsize, worldsize)
                if wp_nearest == None:
                    print "Reached all waypoints"
                    command = Command(agent.tank.index, 0, 0, False)
                    results = agent.bzrc.do_commands([ command ])
                    break
                else:
                    cur_goal, cur_point = wp_nearest, Point('waypoint', waypoint_coord(wp_nearest, gridsize, worldsize))
                    print "Selected new waypoint: {0}".format(wp_nearest)
                    agent.set_goal(cur_point, False)

            time_diff = time.time() - t0
            agent.tick(time_diff)
            # print "Tank located at ({0}, {1})".format(agent.tank.x, agent.tank.y)

            # update probabilities
            pos, occ = bzrc.get_occgrid(agent.tank.index)
            update_occ(prob_occupied, seen, worldsize, (true_pos, true_neg), (pos, occ))

            # maybe save image
            if iters_total % 200 == 0:
                snapshot = np.rot90(prob_occupied * 255, 1)
                image.imsave("snapshots/iteration-{0}.png".format(iters_total), snapshot, cmap='gray')

            iters_total += 1

    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()

    # save thresholded map
    print "Saving map..."
    snapshot = np.rot90(prob_occupied * 255, 1) > 1
    image.imsave("snapshots/final.png".format(iters_total), snapshot, cmap='gray')

    print "Done in {0} iterations and {1:.1f}s".format(iters_total, time.time() - t0)

    # # display probability map
    # plt.imshow(np.rot90(prob_occupied, 1) > 0.1, cmap='gray')
    # plt.show()

if __name__ == '__main__':
    main()
