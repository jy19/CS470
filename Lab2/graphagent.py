import sys
import time
import math

from bzrc import BZRC, Command
from pd_controller import PDController

from weighted_graph import Point, Polygon, buildVisibilityGraph


class GraphAgent(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.obstacles = self.bzrc.get_obstacles()
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
        self.goal_spread = 300
        self.tank = self.bzrc.get_mytanks()[0]

    def set_goal(self, goal, actual_goal):
        self.goal = goal
        self.actual_goal = actual_goal

    def reached_goal(self, goal):
        # if self.tank within self.goal_spread
        dist = (goal.point[0] - self.tank.x) ** 2 + (goal.point[1] - self.tank.y) ** 2
        return dist < self.goal_spread

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

        # don't know what to put for radius -- use 10 for now?
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

    def tangential_field(self, tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius, obstacle_spread=50,
                         beta_const=2.5):
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

    prev_time = time.time()

    # form visibility graph and get path
    start = Point('base', (agent.tank.x, agent.tank.y))
    goal = Point('goal', (agent.enemy_flag.x, agent.enemy_flag.y))

    obstacles = []
    i = 0
    for obstacle in agent.obstacles:
        print 'obstacle-{0}'.format(i)
        curr_obstacle = Polygon('obstacle-{0}'.format(i), obstacle)
        obstacles.append(curr_obstacle)
        i += 1

    vg = buildVisibilityGraph([start, goal], obstacles)
    # get path
    path, cost = vg.aStarSearch(start, goal)

    print 'graph built'
    # Run the agent
    try:
        # go to next goal
        i = 1
        curr_goal = path[i]
        # setting goal so that once a vertex is reached, agent will head for next in path
        agent.set_goal(curr_goal, False)

        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
            # print agent.tank.x, agent.tank.y
            # print agent.goal
            if agent.reached_goal(curr_goal):
                # todo stop increasing i when actual goal reached
                i += 1
                curr_goal = path[i]
                # if goal is not the actual goal..(the last in path)
                if i == len(path) - 1:
                    agent.set_goal(curr_goal, True)
                else:
                    agent.set_goal(curr_goal, False)

            # break
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()
