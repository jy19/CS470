import math
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

from bzrc import BZRC, Command
from pd_controller import PDController


class pfAgent(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.obstacles = self.bzrc.get_obstacles()
        self.bases = self.bzrc.get_bases()
        for base in self.bases:
            if base.color == self.constants['team']:
                self.base = base
                break
        self.commands = []

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color != self.constants['team']]

        self.enemy_color = []

        for enemy in self.enemies:
            if enemy.color not in self.enemy_color:
                self.enemy_color.append(enemy.color)

        self.commands = []

        for tank in mytanks:
            # give each tank a controller
            # if not tank.controller:
            #     tank.controller = PDController()
            try:
                tank.controller
            except AttributeError:
                tank.controller = PDController()
            # split tanks up and send after different teams
            if tank.index < 3:
                self.move_potential_field(tank, self.enemy_color[0])
            elif 3 <= tank.index < 6:
                self.move_potential_field(tank, self.enemy_color[1])
            else:
                self.move_potential_field(tank, self.enemy_color[2])

        # for tank in mytanks:
        #     self.attack_enemies(tank)

        results = self.bzrc.do_commands(self.commands)

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        adjusted_angvel = tank.controller.calc_error(relative_angle)
        # command = Command(tank.index, 1, 2 * relative_angle, True)
        command = Command(tank.index, 1, adjusted_angvel, True)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int(angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def get_flag(self, color):
        for flag in self.flags:
            if flag.color == color:
                return flag
        return None

    def move_potential_field(self, tank, flag_color):
        """ moves a tank based on the potential field exerted on that tank at the tick"""
        delta_x, delta_y = self.calc_potential_field(tank.x, tank.y, tank.flag != "-", flag_color)
        self.move_to_position(tank, tank.x + delta_x, tank.y + delta_y)

    def calc_potential_field(self, tank_x, tank_y, has_flag, flag_color):
        delta_x = 0
        delta_y = 0

        # if has_flag:
        #     base_midpoint_x = (self.base.corner1_x + self.base.corner3_x) / 2.0
        #     base_midpoint_y = (self.base.corner1_y + self.base.corner3_y) / 2.0
        #     attractive_x, attractive_y = self.attractive_field(tank_x, tank_y, base_midpoint_x, base_midpoint_y,
        #                                                        calc_distance(self.base.corner1_x, self.base.corner3_x,
        #                                                                      self.base.corner1_y,
        #                                                                      self.base.corner3_y) / 2.0)
        #     delta_x += attractive_x
        #     delta_y += attractive_y
        # else:
        #     flag = self.get_flag(flag_color)
        #     attractive_x, attractive_y = self.attractive_field(tank_x, tank_y, flag.x, flag.y,
        #                                                        float(self.constants['flagradius']), 100, 8)
        #     delta_x += attractive_x
        #     delta_y += attractive_y

        for obstacle in self.obstacles:
            # find mid-point of rectangle (they seem to be squares)
            obstacle_x = (obstacle[0][0] + obstacle[2][0]) / 2.0
            obstacle_y = (obstacle[0][1] + obstacle[1][1]) / 2.0
            obstacle_radius = 50
            repulsive_x, repulsive_y = self.repulsive_field(tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius, 50, 0.5)
            delta_x += repulsive_x
            delta_y += repulsive_y
            tangential_x, tangential_y = self.tangential_field(tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius)
            delta_x += tangential_x
            delta_y += tangential_y

        if math.isnan(delta_x):
            delta_x = 0
        if math.isnan(delta_y):
            delta_y = 0

        # put tangential field around enemy tanks
        # for enemy in self.enemies:
        #     # print enemy.x, enemy.y
        #     obstacle_x = enemy.x
        #     obstacle_y = enemy.y
        #     obstacle_radius = float(self.constants['tankradius'])
        #     tangential_x, tangential_y = self.tangential_field(tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius, 5, 2)
        #     delta_x += tangential_x
        #     delta_y += tangential_y

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

    def repulsive_field(self, tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius, obstacle_spread=100, beta_const=1):
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

    def tangential_field(self, tank_x, tank_y, obstacle_x, obstacle_y, obstacle_radius, obstacle_spread = 50, beta_const=2.5):
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

    def plot_potential_field(self):
        # number of points on potential fields graph
        num_points = 61
        # scale the world size down to graph
        grid_step = int(self.constants['worldsize']) / num_points
        world_size = int(self.constants['worldsize']) / 2

        # x y arrow locations on graph
        x = np.linspace(-world_size, world_size, num_points)
        y = np.linspace(-world_size, world_size, num_points)

        # 2d arrays, x y component of arrows at each point
        dx = np.zeros(shape=(num_points, num_points))
        dy = np.zeros(shape=(num_points, num_points))

        # skip = (slice(None, None, grid_step), slice(None, None, grid_step))
        for i in range(-world_size, world_size - grid_step, grid_step):
            for j in range(-world_size, world_size - grid_step, grid_step):
                curr_dx, curr_dy = self.calc_potential_field(i, j, False, 'purple')
                row = (i + world_size) / grid_step
                col = (j + world_size) / grid_step

                dx[col][row] = curr_dx
                dy[col][row] = curr_dy

        fig, ax = plt.subplots()
        ax.quiver(x, y, dx, dy, color='black', headwidth=3, headlength=6)

        plt.savefig('field.png')
        plt.show()


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

    # agent = Agent(bzrc)
    agent = pfAgent(bzrc)

    prev_time = time.time()

    # test plotting fields
    agent.tick(0.00)
    agent.plot_potential_field()

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
