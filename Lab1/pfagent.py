import math
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

from bzrc import BZRC, Command

class pf_agent(object):
    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.obstacles = self.bzrc.get_obstacles() # put this here since obstacles don't move, only have to get once
        # obstacles is list of list
        # print self.obstacles
        self.commands = []


    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color != self.constants['team']]

        self.commands = []

        # todo check that if tank is holding a flag, return to base (Base is goal instead)
        for tank in mytanks:
            self.move_potential_field(tank)

        # for tank in mytanks:
        #     self.attack_enemies(tank)

        results = self.bzrc.do_commands(self.commands)

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

    def move_potential_field(self, tank):
        """ moves a tank based on the potential field exerted on that tank at the tick"""
        delta_x, delta_y = self.calc_potential_field(tank.x, tank.y)
        self.move_to_position(tank, tank.x + delta_x, tank.y + delta_y)

    def calc_potential_field(self, tank_x, tank_y):
        delta_x = 0
        delta_y = 0
        # temporarily grabs first flag that is not own color
        for flag in self.flags:
            # todo calculate flags' attractive field
            if flag.color != self.constants['team']:
                attractive_x, attractive_y = self.attractive_field(tank_x, tank_y, flag)
                delta_x += attractive_x
                delta_y += attractive_y
                break

        for obstacle in self.obstacles:
            # find mid-point of rectangle (they seem to be squares)
            obstacle_x = (obstacle[0][0] + obstacle[2][0]) / 2.0
            obstacle_y = (obstacle[0][1] + obstacle[1][1]) / 2.0
            repulsive_x, repulsive_y = self.repulsive_field(tank_x, tank_y, obstacle_x, obstacle_y)
            delta_x += repulsive_x
            delta_y += repulsive_y

        return delta_x, delta_y

    def attractive_field(self, tank_x, tank_y, flag):
        """ calculates the attractive field created by 'goals' """
        # temporary function for calculating attractive field..distance times some constant
        # so the farther away the tank is, the stronger the attractive field
        const = 10
        distance = calc_distance(tank_x, flag.x, tank_y, flag.y)
        attractive_force = distance * const
        return attractive_force, attractive_force

    def repulsive_field(self, tank_x, tank_y, obstacle_x, obstacle_y):
        """ calculates repulsive field created by obstacles """
        # for now repulsion is inverse of distance to the obstacle
        # so as tank approaches obstacle, the repulsive force approaches infinite
        # some constant to make the repulsive force stronger
        const = 5
        distance = calc_distance(tank_x, obstacle_x, tank_y, obstacle_y)
        repulsive_force = (1.0 / distance) * const
        return repulsive_force, repulsive_force

    def plot_potential_field(self):
        # number of points on potential fields graph
        num_points = 30
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
                curr_dx, curr_dy = self.calc_potential_field(i, j)
                row = (i + world_size)/grid_step
                col = (j + world_size)/grid_step

                dx[row][col] = curr_dx
                dy[row][col] = curr_dy

        fig, ax = plt.subplots()
        ax.quiver(x, y, dx, dy, color='black', headwidth=3, headlength=5)

        plt.savefig('field.png')
        plt.show()



def calc_distance(x1, x2, y1, y2):
    distance = math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))
    if distance == 0:
        distance = 1
    return distance

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

    # agent = Agent(bzrc)
    agent = pf_agent(bzrc)

    prev_time = time.time()

    # test plotting fields
    agent.tick(0.00)
    agent.plot_potential_field()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
            break
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()