#!/usr/bin/python

import sys
import math
import time
import random

from bzrc import BZRC, Command

class Agent(object):
	"""Class handles all command and control logic for a teams tanks."""

	def __init__(self, bzrc):
		self.bzrc = bzrc
		self.constants = self.bzrc.get_constants()
		self.commands = []

		self.next_action = False
		self.next_fire = 0

	def tick(self, time_diff):
		"""Some time has passed; decide what to do next."""
		mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
		self.mytanks = mytanks
		# self.othertanks = othertanks
		# self.flags = flags
		# self.shots = shots
		# self.enemies = [tank for tank in othertanks if tank.color !=
		#                 self.constants['team']]

		self.commands = []

		if not self.next_action:
			self.next_action = ( 'move_start', 0 )
		elif time.time() > self.next_action[1]:
			if self.next_action[0] is 'move_start':
				for tank in self.mytanks:
					self.commands.append(Command(tank.index, 1, 0, 0))
				self.next_action = ( 'move_stop', time.time() + (3 + random.random() * 5) )
			elif self.next_action[0] is 'move_stop':
				for tank in self.mytanks:
					self.commands.append(Command(tank.index, 0, 0, 0))
				self.next_action = ( 'turn_start', 0 )
			elif self.next_action[0] is 'turn_start':
				for tank in self.mytanks:
					self.commands.append(Command(tank.index, 0, 1 * pow(-1, random.randint(0, 1)), 0))
				self.next_action = ( 'turn_stop', time.time() + 1.5 )
			elif self.next_action[0] is 'turn_stop':
				for tank in self.mytanks:
					self.commands.append(Command(tank.index, 0, 0, 0))
				self.next_action = ( 'move_start', 0 )

		if time.time() > self.next_fire:
			for tank in self.mytanks:
				self.commands.append(Command(tank.index, -255, -255, 1))
			self.next_fire = time.time() + 1.5 + random.random()

		results = self.bzrc.do_commands(self.commands)

	def attack_enemies(self, tank):
		"""Find the closest enemy and chase it, shooting as you go."""
		best_enemy = None
		best_dist = 2 * float(self.constants['worldsize'])
		for enemy in self.enemies:
			if enemy.status != 'alive':
				continue
			dist = math.sqrt((enemy.x - tank.x)**2 + (enemy.y - tank.y)**2)
			if dist < best_dist:
				best_dist = dist
				best_enemy = enemy
		if best_enemy is None:
			command = Command(tank.index, 0, 0, False)
			self.commands.append(command)
		else:
			self.move_to_position(tank, best_enemy.x, best_enemy.y)

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
	except KeyboardInterrupt:
		print "Exiting due to keyboard interrupt."
		bzrc.close()

if __name__ == '__main__':
	main()

# vim: et sw=4 sts=4
