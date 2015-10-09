#!/usr/bin/env python

import math
import heapq


def distance(vfrom, vto):
    return math.sqrt(math.pow(vto.point[0] - vfrom.point[0], 2) + math.pow(vto.point[1] - vfrom.point[1], 2))


'''
A WeightedDirectedGraph is a graph of vertexes and directed, weighted edges.
A vertex can be anything, but must be equatable and hashable. Edges can always
be anything.
'''


class WeightedDirectedGraph:
    def __init__(self):
        self.graph = {}

    '''vertex can be anything that is hashable'''

    def addVertex(self, vertex):
        if self.graph.has_key(vertex):
            raise Exception('Key already exists')
        # print "Adding vertex: {0}".format(vertex)
        self.graph[vertex] = []

    '''an edge is a weighted path between two vertices'''

    def addEdge(self, vfrom, vto, weight):
        if not self.graph.has_key(vfrom):
            raise Exception("Start vertex is not known to the graph: {0}".format(vfrom))
        if not self.graph.has_key(vto):
            raise Exception("End vertex is not known to the graph: {0}".format(vto))
        # print "Adding edge: {0} -> {1}; weight={2}".format(vfrom, vto, weight)
        self.graph[vfrom].append((vto, weight))

    '''implements the A* search algorithm'''

    def aStarSearch(self, start, goal):
        if not self.graph.has_key(start) or not self.graph.has_key(goal):
            raise Exception("Start and goal are not both vertexes in graph")

        frontier, visited = [], []
        best_cost, came_from = {}, {}

        def frontier_append(g, node):
            h = distance(node, goal)
            heapq.heappush(frontier, (g + h, g, node))

        def in_frontier(search_target):
            for cost, g, node in frontier:
                if node == search_target:
                    return True
            return False

        # start at the start node, obviously
        frontier_append(0, start)

        # we'll keep going as long as there's stuff in the frontier
        # but if we reach the goal we'll just immediately break out
        while len(frontier) > 0:

            # grab the most promising step in the frontier
            cost, g, current = heapq.heappop(frontier)

            # if it's the goal, we win! trace the path
            if current == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path, cost

            # mark as visited
            visited.append(current)

            # add each child to the frontier (if we haven't seen it yet)
            for target, weight in self.graph[current]:
                if target in visited:
                    continue

                reach_cost = g + weight

                if not best_cost.has_key(target) or reach_cost < best_cost[target]:
                    best_cost[target] = reach_cost  # record the cost of getting there
                    came_from[target] = current  # record how we got there
                    if not in_frontier(target):
                        frontier_append(reach_cost, target)

        raise Exception("no valid paths exist")

    def breadthFirstSearch(self, start, goal):
        if not self.graph.has_key(start) or not self.graph.has_key(goal):
            raise Exception("Start and goal are not both vertexes in graph")

        frontier, visited, came_from = [(start, 0)], [], {}
        while len(frontier) > 0:
            current, cost = frontier.pop(0)
            for target, weight in self.graph[current]:

                # we might have seen this node already
                if target in visited:
                    continue

                # stick it in the frontier
                came_from[target] = current
                frontier.append((target, cost + weight))
                visited.append(target)

                # but if it's the goal, bail and win
                if target == goal:
                    path = [goal]
                    print came_from
                    while path[-1] != start:
                        path.append(came_from[path[-1]])
                    path.reverse()
                    return path, cost + weight

        raise Exception("no valid paths exist")

    def depth_first_search(self, start, goal):
        if start not in self.graph or goal not in self.graph:
            raise Exception()

        frontier, visited, came_from = [(start, 0, [start])], [], {}
        while frontier:
            current, cost, path = frontier.pop()
            # if current in visited:
            #     continue
            # if current == goal:
            #     print came_from
            #     path = [goal]
            #     while path[-1] != start:
            #         path.append(came_from[path[-1]])
            #     path.reverse()
            #     return path, cost
            # visited.append(current)
            # children = self.graph[current]
            # for child in children:
            #     frontier.append((child[0], child[1]))
            #     print current, child
            #     came_from[child[0]] = current
            for next, weight in self.graph[current]:
                if next in visited:
                    continue
                visited.append(next)
                if next == goal:
                    return (path + [next]), (cost + weight)
                else:
                    frontier.append((next, cost + weight, path + [next]))

        raise Exception("no valid paths exist")

    def __str__(self):
        vertices = len(self.graph.keys())
        edges = reduce(lambda ct, v: ct + len(self.graph[v]), self.graph, 0)
        return "<WeightedDirectedGraph vertices={0} edges={1} >".format(vertices, edges)


'''
A Point is a name and a tuple (x, y). It is equatable and hashable so it can
be used as a vertex in a WeightedDirectedGraph.
'''


class Point:
    def __init__(self, name, point):
        self.name = name
        self.point = point

    def __eq__(self, other):
        return self.name == other.name and self.point == other.point

    def __hash__(self):
        return hash((self.name, self.point))

    def __repr__(self):
        return "<Point \"{0}\" ({1}, {2})>".format(self.name, self.point[0], self.point[1])


class Polygon:
    def __init__(self, name, points):
        self.name = name
        self.points = points


'''
buildVisibilityGraph uses the points and polygons to generate a
WeightedDirectedGraph that represents the correct visibility graph for each
point and every vertex of each polygon, removing edges that intersect with
a polygon's edge
'''


def buildVisibilityGraph(points, polygons):
    g = WeightedDirectedGraph()

    def intersects(v1, v2):
        for poly in polygons:
            for i in range(len(poly.points)):
                vfrom = Point("{0}_{1}".format(poly.name, i), poly.points[i])
                vtoi = (i + 1) % len(poly.points)
                vto = Point("{0}_{1}".format(poly.name, vtoi), poly.points[vtoi])
                if v1 == vfrom or v1 == vto or v2 == vfrom or v2 == vto:
                    continue
                isect_pt = pts_intersect(v1.point, v2.point, vfrom.point, vto.point)
                if isect_pt is not None:
                    # print "Lines intersect: {4}:{0} {5}:{1} {6}:{2} {7}:{3}".format(v1.point, v2.point, vfrom.point, vto.point, v1.name, v2.name, vfrom.name, vto.name)
                    return True
        return False

    def pts_intersect(A, B, C, D):
        # more or less http://stackoverflow.com/a/1968345
        A, B = (float(A[0]), float(A[1])), (float(B[0]), float(B[1]))
        C, D = (float(C[0]), float(C[1])), (float(D[0]), float(D[1]))
        Bx_Ax = B[0] - A[0]
        By_Ay = B[1] - A[1]
        Dx_Cx = D[0] - C[0]
        Dy_Cy = D[1] - C[1]
        determinant = (-Dx_Cx * By_Ay + Bx_Ax * Dy_Cy)
        if abs(determinant) < 1e-20:
            return None
        s = (-By_Ay * (A[0] - C[0]) + Bx_Ax * (A[1] - C[1])) / determinant
        t = (Dx_Cx * (A[1] - C[1]) - Dy_Cy * (A[0] - C[0])) / determinant
        if s >= 0 and s <= 1 and t >= 0 and t <= 1:
            return (A[0] + (t * Bx_Ax), A[1] + (t * By_Ay))
        return None

    # add all the points
    for p in points:
        g.addVertex(p)
    for poly in polygons:
        for i in range(len(poly.points)):
            pt = poly.points[i]
            g.addVertex(Point("{0}_{1}".format(poly.name, i), pt))

    # add all edges between vertices that don't intersect a polygon edge
    vertices = g.graph.keys()
    for v1 in vertices:
        for v2 in vertices:
            if v1 == v2:
                continue
            if not intersects(v1, v2):
                g.addEdge(v1, v2, distance(v1, v2))

    return g

# a bunch of test data
a = Point('a', (0, 30))
b = Point('b', (110, 100))
x = Polygon('x', [(80, 90), (80, 70), (100, 70), (100, 90)])
y = Polygon('y', [(80, 20), (80, 0), (100, 0), (100, 20)])
z = Polygon('z', [(180, 60), (180, 40), (200, 40), (200, 60)])

vg = buildVisibilityGraph([a, b], [x, y, z])
# c = Point('c', (500, 500))
# vg.addVertex(c)
# vg.addEdge(b, c, 1000)
print "Built visibility graph: {0}".format(vg)
path, cost = vg.aStarSearch(a, b)
print "A* search results: {0} costs {1}".format(path, cost)
path, cost = vg.breadthFirstSearch(a, b)
print "BFS results: {0} costs {1}".format(path, cost)
path, cost = vg.depth_first_search(a, b)
print "DFS results: {0} costs {1}".format(path, cost)