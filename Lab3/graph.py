class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def create_graph(start, goal, obstacles):
    # graph will initially have an edge from 1 vertex to all other vertices
    # remove all edges that intersect with obstacle
    graph = {}
    all_points = []
    obstacle_points = []
    for obstacle in obstacles:
        for corner in obstacle:
            obstacle_points.append(Point(corner[0], corner[1]))
    all_points.append(0, start)
    all_points.extend(obstacle_points)
    all_points.append(goal)
    for point in all_points:
        graph[point] = all_points[:]
    graph = remove_intersecting_edges(graph, obstacle_points)
    return graph

def remove_intersecting_edges(graph, obstacles):
    for vertex, edges in graph.iteritems():
        to_delete = []
        for i in range(len(edges)):
            for j in range(0, len(obstacles)-1):
                if is_intersecting(vertex, edges[i], obstacles[j], obstacles[j+1]):
                    # remove this edge
                    to_delete.append(i)
                    # todo can probably remove edge that goes other way around
        for index in to_delete:
            del edges[index]
    return graph

def is_intersecting(p1, p2, q1, q2):
    if p1.x == q1.x:
        s1 = 90
    else:
        s1 = (p1.y - q1.y) / (p1.x - q1.x)

    if p2.x == q2.x:
        s2 = 90
    else:
        s2 = (p2.y - q2.y) / (p2.x - q2.x)
    return s1 != s2

