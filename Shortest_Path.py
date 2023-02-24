import heapq
import itertools

def heuristic(node, end):
    return ((node[0] - end[0]) ** 2 + (node[1] - end[1]) ** 2 + (node[2] - end[2]) ** 2) ** 0.5

def a_star(start, end, points):
    queue = [(heuristic(start, end), 0, 0, start, [])]
    visited = set()
    while queue:
        _, g, passed, curr, path = heapq.heappop(queue)
        if passed == len(points) and curr == end:
            return path + [curr]
        if (curr, passed) in visited:
            continue
        visited.add((curr, passed))
        for point in points:
            if point not in path and passed < len(points):
                point_h = heuristic(curr, point)
                heapq.heappush(queue, (g + point_h, g + point_h, passed + 1, point, path + [curr]))
        end_h = heuristic(curr, end)
        heapq.heappush(queue, (g + end_h, g + end_h, passed, end, path + [curr]))
    return None

# Define the start and end points
start = (0, 0, 0)
end = (10, 10, 10)

# Define the list of points to be visited
points = [(2, 1, 3), (4, 2, 5), (6, 3, 7), (8, 4, 9)]

# Call the a_star function to find the optimal path
path = a_star(start, end, points)

# Print the path
if path is None:
    print("No path found.")
else:
    print("Optimal path:", path)