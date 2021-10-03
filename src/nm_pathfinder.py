import math
from math import inf, sqrt

def find_path (source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = {}

    for box in mesh["boxes"]:
        if contains_point(box, source_point):
            boxes['start'] = box
            #print(f'debug: start - {box}')
        if contains_point(box, destination_point):
            boxes['goal'] = box
            #print(f'debug: goal - {box}')

    if boxes['start'] is None or boxes['goal'] is None:
        print('No path!')
        return path, boxes.values()

    def dijkstras_forward_search (start, goal, graph, adj):
        paths = {start: []}
        pathcosts = {start: 0}
        queue = []
        queue.append((0, start))
        detail_points = {start: source_point, goal: destination_point}
        whole_points = {start: source_point, goal: destination_point}
        linePath = []

        while queue:
            #print(f'debug: here is queue(0): {queue[0]}')
            priority, cell = queue.pop(0)
            #print(f'debug: here is cell: {cell}')
            boxes[cell] = cell
            if cell == goal:
                linePath.append(destination_point)
                while cell != start:
                #    returnPath.append((current_node[0], current_node[1], current_node[2], current_node[3]))

                    dx = detail_points[cell][0]
                    dy = detail_points[cell][1]
                    cell = paths[cell]

                    if dx <= cell[0]: dx = cell[0]
                    if dx >= cell[1]: dx = cell[1]
                    if dy <= cell[2]: dy = cell[2]
                    if dy >= cell[3]: dy = cell[3]

                    detail_points[cell] = (dx, dy)
                    linePath.insert(0, detail_points[cell])
                    #print(f'debug: inserted {detail_points[cell]}')
                    #print(linePath)
                linePath.insert(0, source_point)
                return linePath
            # investigate children
            #print(f'debug: here is adj for {cell}: {adj[cell]}')
            for child in adj[cell]:
                dx = whole_points[cell][0]
                dy = whole_points[cell][1]

                if dx <= child[0]: dx = child[0]
                if dx >= child[1]: dx = child[1]
                if dy <= child[2]: dy = child[2]
                if dy >= child[3]: dy = child[3]

                whole_points[child] = (dx, dy)
                # calculate cost along this path to child
                cost_to_child = priority + transition_cost(whole_points, cell, child)
                if child not in pathcosts or cost_to_child < pathcosts[child]:
                    pathcosts[child] = cost_to_child            # update the cost
                    paths[child] = cell                         # set the backpointer
                    queue.append((cost_to_child, child))     # put the child on the priority queue

        return False

    def transition_cost(points, cell, cell2):
        x1 = points[cell][0]
        y1 = points[cell][1]
        x2 = points[cell2][0]
        y2 = points[cell2][1]
        distance = sqrt((x1 - x2)**2 + (y1 - y2)**2)
        estimated_distance = sqrt((x1 - destination_point[0])**2 + (y1 - destination_point[1])**2)
        #average_cost = (level['spaces'][cell] + level['spaces'][cell2])/2
        print(f'debug: distance: {distance}')
        print(f'debug: estimated_distance: {estimated_distance}')
        return distance + estimated_distance

    def breadth_first_search (start, goal, graph, adj):
        queue = [start]
        prevs = {start: None}
        detail_points = {start: source_point, goal: destination_point}
        linePath = []
        while queue:
            current_node = queue.pop(0)
            boxes[current_node] = current_node
            if current_node == goal:
                linePath.append(destination_point)
                detail_points[goal] = destination_point
                print('Path!')
                while current_node != start:
                #    returnPath.append((current_node[0], current_node[1], current_node[2], current_node[3]))

                    dx = detail_points[current_node][0]
                    dy = detail_points[current_node][1]
                    current_node = prevs[current_node]

                    if dx <= current_node[0]: dx = current_node[0]
                    if dx >= current_node[1]: dx = current_node[1]
                    if dy <= current_node[2]: dy = current_node[2]
                    if dy >= current_node[3]: dy = current_node[3]

                    detail_points[current_node] = (dx, dy)
                    linePath.insert(0, detail_points[current_node])
                    print(f'debug: inserted {detail_points[current_node]}')
                    print(linePath)
                linePath.insert(0, source_point)
                return linePath
            else:
                for new in adj[current_node]:
                    if new not in prevs:
                        prevs[new] = current_node
                        queue.append(new)

        print('No path!')
        return linePath

    path = dijkstras_forward_search(boxes['start'], boxes['goal'], mesh["boxes"], mesh["adj"])
    #path = breadth_first_search(boxes['start'], boxes['goal'], mesh["boxes"], mesh["adj"])

    return path, boxes.values()

def contains_point(box, point):
    # bx1 <= p.x && p.x <= bx2 && by1 <= p.y && p.y <= bx1
    return box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]

def euclidean_distance(point1, point2):
    # distance = √((px2 - px1)^2 + (py2 - py1)^2)
    return math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)