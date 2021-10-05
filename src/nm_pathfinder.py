from math import inf, sqrt
from heapq import heappop, heappush

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

    if 'start' not in boxes or 'goal' not in boxes:
        print('No path!')
        return path, {}

    def bidirectional_astar(start, goal, graph, adj):
        forward_prev_paths = {start: None}
        backward_prev_paths = {goal: None}

        forward_pathcosts = {start: 0}
        backward_pathcosts = {goal: 0}

        queue = []
        heappush(queue, (0, start, 'destination'))  # maintain a priority queue of cells
        heappush(queue, (0, goal, 'start'))

        detail_points = {start: source_point, goal: destination_point}
        forward_whole_points = {start: source_point}
        backward_whole_points = {goal: destination_point}
        line_path = []

        while queue:
            priority, cell, curr_goal = heappop(queue)
            boxes[cell] = cell
            if (cell in backward_prev_paths) and (cell in forward_prev_paths):
                forward_curr = backward_curr = cell
                path_forward = []
                path_backward = [(forward_whole_points[cell], backward_whole_points[cell])]
                while forward_prev_paths[forward_curr] is not None:
                    path_forward.append((forward_whole_points[forward_prev_paths[forward_curr]], forward_whole_points[forward_curr]))
                    forward_curr = forward_prev_paths[forward_curr]
                path_forward.reverse()
                while backward_prev_paths[backward_curr] is not None:
                    path_backward.append((backward_whole_points[backward_prev_paths[backward_curr]], backward_whole_points[backward_curr]))
                    backward_curr = backward_prev_paths[backward_curr]
                    line_path = path_forward + path_backward
                return line_path
            #if (curr_goal == 'destination' and cell in backward_prev_paths) \
            #        or (curr_goal == 'start' and cell in forward_prev_paths):
            #     line_path.append(destination_point)
            #     while cell != start:
            #         dx = detail_points[cell][0]
            #         dy = detail_points[cell][1]
            #
            #         if curr_goal == 'destination':
            #             cell = forward_prev_paths[cell]
            #         else:
            #             cell = backward_prev_paths[cell]
            #
            #         if dx <= cell[0]: dx = cell[0]
            #         if dx >= cell[1]: dx = cell[1]
            #         if dy <= cell[2]: dy = cell[2]
            #         if dy >= cell[3]: dy = cell[3]
            #
            #         detail_points[cell] = (dx, dy)
            #         line_path.insert(0, detail_points[cell])
            #         #print(line_path)
            #     line_path.insert(0, source_point)
            #     return line_path
            # investigate children
            for child in adj[cell]:
                if curr_goal == 'destination':
                    cells = forward_whole_points
                    distance = forward_pathcosts
                    prev_path = forward_prev_paths
                    goal_point = destination_point
                else:
                    cells = backward_whole_points
                    distance = backward_pathcosts
                    prev_path = backward_prev_paths
                    goal_point = source_point

                dx, dy = cells[cell]
                if dx <= child[0]: dx = child[0]
                if dx >= child[1]: dx = child[1]
                if dy <= child[2]: dy = child[2]
                if dy >= child[3]: dy = child[3]

                cells[child] = (dx, dy)
                # calculate cost along this path to child
                cost_to_child = priority + transition_cost(cells[cell], cells[child], goal_point)
                if child not in distance or cost_to_child < distance[child]:
                    distance[child] = cost_to_child                          # update the cost
                    prev_path[child] = cell                                  # set the backpointer
                    heappush(queue, (cost_to_child, child, curr_goal))       # put the child on the priority queue

        print('No path!')
        return []

    def transition_cost(cell, cell2, final_cell):
        distance = euclidean_distance(cell, cell2)
        estimated_distance = euclidean_distance(cell, final_cell)
        #print(f'debug: distance: {distance}')
        #print(f'debug: estimated_distance: {estimated_distance}')
        return distance + estimated_distance

    def breadth_first_search (start, goal, graph, adj):
        queue = [start]
        prevs = {start: None}
        detail_points = {start: source_point, goal: destination_point}
        line_path = []
        while queue:
            current_node = queue.pop(0)
            boxes[current_node] = current_node
            if current_node == goal:
                line_path.append(destination_point)
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
                    line_path.insert(0, detail_points[current_node])
                    print(f'debug: inserted {detail_points[current_node]}')
                    print(line_path)
                line_path.insert(0, source_point)
                return line_path
            else:
                for new in adj[current_node]:
                    if new not in prevs:
                        prevs[new] = current_node
                        queue.append(new)

        print('No path!')
        return line_path

    path = bidirectional_astar(boxes['start'], boxes['goal'], mesh["boxes"], mesh["adj"])
    #path = breadth_first_search(boxes['start'], boxes['goal'], mesh["boxes"], mesh["adj"])

    return path, boxes.values()

def contains_point(box, point):
    # bx1 <= p.x && p.x <= bx2 && by1 <= p.y && p.y <= bx1
    return box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]

def euclidean_distance(point1, point2):
    # distance = √((px2 - px1)^2 + (py2 - py1)^2)
    return sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)