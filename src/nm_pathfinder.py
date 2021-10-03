import math

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

    def breadth_first_search (start, goal, graph, adj):
        queue = [start]
        prevs = {start: None}
        detail_points = {start: source_point, goal: destination_point}
        linePath = []
        while queue:
            current_node = queue.pop(0)
            if current_node == goal:
                linePath.append(destination_point)
                detail_points[goal] = destination_point
                print('Path!')
                while current_node != start:
                    boxes[current_node] = current_node
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

    path = breadth_first_search(boxes['start'], boxes['goal'], mesh["boxes"], mesh["adj"])
        
    return path, boxes.values()

def contains_point(box, point):
    # bx1 <= p.x && p.x <= bx2 && by1 <= p.y && p.y <= bx1
    return box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]

def euclidean_distance(point1, point2):
    # distance = √((px2 - px1)^2 + (py2 - py1)^2)
    return math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)