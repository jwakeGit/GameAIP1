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
        print(f'source point: {source_point}')
        #print('debug 1')
        queue = [start]
        prevs = {start: None}
        detail_points = {start: source_point, goal: destination_point}
        linePath = []
        while queue:
            #print('debug 2')
            current_node = queue.pop(0)
            if current_node == goal:
                linePath.append(destination_point)
                detail_points[goal] = destination_point
                print(f'destination point: {destination_point}')
                #print('debug 4')
                print('Path!')
                #returnPath = []
                while current_node != start:
                    boxes[current_node] = current_node
                #    returnPath.append((current_node[0], current_node[1], current_node[2], current_node[3]))
                    clone_node = current_node
                    current_node = prevs[current_node]
                    current_point = detail_points[clone_node]
                    px = current_point[0]
                    py = current_point[1]
                    b1x1 = clone_node[0]
                    b1x2 = clone_node[1]
                    b1y1 = clone_node[2]
                    b1y2 = clone_node[3]
                    if b1x1 <= current_node[0]: dx = current_node[0]
                    if b1x2 >= current_node[1]: dx = current_node[1]
                    if b1y1 <= current_node[2]: dy = current_node[2]
                    if b1y2 >= current_node[3]: dy = current_node[3]
                    #dx = max(b1x1 - px, 0, px - b1x2)
                    #dy = max(b1y1 - py, 0, py - b1y2)
                    detail_points[current_node] = (dx, dy)
                    linePath.insert(0, detail_points[current_node])
                    print(f'debug: inserted {detail_points[current_node]}')
                    print(linePath)
                #returnPath.append(start)
                linePath.insert(0, source_point)
                return linePath
                #return returnPath
            else:
                #print('debug 3')
                for new in adj[current_node]:
                    if new not in prevs:
                        prevs[new] = current_node
                        queue.append(new)
                        #detail point code has been moved away

        print('No path!')
        return linePath
        #returnPath = []
        #return returnPath

    path = breadth_first_search(boxes['start'], boxes['goal'], mesh["boxes"], mesh["adj"])
        
    #print(boxes.values())
    #this used to return boxes.keys() so keep that in mind
    return path, boxes.values()

def contains_point(box, point):
    # bx1 <= p.x && p.x <= bx2 && by1 <= p.y && p.y <= bx1
    return box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]

def euclidean_distance(point1, point2):
    # distance = √((px2 - px1)^2 + (py2 - py1)^2)
    return math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)