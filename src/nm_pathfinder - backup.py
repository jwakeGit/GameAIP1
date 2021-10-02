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
        if box[0] <= source_point[0] and box[1] >= source_point[0] and box[2] <= source_point[1] and box[3] >= source_point[1]:
            boxes['start'] = box
            print(f'debug: start - {box}')
        if box[0] <= destination_point[0] and box[1] >= destination_point[0] and box[2] <= destination_point[1] and box[3] >= destination_point[1]:
            boxes['goal'] = box
            print(f'debug: goal - {box}')
    def breadth_first_search (start, goal, graph, adj):
        count = 0
        print('debug 1')
        queue = [start]
        prevs = {start: None}
        detail_points = {start: source_point}
        linePath = [destination_point]
        while queue:
            print('debug 2')
            current_node = queue.pop(0)
            if current_node == goal:
                linePath.append(destination_point)
                print('debug 4')
                print('Path!')
                #returnPath = []
                while current_node != start:
                    boxes[current_node] = count
                    count += 1
                #    returnPath.append((current_node[0], current_node[1], current_node[2], current_node[3]))
                    current_node = prevs[current_node]
                    linePath.insert(0, detail_points[current_node])
                    print(f'debug: inserted {detail_points[current_node]}')
                #returnPath.append(start)
                return linePath
                #return returnPath
            else:
                print('debug 3')
                for new in adj[current_node]:
                    if new not in prevs:
                        prevs[new] = current_node
                        queue.append(new)
                        #detail point code:
                        curPoint = detail_points[current_node]
                        px = curPoint[0]
                        py = curPoint[1]
                        b1x1 = current_node[0]
                        b1x2 = current_node[1]
                        b1y1 = current_node[2]
                        b1y2 = current_node[3]
                        if b1x1 < new[0]: b1x1 = new[0]
                        if b1x2 > new[1]: b1x2 = new[1]
                        if b1y1 < new[2]: b1y1 = new[2]
                        if b1y2 > new[3]: b1y2 = new[3]
                        dx = max(b1x1 - px, 0, px - b1x2)
                        dy = max(b1y1 - py, 0, py - b1y2)
                        detail_points[new] = (dx, dy)
                        #these lines are excessive
                        #linePath.append((dx, dy))
                        #print(f'new point: {dx}, {dy}')

        print('No path!')
        return linePath
        #returnPath = []
        #return returnPath

    path = breadth_first_search(boxes['start'], boxes['goal'], mesh["boxes"], mesh["adj"])
        
    print(boxes.keys())
    return path, boxes.keys()
