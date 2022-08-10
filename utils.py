from queue import Empty, PriorityQueue, Queue
from geopy.geocoders import Nominatim
from geopy.distance import geodesic
from geopy.exc import GeocoderTimedOut


TOWNS = ('Nairobi', 'Mombasa', 'Kisumu',
         'Eldoret', 'Kitui', 'Makueni', 'Nakuru', 'Nanyuki', 'Lamu', 'Garissa')


def getCoordForList(cities):
    _dict = dict()
    for city in cities:
        _dict[city] = getCoord(city)

    return _dict


def getCoord(loc):
    return recurrCoord(loc, attempt=1)


def recurrCoord(loc, attempt):
    try:
        geolocator = Nominatim(user_agent='assignment')
        location = geolocator.geocode(loc)
        return (location.latitude, location.longitude)
    except GeocoderTimedOut:
        if attempt <= 5:
            return recurrCoord(loc, attempt+1)
        raise


def getDistance(pt1, pt2):
    return geodesic(getCoord(pt1), getCoord(pt2)).km


def getPathDistance(list):
    total = 0
    for i in range(len(list)-1):
        dist = getDistance(list[i], list[i+1])
        total = total + dist

    return int(total)


def getPath(graph, start, goal, algorithm):
    path = []
    if algorithm == 'Depth First Search':
        path = dfsTest(graph, start, goal)
    elif algorithm == 'Breadth First Search':
        path = bfsTest(graph, start, goal)
    else:
        path = aStarTest(graph, start, goal)

    return path


def getCoordPath(townDict, path):
    res = []
    for i in path:
        res.append(townDict[i])
    return res


def clear(q):
    while not q.empty():
        try:
            q.get(False)
        except Empty:
            continue
        q.task_done()


def dfsTest(graph, startNode, goalNode):
    stack = [startNode]
    visited = set()

    res = []

    while len(stack) != 0:
        curr = stack.pop()

        if curr not in visited:
            res.append(curr)
            visited.add(curr)

            if curr == goalNode:
                break

        for neighbor in graph.getNodes()[curr].getEdges():
            nn = neighbor.getConnTo().getId()
            if nn not in visited:
                stack.append(nn)
    return res


def bfsTest(graph, startNode, goalNode):
    visited = set()
    queue = Queue()
    path = []

    queue.put(startNode)
    visited.add(startNode)

    if startNode == goalNode:
        path.append(queue.get())

    while not queue.empty():
        curr = queue.get()
        path.append(curr)

        for neighbor in graph.getNodes()[curr].getEdges():
            nn = neighbor.getConnTo().getId()
            if nn not in visited:
                visited.add(nn)
                if nn == goalNode:
                    clear(queue)
                    path.append(nn)
                    break
                queue.put(nn)

    return path


def aStarTest(graph, start, goal):
    priorityQ = PriorityQueue()
    visited = set()
    path = []

    priorityQ.put((0, start))

    while not priorityQ.empty():
        curr = priorityQ.get()[-1]
        visited.add(curr)
        path.append(curr)

        if curr == goal:
            clear(priorityQ)
            break

        neighbours = graph.getNodes()[curr].getEdges()
        for node in neighbours:
            nn = node.getConnTo().getId()

            g = cost(start, nn)
            h = heuristic(nn, goal)
            f = g+h

            if nn == goal:
                path.append(nn)
                clear(priorityQ)
                break
            if nn not in visited:
                priorityQ.put((f, nn))

    return path


def heuristic(curr, goal):
    return getDistance(goal, curr)


def cost(start, goal):
    return getDistance(goal, start)
