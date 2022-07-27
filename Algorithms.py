from queue import LifoQueue, Queue, PriorityQueue
from geopy import distance, Nominatim
import requests
import json

geocoder = Nominatim(user_agent="Graphs")
distance = distance.distance

def BFS(goal, graph, start=0):
    current = graph.vertices[start]
    frontier = Queue()
    frontier.put_nowait(current)
    explored = list()
    
    if goal_test(current, goal):
        explored.append(current)
        return explored
    
    while not frontier.empty():
        current = frontier.get_nowait()
        if current not in explored:
            explored.append(current)
        for node in current.getNeighbours():
            if (node not in explored):
                if goal_test(node, goal):
                    explored.append(node)
                    return explored
                frontier.put_nowait(node)


def DFS(goal, graph, start=0):
    current = graph.vertices[start]
    frontier = LifoQueue()
    frontier.put_nowait(current)
    explored = list()

    if goal_test(current, goal):
        explored.append(current)
        return explored

    while not frontier.empty():
        current = frontier.get_nowait()
        if current not in explored:
            explored.append(current)

        for node in current.getNeighbours():
            if (node not in explored):
                if goal_test(node, goal):
                    explored.append(node)
                    return explored
                frontier.put_nowait(node)
                
                
                
def A_Star(graph, goal, start):
    goal_heuristic = heuristic(goal)
    frontier = PriorityQueue()
    came_from = dict()
    cost_so_far = dict()
    current = graph.vertices[start]
    came_from[current.value] = None
    cost_so_far[current.value] = 0
    frontier.put_nowait(current)
    while not frontier.empty():
        current = frontier.get_nowait()
        if is_tuple(current):
            current = current[-1]
            
        if goal_test(current, goal):
            return came_from, cost_so_far
            
        for next in current.getNeighbours():
            cost_to_next = cost(current)
            new_cost = cost_so_far[current.value] + cost_to_next(next)
            if next.value not in cost_so_far or new_cost < cost_so_far[next.value]:
                cost_so_far[next.value] = new_cost
                priority = new_cost + goal_heuristic(next)
                frontier.put_nowait((priority, next))
                came_from[next.value] = current.value
    return came_from, cost_so_far


# search helpers
def heuristic(goal):
    def calculate(node):
        start_town = "{}, Kenya".format(node)
        if start_town in distances:
            return distances[start_town]
        _, start_loc = geocoder.geocode(start_town)
        dist = distance(start_loc, end_loc).kilometers
        distances[start_town] = dist
        return dist
  
    distances = dict()

    end_town = "{}, Kenya".format(goal)
    _, end_loc = geocoder.geocode(end_town)
  
    return calculate
  

def cost(start_node):
    def calculate(end_node):
        end_town = "{}, Kenya".format(end_node)
        _, end_loc = geocoder.geocode(end_town)
        r=requests.get(f'http://router.project-osrm.org/route/v1/driving/{start_loc[1]},{start_loc[0]};{end_loc[1]},{end_loc[0]}?overview=false')
        dista=((json.loads(r.content))["routes"][0]["distance"])*0.00
        
        return dista
  
    start_town = "{}, Kenya".format(start_node)
    _, start_loc = geocoder.geocode(start_town)
  
    return calculate


def RP(came_from, start, goal):
    current = goal
    path = list()
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


def goal_test(vertex, goal):
    if is_tuple(vertex):
        vertex = vertex[-1]
    return vertex.value == goal.value

def is_tuple(var):
    return type(var) == tuple


