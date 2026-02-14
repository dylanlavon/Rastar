"""
Core pathfinding logic and heuristic utilities for the grid-based A* visualizer.

This module implements:
- Heuristic distance functions (Manhattan, Euclidean, Octile)
- A breadth-first search pre-check to verify path reachability
- The A* search algorithm with support for diagonal movement and weighted terrain
- Path reconstruction and real-time visualization hooks

The algorithm is designed for interactive use with Pygame, including live rendering,
event handling, and diagnostic checks for heuristic admissibility and consistency.
No rendering or grid construction is performed here beyond algorithm-driven updates.
"""

# External dependencies
import time
import math
import pygame
from queue import PriorityQueue
from collections import deque


def h(n1, n2, heuristic):
    """
    Get the heuristic estimate from one node to another.
    
    :param n1: X-Y coordinates of the first node
    :param n2: X-Y coordinates of the second node
    :param heuristic: Heuristic being used (either 'manhattan', 'euclidean', or 'octile')
    """
    x1, y1 = n1
    x2, y2 = n2
    dx, dy = abs(x1 - x2), abs(y1 - y2)

    if heuristic.lower() == "manhattan":
        return dx + dy
    elif heuristic.lower() == "euclidean":
        return math.sqrt(dx ** 2 + dy ** 2)
    elif heuristic.lower() == "octile":
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

def reconstruct_path(came_from, current, draw, start_pos):
    """
    Starting from the goal node, this function walks backward through the
    `came_from` mapping (which is populated in the algorithm() function)
    until reaching the start node.

    Each intermediate node is marked as part of the final path, the display 
    is updated after each step, and the nodes are compiled into a list.
    
    :param came_from: Dict mapping each node to the node it was reach from during the search
    :param current: The goal node from which path reconstruction begins
    :param draw: Callback function used to redraw the grid after each update
    :param start_pos: The starting node for this path; not marked as part of the path
    :return: A sequential list of nodes representing the final path
    """
    path = [current]
    while current in came_from:
        if came_from[current] == start_pos:
            break  # Stop before visually setting the start node to purple
        current = came_from[current]
        current.set_path() # Visually color the node purple
        path.append(current)
        draw()

    path.append(start_pos)
        
    # Reverse the list so it flows from start -> end
    path.reverse()
    return path

def bfs_precheck(start, end):
    """
    Timed breadth-first search to be run before proper A* pathfinding takes place.

    Used to determine if a path is reachable at all before finding the optimal path.
    
    :param start: Starting node of the path
    :param end: Destination node of the path
    """
    bfs_start_time = time.time()    
    visited = set()
    queue = deque([start])
    
    is_reachable = False
    while queue:
        current = queue.popleft()
        if current == end:
            is_reachable = True
            break
        for neighbor in current.neighbors:
            if not neighbor.is_barrier() and neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)

    bfs_end_time = time.time()
    bfs_elapsed_time = bfs_end_time - bfs_start_time
    if not is_reachable:
        print(f"\nEnd node [{end.row}, {end.col}] is unreachable from start node [{start.row}, {start.col}].")   
        print(f"BFS pre-check took {bfs_elapsed_time:.4f} seconds to confirm.")
        quit()
    print(f"\nA path exists from start node [{start.row}, {start.col}] to end node [{end.row}, {end.col}].")   
    print(f"BFS pre-check took {bfs_elapsed_time:.4f} seconds to confirm.")

def algorithm(draw, grid, start_pos, end_pos, heurisitc):
    """
    Executes the A* pathfinding algorithm on a grid with optional diagonal movement
    and weighted terrain.

    The function searches for the lowest-cost path from `start_pos` to `end_pos`
    using a priority queue ordered by f = g + h. It supports multiple heuristics,
    optional diagonal movement, and per-node extra traversal costs. While running,
    the grid is continuously redrawn to visualize open, closed, and final path nodes.

    The function also performs runtime checks for heuristic admissibility and
    consistency and reports execution statistics upon completion.
    
    :param draw: Callback function that redraws the grid for visualization
    :param grid: The nxn matrix of nodes
    :param start_pos: Starting node for the path search
    :param end_pos: Goal node for the path search
    :param heurisitc: Heuristic being used (either 'manhattan', 'euclidean', or 'octile')
    :return: List of path nodes if successful, None otherwise
    """
    start_time = time.time() # Start timer
    count = 0
    nodes_explored = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start_pos)) # Start with the start node in the open set
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row} # Keeps track of the current shortest distance from start node to this node
    g_score[start_pos] = 0
    f_score = {node: float("inf") for row in grid for node in row} # Keeps track of the predicted distance from this node to the end node
    f_score[start_pos] = h(start_pos.get_pos(), end_pos.get_pos(), heurisitc)

    open_set_hash = {start_pos}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        
        current = open_set.get()[2]
        open_set_hash.remove(current)
        nodes_explored += 1

        if current == end_pos:
            # Reconstruct the path and capture the returned list
            final_path = reconstruct_path(came_from, end_pos, draw, start_pos)
            end_pos.set_end()
            end_time = time.time()
            print(f"\nPath successfully found.\nExecution time: {end_time - start_time:.4f} seconds\nTotal path cost: {g_score[end_pos]:.4f}\nTotal spaces explored: {nodes_explored}")

            # Admissibility check: heuristic(start) should not be greater than actual cost
            true_cost = g_score[end_pos]
            h_start = h(start_pos.get_pos(), end_pos.get_pos(), heurisitc)

            if h_start > true_cost + 1e-5:
                print(f"NOT ADMISSIBLE! Heuristic from start ({start_pos.get_pos()}) "
                    f"overestimates cost. h(start): {h_start:.4f}, true_cost: {true_cost:.4f}")
            else:
                print(f"Heuristic appears ADMISSIBLE. h(start): {h_start:.4f}, true_cost: {true_cost:.4f}")

            # Return the list of waypoints so main.py can use them for the dynamic pass
            return final_path
        
        for neighbor in current.neighbors:
            # Determine g_score based on cardinal/diagonal
            if heurisitc.lower() == "manhattan": 
                move_cost = 1
            else: 
                move_cost = 1 if abs(neighbor.row - current.row) + abs(neighbor.col - current.col) == 1 else math.sqrt(2)
            
            # Make sure to add extra edge weights based on node color
            temp_g_score = g_score[current] + move_cost + neighbor.extra_cost

            # Get heuristic values
            h_current = h(current.get_pos(), end_pos.get_pos(), heurisitc)
            h_neighbor = h(neighbor.get_pos(), end_pos.get_pos(), heurisitc)

            # Consistency check: h(current) <= move_cost + h(neighbor)
            if h_current > move_cost + h_neighbor + 1e-5:  # Add epsilon for float precision
                print(f"INCONSISTENT! At node {current.get_pos()} to {neighbor.get_pos()}: "
                    f"h(current): {h_current:.4f}, move_cost: {move_cost:.4f}, "
                    f"h(neighbor): {h_neighbor:.4f}")

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end_pos.get_pos(), heurisitc)

                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.set_open()
        draw()

        if current != start_pos:
            current.set_closed()

    end_time = time.time()
    print(f"\nUnable to find a path.\nExecution time: {end_time - start_time:.4f} seconds\nTotal spaces explored: {nodes_explored}")
    return None