#!/usr/bin/env python3

# External dependencies
import pygame
import argparse
import math
import time
import colors
import os
from PIL import Image
from queue import PriorityQueue
from collections import deque

# Internal dependencies
from node import Node

WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
MAPS_DIR = "maps"
pygame.display.set_caption("Rover A* Pathfinding Tool")
pygame.init()
    
def h(n1, n2, heuristic):
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
    while current in came_from:
        if came_from[current] == start_pos:
            break  # Stop before setting the start node as path
        current = came_from[current]
        current.set_path()
        draw()

def bfs_precheck(start, end):
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

def toggle_search_area(grid):
        for row in grid:
            for node in row:
                if node.node_type == "traversed":
                    if node.color == colors.GREEN or node.color == colors.RED:
                        node.prev_color = node.color
                        if node.extra_cost == 0:
                            node.color = colors.WHITE
                        elif node.extra_cost == 1:
                            node.color = colors.FIVESPLIT_1
                        elif node.extra_cost == 2:
                            node.color = colors.FIVESPLIT_2
                        elif node.extra_cost == 3:
                            node.color = colors.FIVESPLIT_3
                        elif node.extra_cost == 4:
                            node.color = colors.FIVESPLIT_4
                    else:
                        node.color = node.prev_color

def algorithm(draw, grid, start_pos, end_pos, heurisitc):
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
            reconstruct_path(came_from, end_pos, draw, start_pos)
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

            return True
        
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
    return False

def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid

def draw_gridlines(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, colors.GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, colors.GREY, (j * gap, 0), (j * gap, width))

def draw(win, grid, rows, width):
    win.fill(colors.WHITE)
    for row in grid:
        for node in row:
            node.draw(win)

    draw_gridlines(win, rows, width)

    # Draw coordinates of hovered node
    mouse_x, mouse_y = pygame.mouse.get_pos()
    if 0 <= mouse_x < width and 0 <= mouse_y < width:
        row, col = get_clicked_pos((mouse_x, mouse_y), rows, width)
        coord_text = f"({row}, {col})"
        font = pygame.font.SysFont(None, 24)
        text_surf = font.render(coord_text, True, (0, 0, 0))

        # Draw at different offset based on mouse x/y to prevent it rendering outside of the window
        x_offset = -50 if row >= rows / 2 else 15
        y_offset = -20 if col >= rows / 2 else 10
        text_rect = text_surf.get_rect(topleft=(mouse_x + x_offset, mouse_y + y_offset))
        win.blit(text_surf, text_rect)
    
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col

def load_map(grid, map_path, map_img):
    map_pixels = Image.open(map_path).load()
    for y in range(map_img.height):
        for x in range(map_img.width):
            if map_pixels[x,y] == colors.BLACK:
                grid[x][y].set_barrier() 
            elif map_pixels[x,y] == colors.FIVESPLIT_4:
                grid[x][y].set_fivesplit4()
            elif map_pixels[x,y] == colors.FIVESPLIT_3:
                grid[x][y].set_fivesplit3()
            elif map_pixels[x,y] == colors.FIVESPLIT_2:
                grid[x][y].set_fivesplit2()
            elif map_pixels[x,y] == colors.FIVESPLIT_1:
                grid[x][y].set_fivesplit1()

def main(win, width):
    parser = argparse.ArgumentParser()
    parser.add_argument("heuristic", type=str, choices=["manhattan", "euclidean", "octile"], help="Choose the heuristic function.")
    parser.add_argument("--size", type=int, default=50, help="Grid size. Grid is square, so 'size' value will apply to height AND width of the grid.")
    parser.add_argument("--use_map", type=str, help="Choose an image to use for a predefined map. Image dimensions required to match grid size. Overrides --size.")
    parser.add_argument("--path_only", type=int, nargs="+", help="Enter two points in the form [X1 Y1 X2 Y2]. Will only display the final path.")
    parser.add_argument("--dynamic", type=str, help="Load a higher-resolution map that updates the path after each move. Only use with --use_map.")
    parser.add_argument("-p", "--precheck", action="store_true", help="Run a BFS precheck to confirm that the start node can reach the end node")
    args = parser.parse_args()

    start_pos = None
    end_pos = None

    if args.use_map:
        # Load map if path exists
        map_path = os.path.join(MAPS_DIR, args.use_map)
        if not os.path.exists(map_path):
            print(f"ERR: Could not find the map image at: {args.use_map}")
            quit()
        map_img = Image.open(map_path)
        args.size = map_img.width

    if args.dynamic:
        # Check if trying to use dynamic mode without using use_map
        if not args.use_map:
            print(f"ERR: Dynamic mode should only be used when loading a map using --use_map.")
            quit()
        # Load dynamic map if path exists
        dyn_map_path = os.path.join(MAPS_DIR, args.dynamic)
        if not os.path.exists(dyn_map_path):
            print(f"ERR: Could not find the dynamic map image at: {args.dynamic}")
            quit()
        dyn_map_img = Image.open(dyn_map_path)
        dyn_map_size = dyn_map_img.width

    if args.path_only:
        if len(args.path_only) != 4:
            print("ERR: Invalid number of supplied values. Supplied points for --path_only should be in form [X1 Y1 X2 Y2].")
            quit()
        for coord in args.path_only:
            if coord >= args.size or coord < 0:
                print("ERR: Invalid coord in --path_only. Coord value must be between 0 and the map size.")
                quit()

    grid = make_grid(args.size, width)

    if args.use_map:
        load_map(grid, map_path, map_img)

    if args.path_only:
        x1, y1, x2, y2 = args.path_only
        start_pos = grid[x1][y1]
        start_pos.set_start()
        end_pos = grid[x2][y2]
        end_pos.set_end()

        # Update neighbors first
        for row in grid:
            for node in row:
                node.update_neighbors(grid, args.heuristic)

        if args.precheck:
            bfs_precheck(start_pos, end_pos)

        # Run algorithm with dummy draw function
        algorithm(lambda: None, grid, start_pos, end_pos, args.heuristic)

        # Now show window and draw final state
        global WIN
        WIN = pygame.display.set_mode((width, width))
        draw(WIN, grid, args.size, width)

    run = True
    while run:
        draw(WIN, grid, args.size, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, args.size, width)
                node = grid[row][col]
                if not start_pos and node != end_pos:
                    start_pos = node
                    start_pos.set_start()
                elif not end_pos and node != start_pos:
                    end_pos = node
                    end_pos.set_end()
                elif node != start_pos and node != end_pos:
                    node.set_barrier()

            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, args.size, width)
                node = grid[row][col]
                node.reset()
                if node == start_pos:
                    start_pos = None
                if node == end_pos:
                    end_pos = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start_pos and end_pos:
                    # Update neighbors for each node
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid, args.heuristic)

                    if args.precheck:
                        # Quick BFS check if the end node is reachable from the start node
                        bfs_precheck(start_pos, end_pos)

                    algorithm(lambda: draw(WIN, grid, args.size, width), grid, start_pos, end_pos, args.heuristic)

                if event.key == pygame.K_c:
                    start_pos = None
                    end_pos = None
                    grid = make_grid(args.size, width)

                if event.key == pygame.K_r:
                    start_pos = None
                    end_pos = None
                    grid = make_grid(args.size, width)
                    load_map(grid, map_path, map_img)

                if event.key == pygame.K_t:
                    toggle_search_area(grid)

    pygame.quit()

main(WIN, WIDTH)