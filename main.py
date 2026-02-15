#!/usr/bin/env python3
"""
Entry point for the Rover A* Pathfinding Tool.
"""

import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import argparse
from PIL import Image

import grid_manager
import algo

WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
MAPS_DIR = "maps"
pygame.display.set_caption("Rover A* Pathfinding Tool")
pygame.init()

def execute_dynamic_pathfinding(coarse_grid, dyn_grid, args):
    print("Starting Dynamic Hierarchical Pathfinding...")
    
    for row in coarse_grid.grid:
        for node in row:
            node.update_neighbors(coarse_grid.grid, args.heuristic)
    
    if args.precheck:
        algo.bfs_precheck(coarse_grid.start_pos, coarse_grid.end_pos)
        
    coarse_path = algo.algorithm(lambda: None, coarse_grid.grid, coarse_grid.start_pos, coarse_grid.end_pos, args.heuristic)
    
    if not coarse_path:
        print("ERR: No valid path found on the coarse grid.")
        return

    # Extract coordinates and attach to the Grid object
    gps_guideline = []
    for c_node in coarse_path:
        center_x = c_node.x + (c_node.width // 2)
        center_y = c_node.y + (c_node.width // 2)
        gps_guideline.append((center_x, center_y))
        
    dyn_grid.global_route = gps_guideline

    scale = len(dyn_grid.grid) / len(coarse_grid.grid)
    
    c_start = coarse_path[0]
    rx = min(int(c_start.row * scale + scale / 2), len(dyn_grid.grid) - 1)
    ry = min(int(c_start.col * scale + scale / 2), len(dyn_grid.grid) - 1)
    
    rover_node = dyn_grid.grid[rx][ry]
    dyn_grid.start_pos = rover_node
    rover_node.set_start()
    
    c_end = coarse_path[-1]
    ex = min(int(c_end.row * scale + scale / 2), len(dyn_grid.grid) - 1)
    ey = min(int(c_end.col * scale + scale / 2), len(dyn_grid.grid) - 1)
    global_end_node = dyn_grid.grid[ex][ey]

    historical_path = []

    for waypoint in coarse_path[1:]:
        # Allow toggling G inside the simulation pauses
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_g:
                    dyn_grid.toggle_guideline()
                    dyn_grid.draw()

        min_x = int(waypoint.row * scale)
        max_x = int((waypoint.row + 1) * scale) - 1
        min_y = int(waypoint.col * scale)
        max_y = int((waypoint.col + 1) * scale) - 1
        
        target_x = max(min_x, min(rover_node.row, max_x))
        target_y = max(min_y, min(rover_node.col, max_y))
        target_node = dyn_grid.grid[target_x][target_y]
        
        if rover_node == target_node: 
            continue
        
        target_node.set_end()
        
        for row in dyn_grid.grid:
            for node in row:
                node.update_neighbors(dyn_grid.grid, args.heuristic)
                
        dyn_grid.sensor_sweep(rover_node, radius=args.sensor)
                
        local_path = algo.algorithm(lambda: dyn_grid.draw(), dyn_grid.grid, rover_node, target_node, args.heuristic)
        
        if not local_path or len(local_path) < 2:
            print(f"ERR: Rover got stuck at [{rover_node.row}, {rover_node.col}]! Obstacles blocked the way.")
            return
        
        for step_node in local_path[1:]:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
                # Catch K_g during the driving animation!
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_g:
                        dyn_grid.toggle_guideline()
            
            historical_path.append(rover_node)
            rover_node.set_path()     
            rover_node = step_node
            rover_node.set_start()    
            dyn_grid.draw()
            pygame.time.delay(10)     
        
        for row in dyn_grid.grid:
            for node in row:
                if node.is_open_node or node.is_closed_node:
                    node.is_open_node = False
                    node.is_closed_node = False
                    node.update_color(show_search=dyn_grid.show_search)
        
        for node in historical_path:
            node.set_path()
        
        global_end_node.set_end()
        dyn_grid.draw()
            
    print("Destination reached on the dynamic map!")

    for row in dyn_grid.grid:
        for node in row:
            if node.is_open_node or node.is_closed_node:
                node.is_open_node = False
                node.is_closed_node = False
                node.update_color(show_search=dyn_grid.show_search)
                
    for node in historical_path:
        node.set_path()
        
    rover_node.set_start()
    global_end_node.set_end()
    dyn_grid.draw()

def main(win, width):
    parser = argparse.ArgumentParser()
    parser.add_argument("heuristic", type=str, choices=["manhattan", "euclidean", "octile"], help="Choose the heuristic function.")
    parser.add_argument("--size", type=int, default=50, help="Grid size. Grid is square, so 'size' value will apply to height AND width of the grid.")
    parser.add_argument("--use_map", type=str, help="Choose an image to use for a predefined map. Image dimensions required to match grid size. Overrides --size.")
    parser.add_argument("--path_only", type=int, nargs="+", help="Enter two points in the form [X1 Y1 X2 Y2]. Will only display the final path.")
    parser.add_argument("--dynamic", type=str, help="Load a higher-resolution map that updates the path after each move. Only use with --use_map.")
    parser.add_argument("--sensor", type=int, default=1, help="Radius of the rover's sensor sweep (default: 1).")
    parser.add_argument("-p", "--precheck", action="store_true", help="Run a BFS precheck to confirm that the start node can reach the end node")
    args = parser.parse_args()

    map_img = None  
    dyn_map_img = None
    dyn_map_size = None

    if args.use_map:
        map_path = os.path.join(MAPS_DIR, args.use_map)
        if not os.path.exists(map_path):
            print(f"ERR: Could not find the map image at: {args.use_map}")
            quit()
        map_img = Image.open(map_path)
        args.size = map_img.width

    if args.dynamic:
        if not args.use_map:
            print(f"ERR: Dynamic mode should only be used when loading a map using --use_map.")
            quit()
        dyn_map_path = os.path.join(MAPS_DIR, args.dynamic)
        if not os.path.exists(dyn_map_path):
            print(f"ERR: Could not find the dynamic map image at: {args.dynamic}")
            quit()
        dyn_map_img = Image.open(dyn_map_path)
        dyn_map_size = dyn_map_img.width

    if args.path_only:
        if len(args.path_only) != 4:
            print("ERR: Invalid number of supplied values.")
            quit()
        for coord in args.path_only:
            if coord >= args.size or coord < 0:
                print("ERR: Invalid coord in --path_only.")
                quit()

    coarse_grid = grid_manager.Grid(args.size, width, win, map_img)
    coarse_grid.start_pos = None
    coarse_grid.end_pos = None
    if args.use_map:
        coarse_grid.load_map()

    dyn_grid = None
    if args.dynamic:
        dyn_grid = grid_manager.Grid(dyn_map_size, width, win, dyn_map_img)
        dyn_grid.start_pos = None
        dyn_grid.end_pos = None
        
    active_grid = coarse_grid
    grid = active_grid.grid

    if args.path_only:
        x1, y1, x2, y2 = args.path_only
        coarse_grid.start_pos = coarse_grid.grid[x1][y1]
        coarse_grid.start_pos.set_start()
        coarse_grid.end_pos = coarse_grid.grid[x2][y2]
        coarse_grid.end_pos.set_end()

        if dyn_grid:
            active_grid = dyn_grid
            global WIN
            WIN = pygame.display.set_mode((width, width))
            active_grid.win = WIN
            active_grid.draw()
            execute_dynamic_pathfinding(coarse_grid, dyn_grid, args)
        else:
            for row in coarse_grid.grid:
                for node in row:
                    node.update_neighbors(coarse_grid.grid, args.heuristic)

            if args.precheck:
                algo.bfs_precheck(coarse_grid.start_pos, coarse_grid.end_pos)

            algo.algorithm(lambda: None, coarse_grid.grid, coarse_grid.start_pos, coarse_grid.end_pos, args.heuristic)
            WIN = pygame.display.set_mode((width, width))
            coarse_grid.win = WIN
            active_grid.draw()

    run = True
    while run:
        active_grid.draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = active_grid.get_clicked_pos(pos)
                node = grid[row][col]
                if not active_grid.start_pos and node != active_grid.end_pos:
                    active_grid.start_pos = node
                    active_grid.start_pos.set_start()
                elif not active_grid.end_pos and node != active_grid.start_pos:
                    active_grid.end_pos = node
                    active_grid.end_pos.set_end()
                elif node != active_grid.start_pos and node != active_grid.end_pos:
                    node.set_barrier()

            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = active_grid.get_clicked_pos(pos)
                node = grid[row][col]
                node.reset()
                if node == active_grid.start_pos:
                    active_grid.start_pos = None
                if node == active_grid.end_pos:
                    active_grid.end_pos = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and active_grid.start_pos and active_grid.end_pos:
                    if dyn_grid and active_grid == coarse_grid:
                        active_grid = dyn_grid
                        grid = active_grid.grid
                        execute_dynamic_pathfinding(coarse_grid, dyn_grid, args)
                    else:
                        for row in grid:
                            for node in row:
                                node.update_neighbors(grid, args.heuristic)

                        if args.precheck:
                            algo.bfs_precheck(active_grid.start_pos, active_grid.end_pos)

                        algo.algorithm(lambda: active_grid.draw(), grid, active_grid.start_pos, active_grid.end_pos, args.heuristic)

                if event.key == pygame.K_d and dyn_grid:
                    active_grid = dyn_grid if active_grid == coarse_grid else coarse_grid
                    grid = active_grid.grid

                if event.key == pygame.K_c:
                    if active_grid == coarse_grid:
                        coarse_grid = grid_manager.Grid(args.size, width, WIN)
                        coarse_grid.start_pos = None
                        coarse_grid.end_pos = None
                        active_grid = coarse_grid
                        
                        if dyn_grid:
                            dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN, dyn_map_img)
                            dyn_grid.start_pos = None
                            dyn_grid.end_pos = None
                    else:
                        dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN)
                        dyn_grid.start_pos = None
                        dyn_grid.end_pos = None
                        active_grid = dyn_grid
                    grid = active_grid.grid

                if event.key == pygame.K_r:
                    if active_grid == coarse_grid:
                        coarse_grid = grid_manager.Grid(args.size, width, WIN, map_img)
                        coarse_grid.start_pos = None
                        coarse_grid.end_pos = None
                        if args.use_map:
                            coarse_grid.load_map()
                        active_grid = coarse_grid
                        
                        if dyn_grid:
                            dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN, dyn_map_img)
                            dyn_grid.start_pos = None
                            dyn_grid.end_pos = None
                    else:
                        dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN, dyn_map_img)
                        dyn_grid.start_pos = None
                        dyn_grid.end_pos = None
                        dyn_grid.load_map() 
                        active_grid = dyn_grid
                    grid = active_grid.grid

                if event.key == pygame.K_t:
                    active_grid.toggle_search_area()
                
                # Catch 'g' in the main loop to toggle the guideline
                if event.key == pygame.K_g:
                    active_grid.toggle_guideline()

    pygame.quit()

main(WIN, WIDTH)