#!/usr/bin/env python3
"""
Entry point for the Rover A* Pathfinding Tool.

This module initializes the Pygame environment, parses command-line arguments,
constructs and manages the grid, and coordinates user interaction with the
pathfinding algorithms. It supports interactive map editing, predefined map
loading, weighted terrain, multiple heuristics, optional reachability prechecks,
and headless path-only execution modes.

All rendering, input handling, and high-level program flow are orchestrated here,
while pathfinding logic and grid utilities are delegated to dedicated modules.
"""

# External dependencies
import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import argparse
from PIL import Image

# Internal dependencies
import grid_manager
import algo

WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
MAPS_DIR = "maps"
pygame.display.set_caption("Rover A* Pathfinding Tool")
pygame.init()

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
    map_img = None  # Initialize map_img to pass to Grid securely

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

    # === Instantiate the new Grid class ===
    my_grid = grid_manager.Grid(args.size, width, win, map_img)
    
    if args.use_map:
        my_grid.load_map()
        
    # Keep a reference to the 2D array for algo functions and iterating
    grid = my_grid.grid

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
            algo.bfs_precheck(start_pos, end_pos)

        # Run algorithm with dummy draw function
        algo.algorithm(lambda: None, grid, start_pos, end_pos, args.heuristic)

        # Now show window and draw final state
        global WIN
        WIN = pygame.display.set_mode((width, width))
        my_grid.win = WIN  # Update the window reference in the class
        my_grid.draw()

    run = True
    while run:
        my_grid.draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = my_grid.get_clicked_pos(pos)
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
                row, col = my_grid.get_clicked_pos(pos)
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
                        algo.bfs_precheck(start_pos, end_pos)

                    algo.algorithm(lambda: my_grid.draw(), grid, start_pos, end_pos, args.heuristic)

                if event.key == pygame.K_c:
                    start_pos = None
                    end_pos = None
                    my_grid = grid_manager.Grid(args.size, width, WIN)
                    grid = my_grid.grid

                if event.key == pygame.K_r:
                    start_pos = None
                    end_pos = None
                    my_grid = grid_manager.Grid(args.size, width, WIN, map_img)
                    if args.use_map:
                        my_grid.load_map()
                    grid = my_grid.grid

                if event.key == pygame.K_t:
                    my_grid.toggle_search_area()

    pygame.quit()

main(WIN, WIDTH)