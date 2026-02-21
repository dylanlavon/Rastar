#!/usr/bin/env python3
"""
Entry point for the Rover A* Pathfinding Tool.
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

# Pygame initialization
WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
MAPS_DIR = "maps"
pygame.display.set_caption("Rover A* Pathfinding Tool")
pygame.init()

def execute_dynamic_pathfinding(coarse_grid, dyn_grid, args, animate=True):
    """
    Executes a Hierarchical Pathfinding simulation using a 'Sense, Plan, Act' loop.

    This function bridges the gap between global route planning and local obstacle avoidance,
    simulating how a lunar rover navigates unknown, high-resolution terrain based on a 
    low-resolution orbital map. It performs the following sequence:
    
    1. Calculates a macro-route across the low-resolution `coarse_grid` (Global Planning).
    2. Translates the resulting coarse waypoints into bounding box coordinates on the `dyn_grid`.
    3. Generates a visual macro-guideline (golden line) representing the intended global path.
    4. Iteratively drives the rover towards the nearest edge of the next coarse waypoint by:
       - Sensing: Uncovering high-resolution obstacles using `sensor_sweep` within the rover's radius.
       - Planning: Running a localized A* search to navigate around newly discovered barriers.
       - Acting: Animating the rover one step forward along the local path and painting a historical trail.
    5. Performs automatic cleanup of A* search artifacts (open/closed nodes) between steps 
       and upon arrival at the final destination.

    :param coarse_grid: The Grid object representing the global, low-resolution map.
    :param dyn_grid: The Grid object representing the local, high-resolution map.
    :param args: Parsed command-line arguments containing configuration data (e.g., .heuristic, .precheck, .sensor).
    :param animate: Boolean controlling whether the pathfinding execution renders to the display frame-by-frame.
    """
    print("Starting Dynamic Hierarchical Pathfinding...")
    
    # ==========================================
    # 1: MACRO-PLANNING (COARSE GRID)
    # ==========================================
    
    # Initialize the neighbors for the low-resolution grid
    for row in coarse_grid.grid:
        for node in row:
            node.update_neighbors(coarse_grid.grid, args.heuristic)
            
    # Optional BFS precheck to ensure the global destination is actually reachable
    if args.precheck:
        algo.bfs_precheck(coarse_grid.start_pos, coarse_grid.end_pos)
        
    # Run the initial A* algorithm silently (no drawing) to get the global route
    coarse_path = algo.algorithm(lambda: None, coarse_grid.grid, coarse_grid.start_pos, coarse_grid.end_pos, args.heuristic)
    if not coarse_path:
        print("ERR: No valid path found on the coarse grid.")
        return

    # Extract the exact pixel coordinates of the coarse path to draw the golden macro-guideline
    gps_guideline = []
    for c_node in coarse_path:
        # Node.x/y is the top-left pixel, so we add half the width to hit dead center
        center_x = c_node.x + (c_node.width // 2)
        center_y = c_node.y + (c_node.width // 2)
        gps_guideline.append((center_x, center_y))
        
    # Attach the guideline to the dynamic grid so its draw() method can render it
    dyn_grid.global_route = gps_guideline

    # ==========================================
    # 2: SETUP DYNAMIC GRID TRANSLATION
    # ==========================================

    # Calculate the size difference between the grids
    scale = len(dyn_grid.grid) / len(coarse_grid.grid)
    
    # Map the coarse starting node to the exact center of its corresponding block on the dynamic grid
    c_start = coarse_path[0]
    rx = min(int(c_start.row * scale + scale / 2), len(dyn_grid.grid) - 1)
    ry = min(int(c_start.col * scale + scale / 2), len(dyn_grid.grid) - 1)
    
    # Place the rover at the calculated starting coordinates
    rover_node = dyn_grid.grid[rx][ry]
    dyn_grid.start_pos = rover_node
    rover_node.set_start()
    
    # Map the final coarse destination to the dynamic grid so we have a visual target (Turquoise node)
    c_end = coarse_path[-1]
    ex = min(int(c_end.row * scale + scale / 2), len(dyn_grid.grid) - 1)
    ey = min(int(c_end.col * scale + scale / 2), len(dyn_grid.grid) - 1)
    global_end_node = dyn_grid.grid[ex][ey]

    # Initialize a list to track exactly where the rover drives so we can draw a solid purple trail
    historical_path = []

    # ==========================================
    # 3: SENSE, PLAN, ACT LOOP
    # ==========================================

    # Loop through every macro-waypoint the coarse grid told us to visit
    for waypoint in coarse_path[1:]:
        
        # Keep the Pygame window responsive so it doesn't freeze or crash during the loop
        if animate:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
                # Allow the user to toggle the golden guideline mid-simulation
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_g:
                        dyn_grid.toggle_guideline()
                        dyn_grid.draw()

        # Calculate the literal physical boundaries (bounding box) of the next coarse node block
        min_x = int(waypoint.row * scale)
        max_x = int((waypoint.row + 1) * scale) - 1
        min_y = int(waypoint.col * scale)
        max_y = int((waypoint.col + 1) * scale) - 1
        
        # Clamp the local target to the nearest edge of the target bounding box. 
        # This prevents the rover from having to drive to the *exact center* of every coarse block.
        target_x = max(min_x, min(rover_node.row, max_x))
        target_y = max(min_y, min(rover_node.col, max_y))
        target_node = dyn_grid.grid[target_x][target_y]
        
        # If the rover is already on the edge of the bounding box, skip to the next waypoint
        if rover_node == target_node: 
            continue
        
        # Visually mark the local mini-goal
        target_node.set_end()
        
        # Update neighbors on the high-res grid before planning so A* knows about new barriers
        for row in dyn_grid.grid:
            for node in row:
                node.update_neighbors(dyn_grid.grid, args.heuristic)
                
        # --- SENSE ---
        # The rover pings its surroundings, uncovering actual high-res terrain from the hidden map image
        dyn_grid.sensor_sweep(rover_node, radius=args.sensor)
                
        # --- PLAN ---
        # Run A* locally to figure out how to get to the bounding box edge, animating the search if enabled
        draw_func = (lambda: dyn_grid.draw()) if animate else (lambda: None)
        local_path = algo.algorithm(draw_func, dyn_grid.grid, rover_node, target_node, args.heuristic)
        
        # If A* fails locally, the terrain is impassable
        if not local_path or len(local_path) < 2:
            print(f"ERR: Rover got stuck at [{rover_node.row}, {rover_node.col}]! Obstacles blocked the way.")
            return
        
        # --- ACT ---
        # Drive the rover step-by-step along the localized path we just generated
        for step_node in local_path[1:]:
            
            if animate:
                # Keep the window responsive during the driving animation
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        quit()
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_g:
                            dyn_grid.toggle_guideline()
            
            # Save the node we are currently standing on to our permanent trail
            historical_path.append(rover_node)
            rover_node.set_path()      # Color the old node purple
            
            # Move the rover to the next physical step
            rover_node = step_node
            rover_node.set_start()     # Color the new node orange
            
            if animate:
                # Render the frame and pause briefly to animate the movement
                dyn_grid.draw()
                pygame.time.delay(10)      
        
        # ==========================================
        # 4: LOCAL CLEANUP
        # ==========================================
        
        # Wipe the red/green A* search artifacts off the grid so the next local plan starts clean
        for row in dyn_grid.grid:
            for node in row:
                if node.is_open_node or node.is_closed_node:
                    node.is_open_node = False
                    node.is_closed_node = False
                    node.update_color(show_search=dyn_grid.show_search)
        
        # The cleanup loop above might have accidentally wiped parts of our purple trail; repaint it!
        for node in historical_path:
            node.set_path()
        
        # Ensure the final ultimate destination marker is still visible
        global_end_node.set_end()
        
        if animate:
            dyn_grid.draw()
            
    # ==========================================
    # 5: FINAL SIMULATION CLEANUP
    # ==========================================
    
    print("Destination reached on the dynamic map!")

    # Perform one last wipe of the search artifacts so the final UI presentation is completely clean
    for row in dyn_grid.grid:
        for node in row:
            if node.is_open_node or node.is_closed_node:
                node.is_open_node = False

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
            execute_dynamic_pathfinding(coarse_grid, dyn_grid, args, animate=False)
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
                        execute_dynamic_pathfinding(coarse_grid, dyn_grid, args, animate=True)
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