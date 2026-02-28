#!/usr/bin/env python3
"""
Entry point for the Rover A* Pathfinding Tool.
"""

# External dependencies
import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import argparse
import itertools
import yaml
from PIL import Image

# Internal dependencies
import grid_manager
import algo

# Pygame initialization
WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
MAPS_DIR = "maps"
CONFIG_DIR = "config"
pygame.display.set_caption("Rover A* Pathfinding Tool")
pygame.init()

def compute_tsp_matrix(grid_obj, heuristic):
    """
    Silently calculates the A* distance matrix for the start node and all destinations.
    Evaluates every permutation and returns the optimal route and a generated report.
    """
    for row in grid_obj.grid:
        for node in row:
            node.update_neighbors(grid_obj.grid, heuristic)

    tsp_report = []
    tsp_report.append("\n" + "="*50)
    tsp_report.append("TSP ROUTE OPTIMIZATION REPORT")
    tsp_report.append("="*50)
    
    # Build the Distance Matrix
    all_points = [grid_obj.start_pos] + grid_obj.destinations
    distances = {}

    tsp_report.append(f"Calculated distance matrix for {len(all_points)} total points.")
    for i in range(len(all_points)):
        for j in range(i + 1, len(all_points)):
            node_a = all_points[i]
            node_b = all_points[j]
            
            path = algo.algorithm(lambda: None, grid_obj.grid, node_a, node_b, heuristic)
            
            if path:
                cost = len(path) + sum(n.extra_cost for n in path)
                distances[(node_a, node_b)] = cost
                distances[(node_b, node_a)] = cost
            else:
                distances[(node_a, node_b)] = float('inf')
                distances[(node_b, node_a)] = float('inf')
                
    # Clean up ALL silent A* search artifacts (red/green/purple) off the visual grid
    for row in grid_obj.grid:
        for node in row:
            if node.is_open_node or node.is_closed_node:
                node.is_open_node = False
                node.is_closed_node = False
            
            if node != grid_obj.start_pos and node not in grid_obj.destinations:
                if hasattr(node, 'is_path_node'): node.is_path_node = False
                elif hasattr(node, 'is_path'): node.is_path = False
                elif hasattr(node, 'path'): node.path = False
            
            node.update_color(show_search=getattr(grid_obj, 'show_search', False))

    # Evaluate all possible route permutations
    perms = list(itertools.permutations(grid_obj.destinations))
    best_cost = float('inf')
    best_perm = None
    
    tsp_report.append(f"\nEvaluating {len(perms)} possible route permutations:")
    for perm in perms:
        current_cost = 0
        current_node = grid_obj.start_pos
        valid = True
        seq_str = f"Start[{current_node.row},{current_node.col}]"
        
        for dest in perm:
            segment_cost = distances.get((current_node, dest), float('inf'))
            if segment_cost == float('inf'):
                valid = False
                break
            
            current_cost += segment_cost
            seq_str += f" -> Dest[{dest.row},{dest.col}]"
            current_node = dest
            
        if valid:
            tsp_report.append(f"{seq_str} | Total Cost: {current_cost}")
            if current_cost < best_cost:
                best_cost = current_cost
                best_perm = perm
        else:
            tsp_report.append(f"{seq_str} | Total Cost: INF (Path Blocked)")

    return best_perm, best_cost, tsp_report

def execute_dynamic_pathfinding(coarse_grid, dyn_grid, args, animate=True, vis_coarse=False):
    """
    Executes a Hierarchical Pathfinding simulation using a 'Sense, Plan, Act' loop.
    ... [Docstring omitted for brevity, but stays conceptually the same] ...
    """
    print("Starting Dynamic Hierarchical Pathfinding...")
    print("Calculating TSP matrix silently in the background...")
    
    # ==========================================
    # 1: MACRO-PLANNING & TSP (COARSE GRID)
    # ==========================================
    
    best_perm, best_cost, tsp_report = compute_tsp_matrix(coarse_grid, args.heuristic)

    if not best_perm:
        print("ERR: No valid macro-route found connecting all destinations.")
        return

    tsp_report.append("\n" + "-"*50)
    tsp_report.append(f"OPTIMAL MACRO-ROUTE FOUND (Cost: {best_cost})")
    tsp_report.append("-" * 50)

    # Stitch the optimal path segments together
    coarse_path = []
    current_start = coarse_grid.start_pos
    draw_func = (lambda: coarse_grid.draw()) if vis_coarse else (lambda: None)
    
    for dest in best_perm:
        segment_path = algo.algorithm(draw_func, coarse_grid.grid, current_start, dest, args.heuristic)
        
        if coarse_path:
            coarse_path.extend(segment_path[1:]) 
        else:
            coarse_path.extend(segment_path)
            
        current_start = dest
        
    # Clean up the grid BEFORE the pause
    gps_guideline = []
    for c_node in coarse_path:
        center_x = c_node.x + (c_node.width // 2)
        center_y = c_node.y + (c_node.width // 2)
        gps_guideline.append((center_x, center_y))
        
        # Strip the purple path state from the node
        if c_node != coarse_grid.start_pos and c_node not in coarse_grid.destinations:
            if hasattr(c_node, 'is_path_node'): c_node.is_path_node = False
            if hasattr(c_node, 'is_path'): c_node.is_path = False
            if hasattr(c_node, 'path'): c_node.path = False

    for row in coarse_grid.grid:
        for node in row:
            if node.is_open_node or node.is_closed_node:
                node.is_open_node = False
                node.is_closed_node = False
            node.update_color(show_search=getattr(coarse_grid, 'show_search', False))

    coarse_grid.global_route = gps_guideline
    coarse_grid.show_guideline = True 
    
    if vis_coarse and animate:
        coarse_grid.draw() 
        pygame.time.delay(1200)

    dyn_grid.global_route = gps_guideline

    # ==========================================
    # 2: SETUP DYNAMIC GRID TRANSLATION
    # ==========================================
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

    # ==========================================
    # 3: SENSE, PLAN, ACT LOOP
    # ==========================================
    for waypoint in coarse_path[1:]:
        if animate:
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
                
        # --- SENSE & PLAN ---
        dyn_grid.sensor_sweep(rover_node, radius=args.sensor)
        draw_func = (lambda: dyn_grid.draw()) if animate else (lambda: None)
        local_path = algo.algorithm(draw_func, dyn_grid.grid, rover_node, target_node, args.heuristic)
        
        if not local_path or len(local_path) < 2:
            print(f"ERR: Rover got stuck at [{rover_node.row}, {rover_node.col}]! Obstacles blocked the way.")
            return
        
        # --- ACT ---
        for step_node in local_path[1:]:
            if animate:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        quit()
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_g:
                            dyn_grid.toggle_guideline()
            
            historical_path.append(rover_node)
            rover_node.set_path()      
            
            rover_node = step_node
            rover_node.set_start()     
            
            if animate:
                dyn_grid.draw()
                pygame.time.delay(10)      
        
        # ==========================================
        # 4: LOCAL CLEANUP
        # ==========================================
        for row in dyn_grid.grid:
            for node in row:
                if node.is_open_node or node.is_closed_node:
                    node.is_open_node = False
                    node.is_closed_node = False
                    node.update_color(show_search=dyn_grid.show_search)
        
        for node in historical_path:
            node.set_path()
        
        global_end_node.set_end()
        
        if animate:
            dyn_grid.draw()
            
    # ==========================================
    # 5: FINAL SIMULATION CLEANUP & METRICS
    # ==========================================
    coarse_steps = len(coarse_path) - 1
    coarse_terrain_cost = sum(node.extra_cost for node in coarse_path)
    
    dyn_steps = len(historical_path) 
    dyn_terrain_cost = sum(node.extra_cost for node in historical_path) + global_end_node.extra_cost

    equiv_coarse_steps = coarse_steps * scale
    equiv_coarse_cost = coarse_terrain_cost * scale
    
    coarse_avg_severity = coarse_terrain_cost / max(1, coarse_steps)
    dyn_avg_severity = dyn_terrain_cost / max(1, dyn_steps)

    print("\n\n" + "*"*70)
    print("FINAL SIMULATION REPORT")
    print("*"*70)
    
    print("\n".join(tsp_report))

    print("\n" + "="*50)
    print("SUCCESS: Destination reached on the dynamic map!")
    print("-" * 50)
    print("PATHFINDING METRICS (Apples-to-Apples):")
    print(f"Coarse Macro-Route (Scaled) : {int(equiv_coarse_steps)} steps | Est. Terrain Penalty: {int(equiv_coarse_cost)}")
    print(f"Dynamic Micro-Route (Actual): {dyn_steps} steps | Actual Terrain Penalty: {dyn_terrain_cost}")
    print("-" * 50)
    print("TERRAIN SEVERITY (Normalized Cost Per Step):")
    print(f"Expected Severity: {coarse_avg_severity:.2f} penalty/step")
    print(f"Actual Severity  : {dyn_avg_severity:.2f} penalty/step")
    print("="*50 + "\n")

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
    
    parser.add_argument("heuristic", type=str, nargs="?", choices=["manhattan", "euclidean", "octile"], help="Choose the heuristic function.")
    parser.add_argument("--config", type=str, help="Path to a YAML configuration file.")
    
    parser.add_argument("--dynamic", type=str, help="Load a higher-resolution map.")
    parser.add_argument("--sensor", type=int, default=1, help="Radius of the rover's sensor sweep.")
    parser.add_argument("-p", "--precheck", action="store_true", help="Run a BFS precheck.")

    size_excl_group = parser.add_mutually_exclusive_group()
    size_excl_group.add_argument("--size", type=int, default=50, help="Grid size.")
    size_excl_group.add_argument("--use_map", type=str, help="Choose an image to use for a predefined map.")
 
    mode_excl_group = parser.add_mutually_exclusive_group()
    mode_excl_group.add_argument("--path_only", type=int, nargs="+", help="Enter two points in the form [X1 Y1 X2 Y2 ...].")
    mode_excl_group.add_argument("--mode", type=str, choices=["sandbox", "fast", "full"], 
                                 help="Execution Mode. 'sandbox': Free play. 'fast': Silent Coarse -> Vis Dynamic. 'full': Vis Coarse -> Vis Dynamic.")
    
    # ==========================================
    # YAML CONFIGURATION INJECTION
    # ==========================================
    # 1. Parse *only* known arguments first to see if a --config flag was passed
    temp_args, remaining_argv = parser.parse_known_args()
    
    # 2. If a config file exists, route it to the config directory, load it, and inject defaults
    if temp_args.config:
        config_path = os.path.join(CONFIG_DIR, temp_args.config)
        
        if not os.path.exists(config_path):
            parser.error(f"ERR: Could not find configuration file at: {config_path}")
            
        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)
            if config_data:
                parser.set_defaults(**config_data)
                
    # 3. Parse EVERYTHING. Any explicitly typed CLI flags will automatically override the YAML defaults
    args = parser.parse_args()
    
    # 4. Final check to ensure a heuristic was provided (either via YAML or CLI)
    if not args.heuristic:
        parser.error("ERR: A heuristic must be provided either via the command line or a YAML config file.")

    # ==========================================
    # INPUT VALIDATION & PATH ROUTING
    # ==========================================
    if args.path_only:
        # Check for at least 4 coordinates, and ensure it's an even number (pairs of X,Y)
        if len(args.path_only) < 4 or len(args.path_only) % 2 != 0:
            parser.error("ERR: --path_only requires pairs of coordinates (e.g., StartX StartY Dest1X Dest1Y Dest2X Dest2Y...).")
        for coord in args.path_only:
            if coord >= args.size or coord < 0:
                parser.error("ERR: Invalid coord in --path_only. Out of grid bounds.")

    map_path = None
    if args.use_map:
        map_path = os.path.join(MAPS_DIR, args.use_map)
        if not os.path.exists(map_path):
            parser.error(f"ERR: Could not find the map image at: {args.use_map}")

    dyn_map_path = None
    if args.dynamic:
        if not args.use_map:
            parser.error("ERR: Dynamic mode must be used in conjunction with a coarse map (--use_map).")
        
        dyn_map_path = os.path.join(MAPS_DIR, args.dynamic)
        if not os.path.exists(dyn_map_path):
            parser.error(f"ERR: Could not find the dynamic map image at: {args.dynamic}")
    else:
        if args.sensor != 1: 
            parser.error("--sensor can only be used in conjunction with the --dynamic flag.")
        if args.mode is not None:
            parser.error("--mode can only be used in conjunction with the --dynamic flag.")
    
    # ==========================================
    # INITIALIZE PARAMETERS
    # ==========================================
    if not args.mode:
        args.mode = "fast"
        
    map_img = None  
    dyn_map_img = None
    dyn_map_size = None
    
    if args.use_map:
        map_img = Image.open(map_path)
        args.size = map_img.width

    if args.dynamic:
        dyn_map_img = Image.open(dyn_map_path)
        dyn_map_size = dyn_map_img.width

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
        if args.mode == "sandbox":
            dyn_grid.load_map()
        
    active_grid = coarse_grid
    grid = active_grid.grid

    # ==========================================
    # INITIALIZE UI BRUSH STATE & TSP LISTS
    # ==========================================
    selected_brush = 'Barrier (5)' 
    coarse_grid.destinations = []
    if dyn_grid:
        dyn_grid.destinations = []
    active_grid.selected_brush = selected_brush

    if args.path_only:
        coords = args.path_only
        
        # 1. Assign the Start Node (First pair)
        start_x, start_y = coords[0], coords[1]
        coarse_grid.start_pos = coarse_grid.grid[start_x][start_y]
        coarse_grid.start_pos.set_start()
        
        # 2. Assign all subsequent pairs as Destination Nodes
        for i in range(2, len(coords), 2):
            dest_x, dest_y = coords[i], coords[i+1]
            dest_node = coarse_grid.grid[dest_x][dest_y]
            coarse_grid.destinations.append(dest_node)
            dest_node.set_end()

        # 3. Trigger the appropriate execution mode
        if dyn_grid:
            active_grid = dyn_grid
            active_grid.draw()
            execute_dynamic_pathfinding(coarse_grid, dyn_grid, args, animate=False)
        else:
            active_grid.draw()
            # If no dynamic grid, just run the TSP matrix and render the coarse result immediately
            best_perm, best_cost, tsp_report = compute_tsp_matrix(coarse_grid, args.heuristic)
            print("\n".join(tsp_report))
            if best_perm:
                current_start = coarse_grid.start_pos
                for dest in best_perm:
                    algo.algorithm(lambda: None, coarse_grid.grid, current_start, dest, args.heuristic)
                    current_start = dest
            active_grid.draw()

    run = True
    while run:
        active_grid.draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            # ==========================================
            # KEYBOARD EVENTS
            # ==========================================
            if event.type == pygame.KEYDOWN:
                
                # --- CHANGE BRUSH TYPE (0-5, 9) ---
                if event.key == pygame.K_9:
                    selected_brush = 'Start (9)'
                elif event.key == pygame.K_0:
                    selected_brush = 'Dest (0)'
                elif event.key == pygame.K_1:
                    selected_brush = 'Cost 1'
                elif event.key == pygame.K_2:
                    selected_brush = 'Cost 2'
                elif event.key == pygame.K_3:
                    selected_brush = 'Cost 3'
                elif event.key == pygame.K_4:
                    selected_brush = 'Cost 4'
                elif event.key == pygame.K_5:
                    selected_brush = 'Barrier (5)'
                
                active_grid.selected_brush = selected_brush
                
                # --- START PATHFINDING ---
                if event.key == pygame.K_SPACE and active_grid.start_pos and len(active_grid.destinations) > 0:
                    
                    if args.mode == "sandbox" or not dyn_grid:
                        best_perm, best_cost, tsp_report = compute_tsp_matrix(active_grid, args.heuristic)
                        
                        print("\n".join(tsp_report))

                        if best_perm:
                            print("\n" + "-"*50)
                            print(f"OPTIMAL ROUTE FOUND (Cost: {best_cost})")
                            print("-" * 50)
                            print("Animating sequence...")
                            
                            current_start = active_grid.start_pos
                            for dest in best_perm:
                                algo.algorithm(lambda: active_grid.draw(), grid, current_start, dest, args.heuristic)
                                current_start = dest 
                                
                            print("Simulation Complete.\n")
                        else:
                            print("\nERR: No valid route connecting all destinations exists due to barriers.")

                    elif args.mode == "fast":
                        if dyn_grid and active_grid == coarse_grid:
                            active_grid = dyn_grid
                            grid = active_grid.grid
                            execute_dynamic_pathfinding(coarse_grid, dyn_grid, args, animate=True, vis_coarse=False)

                    elif args.mode == "full":
                        if dyn_grid and active_grid == coarse_grid:
                            active_grid = dyn_grid
                            grid = active_grid.grid
                            execute_dynamic_pathfinding(coarse_grid, dyn_grid, args, animate=True, vis_coarse=True)

                # --- TOGGLE MAPS ---
                if event.key == pygame.K_d and dyn_grid:
                    active_grid = dyn_grid if active_grid == coarse_grid else coarse_grid
                    grid = active_grid.grid
                    active_grid.selected_brush = selected_brush

                # --- CLEAR GRIDS (Soft Reset) ---
                if event.key == pygame.K_c:
                    if active_grid == coarse_grid:
                        coarse_grid = grid_manager.Grid(args.size, width, WIN)
                        coarse_grid.start_pos = None
                        coarse_grid.end_pos = None
                        coarse_grid.destinations = []
                        active_grid = coarse_grid
                        if dyn_grid:
                            dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN, dyn_map_img)
                            dyn_grid.start_pos = None
                            dyn_grid.end_pos = None
                            dyn_grid.destinations = []
                    else:
                        dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN)
                        dyn_grid.start_pos = None
                        dyn_grid.end_pos = None
                        dyn_grid.destinations = []
                        active_grid = dyn_grid
                    grid = active_grid.grid
                    active_grid.selected_brush = selected_brush

                # --- RELOAD GRIDS (Hard Reset) ---
                if event.key == pygame.K_r:
                    if active_grid == coarse_grid:
                        coarse_grid = grid_manager.Grid(args.size, width, WIN, map_img)
                        coarse_grid.start_pos = None
                        coarse_grid.end_pos = None
                        coarse_grid.destinations = []
                        if args.use_map:
                            coarse_grid.load_map()
                        active_grid = coarse_grid
                        
                        if dyn_grid:
                            dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN, dyn_map_img)
                            dyn_grid.start_pos = None
                            dyn_grid.end_pos = None
                            dyn_grid.destinations = []
                            if args.mode == "sandbox":
                                dyn_grid.load_map()
                    else:
                        dyn_grid = grid_manager.Grid(dyn_map_size, width, WIN, dyn_map_img)
                        dyn_grid.start_pos = None
                        dyn_grid.end_pos = None
                        dyn_grid.destinations = []
                        dyn_grid.load_map() 
                        active_grid = dyn_grid
                    grid = active_grid.grid
                    active_grid.selected_brush = selected_brush

                # --- TOGGLES ---
                if event.key == pygame.K_t:
                    active_grid.toggle_search_area()
                if event.key == pygame.K_g:
                    active_grid.toggle_guideline()

            # ==========================================
            # MOUSE EVENTS
            # ==========================================
            
            # --- LEFT CLICK (PAINT) ---
            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = active_grid.get_clicked_pos(pos)
                
                if 0 <= row < active_grid.rows and 0 <= col < active_grid.rows:
                    node = grid[row][col]
                    
                    if selected_brush == 'Start (9)':
                        if active_grid.start_pos:
                            active_grid.start_pos.reset()
                        active_grid.start_pos = node
                        node.set_start()
                        
                    elif selected_brush == 'Dest (0)':
                        if node not in active_grid.destinations and node != active_grid.start_pos:
                            active_grid.destinations.append(node)
                            node.set_end()
                            
                    elif selected_brush == 'Barrier (5)':
                        if node != active_grid.start_pos and node not in active_grid.destinations:
                            node.set_barrier()
                            
                    elif selected_brush.startswith('Cost'):
                        if node != active_grid.start_pos and node not in active_grid.destinations:
                            weight = int(selected_brush.split(' ')[1])
                            node.extra_cost = weight
                            node.is_barrier_node = False
                            node.update_color(show_search=active_grid.show_search)

            # --- RIGHT CLICK (ERASE) ---
            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = active_grid.get_clicked_pos(pos)
                
                if 0 <= row < active_grid.rows and 0 <= col < active_grid.rows:
                    node = grid[row][col]
                    node.reset()
                    
                    if node == active_grid.start_pos:
                        active_grid.start_pos = None
                    if node in active_grid.destinations:
                        active_grid.destinations.remove(node)

    pygame.quit()

main(WIN, WIDTH)