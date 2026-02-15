# ROVER PATHFINDING/PATHPLANNING USING A* 

Initially being developed for CENG 6332 (High Performance Computer Architecture) and subsequently being built upon for thesis, the goal of this project is to develop an understanding of pathfinding algorithms and how they could be used, particularly in an HPC environment, to calculate optimal paths for lunar rovers such as *VIPER*.
<br><br>

![image](https://github.com/user-attachments/assets/c70ffcf4-3bcb-4368-9cf8-a9408b967010)


## To Do:
- [X] Implement Single-heuristic D*
- [ ] Add multi-heuristic support
- - [ ] Mock lighting
- - [ ] Mock slope
- - [ ] Mock comms line-of-sight
- [ ] Multiple science stations (TSP)
- [ ] Field-based traversal
<br><br>
## Using main.py
**Runs the A\* Pathfinding visualization.**

In a new grid, the first left click will place the start node (teal). The second left click will place the goal node (orange). Any left clicks after these two nodes are placed will be barrier nodes (black).

- Nodes can be erased by using right click.
- A grid can be cleared by pressing the **"C"** key.
- If running using a map, pressing **"R"** will reset to the initial state of that map. *(Note: If viewing the dynamic map, pressing R will fully load its barriers, allowing for sandbox pathfinding on the high-resolution grid).*
- After the algorithm completes, pressing **"T"** will toggle the search area, and will only show the start, end, and path nodes.
- When running a dynamic simulation, pressing **"D"** toggles the view between the coarse global grid and the dynamic local grid.
- Pressing **"G"** toggles the golden macro-path guideline overlay on the dynamic map.

Hitting the **Spacebar** will start the algorithm, as long as the start and goal nodes are placed on the grid.

After the algorithm completes, the elapsed time will display in the console output.

**Hierarchical Dynamic Pathfinding (Sense, Plan, Act)**
By supplying both *--use_map* (coarse global map) and *--dynamic* (high-resolution local map), the script simulates how real rovers navigate.
1. The algorithm first calculates a global macro-route across the coarse map.
2. The simulation then switches to the dynamic map, where the rover is initially "blind" to high-resolution obstacles.
3. The rover executes a continuous loop: it sweeps its sensors (revealing hidden barriers), calculates a micro-path toward the next coarse waypoint, and drives forward.

---

Arguments: 
- _heuristic_, **required**, positional: Tell the script which heuristic function to use. [manhattan, euclidean, octile]
- _size_: Width/height of the grid; 50 by default. Will be overwritten by the size of a map if using --use_map.
- _use_map_: The full name of an image in the _maps_ subdirectory. Defines barrier/empty nodes. Replaces the size of the grid if using _size_.
- _path_only_: Supply two node locations in the form [X1 Y1 X2 Y2]. Running using this arg will only render the final path between these two nodes. Can be used with or without loading a map.
- _precheck_: Runs a quick BFS to confirm that any path exists from the start node to the end node.
- _dynamic_: Load a higher-resolution map image from the maps subdirectory to run the hierarchical "Sense, Plan, Act" simulation. Must be used in conjuction with *--use_map*.
- _sensor_: Radius of the rover's sensor sweep when running a dynamic simulation. Defines how far the rover can "see" obstacles (default: 1, creating a 3x3 visibility grid).

<br><br>
## Using img_to_grid.py
**Convert a square image into a map for use in main.py.**

Downscales an image to the specified size.

- If using the _binary_ flag, also clamps pixels to blackor white using the value provided. For example, if a value supplied for _binary_ is **.5**, any pixel that has a brightness of 128 (50% of the max value, 255, aka white) or higher will be set to **white**. Otherwise, it will be **black**.
- If using the _fivesplit_ flag, pixels will be clamped to their respective colors.FIVESPLIT_<**n**> color based on the RGB value range they fall into, as defined by the range created using the different split points.

Arguments: 
- _source_img_, **required**, positional: Tell the script which image to use in the _source_images_ subdirectory.
- _size_, **required**, positional: Width/height of the of the new map image. Must be smaller than the original image.
- _binary_: Threshold used to set pixels to either black (barrier) or white (empty node). A higher value means more barriers. Float value between 0 and 1.
- _fivesplit_: Indicate five "split points" which divide grayscale (0-255) into varying edge weights. Each divided area closer to black (RGB 0,0,0) has an incremented edge weight, starting at 0. See below (using _--fivesplit 50 80 100 150 200_):
  ![fivesplit](https://github.com/user-attachments/assets/4a1448fd-f097-46e1-bab2-4002c5020918)



<br><br>
## Resources

[A* Literature](https://www.sciencedirect.com/science/article/pii/S1877050921000399?via%3Dihub)

[Algorithm Visualization Base](https://www.youtube.com/watch?v=JtiK0DOeI4A&ab_channel=TechWithTim)

---

Dylan Britain

Dr. Liwen Shih, University of Houston - Clear Lake, Spring 2025 - Fall 2026

---
