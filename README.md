# ROVER PATHFINDING/PATHPLANNING USING A* 

Initially being developed for CENG 6332 (High Performance Computer Architecture) and subsequently being built upon for thesis, the goal of this project is to develop an understanding of pathfinding algorithms and how they could be used, particularly in an HPC environment, to calculate optimal paths for lunar rovers such as *VIPER*.
<br><br>

![image](https://github.com/user-attachments/assets/c70ffcf4-3bcb-4368-9cf8-a9408b967010)


## To Do:
- [X] Implement Single-heuristic D*
- [ ] DEM to PNG conversion script
- [ ] Add multi-heuristic support
- - [ ] Mock lighting
- - [ ] Mock slope
- - [ ] Mock comms line-of-sight
- [X] Multiple science stations (TSP)
- [ ] Field-based traversal
<br><br>

## Getting Started
**Creating the Virtual Environment**

1. Upon a fresh clone, run `./setup.sh` to create the _raster-venv_ virtual environment and install the required python packages.
2. Run `source ~/.bashrc` to load the newly created _rastar_ alias.
3. Enter `rastar` to activate the virtual environment, and `deactivate` to deactivate it.


## Using main.py
**Runs the A\* Pathfinding visualization.**

In a new grid, use the following controls to place/remove nodes, weighted terrain, and barriers.
- **Left Click**: Place the currently selected node at the cursor's location. This will overwrite the weight of the node that was there previously.
- **Right Click**: Remove the node at the cursor's location. If map data was loaded using --use_map, the original node from the loaded map will be restored. Otherwise, it will return to an empty node.
- **Key 1**: Select the Cost 1 node. This node will have an extra weight of 1.
- **Key 2**: Select the Cost 2 node. This node will have an extra weight of 2.
- **Key 3**: Select the Cost 3 node. This node will have an extra weight of 3.
- **Key 4**: Select the Cost 4 node. This node will have an extra weight of 4.
- **Key 5**: Select the Cost 5 (Barrier) node. This node is completely impassable.
- **Key 9**: Select the start node (Teal). Only a single start node can be placed. Attempting to place a second start node will remove the preexisting start node.
- **Key 0**: Select the destination node (Orange). Multiple destination nodes can be placed.

Additional controls include:
- **Spacebar**: Start running the algorithm. Requires at least one start node and destination node on the grid.
- **Key C**: Completely clear the entire currently selected grid, setting all nodes to empty. Only available when the algorithm is not running.
- **Key R**: Reset the currently selected grid back to to its initial state. If a map was loaded using --use_map, this data is loaded. Otherwise, the grid is completely reset. Only available when the algorithm is not running.
- **Key T**: Toggles the search area. After a search is completed, toggle between rendering the entire search area or just the path that has been found. Only available when the algorithm is not running.
- **Key D**: Toggles between the Coarse map and the Dynamic map. Only available when using --dynamic and when the algorithm is not running.
- **Key G**: Toggles the Golden macro-path guideline overlay on the Dynamic map. Only available when using --dynamic and when the algorithm is not running.

After the algorithm completes, the elapsed time will display in the console output.

**Hierarchical Dynamic Pathfinding (Sense, Plan, Act)**
By supplying both *--use_map* (coarse global map) and *--dynamic* (high-resolution local map), the script simulates how real rovers navigate.
1. The algorithm first calculates a global macro-route across the coarse map.
2. The simulation then switches to the dynamic map, where the rover is initially "blind" to high-resolution obstacles.
3. The rover executes a continuous loop: it sweeps its sensors (revealing hidden barriers), calculates a micro-path toward the next coarse waypoint, and drives forward.

---

Arguments: 
- _heuristic_, **required**, positional: Tell the script which heuristic function to use. [manhattan, euclidean, octile]
- _config_: Name of a yaml file in the /config directory. Parameters defined in the yaml act as defaults, and can be overwritten by command line arguments.
- _size_: Width/height of the grid; 50 by default. Will be overwritten by the size of a map if using --use_map.
- _use_map_: The full name of an image in the _maps_ subdirectory. Defines barrier/empty nodes. Replaces the size of the grid if using _size_.
- _path_only_: Supply two node locations in the form [X1 Y1 X2 Y2]. Running using this arg will only render the final path between these two nodes. Can be used with or without loading a map.
- _precheck_: Runs a quick BFS to confirm that any path exists from the start node to the end node.
- _dynamic_: Load a higher-resolution map image from the maps subdirectory to run the hierarchical "Sense, Plan, Act" simulation. Must be used in conjuction with *--use_map*.
- _sensor_: Radius of the rover's sensor sweep when running a dynamic simulation. Defines how far the rover can "see" obstacles (default: 1, creating a 3x3 visibility grid).
- _mode_: Select the execution mode. Execution Mode. 'sandbox': Free play. 'fast': Silent Coarse -> Vis Dynamic. 'full': Vis Coarse -> Vis Dynamic.

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
