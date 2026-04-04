"""
Defines the Node class used to represent individual cells in the grid-based
pathfinding visualization.
"""

# External dependencies
import pygame

# Internal dependencies
import colors

class Node:
    """
    Represents a single cell in the grid used for pathfinding.
    """
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.width = width
        self.total_rows = total_rows
        self.x = row * width
        self.y = col * width
        
        # --- LOGICAL TERRAIN STATE ---
        self.is_barrier_node = False

        # --- INDEPENDENT HEURISTIC LAYERS ---
        self.costs = {
            'height': 0,
            'slope': 0,
            'fivesplit': 0
        }
        
        # --- A* SEARCH STATE ---
        self.is_start_node = False
        self.is_end_node = False
        self.is_path_node = False
        self.is_open_node = False
        self.is_closed_node = False
        
        # --- RENDER STATE ---
        self.color = colors.WHITE
        self.base_color = colors.WHITE # ---> NEW: Memory of the original terrain color
        self.neighbors = []

    def get_total_cost(self):
        """Returns the combined cost penalty of all active terrain layers."""
        if self.is_barrier_node:
            return float('inf')
        # TODO Add multipliers to weight heuristics: 
        # return (self.costs['slope'] * 2.0) + self.costs['height']
        return sum(self.costs.values())
        
    def get_pos(self):
        '''Returns the (row, column) position of the node.'''
        return self.row, self.col
    
    # --- STATE GETTERS ---
    def is_closed(self): return self.is_closed_node
    def is_open(self): return self.is_open_node
    def is_barrier(self): return self.is_barrier_node
    def is_start(self): return self.is_start_node
    def is_end(self): return self.is_end_node
    def is_path(self): return self.is_path_node

    # --- STATE SETTERS ---
    def set_closed(self):
        self.is_closed_node = True
        self.update_color()

    def set_open(self):
        self.is_open_node = True
        self.update_color()
    
    def set_barrier(self):
        self.reset_search_state()
        for key in self.costs:
            self.costs[key] = 0
        self.is_barrier_node = True
        self.base_color = colors.BLACK 
        self.update_color()

    def set_fivesplit(self, cost, color):
        """Helper to condense the fivesplit methods"""
        self.is_barrier_node = False
        self.costs['fivesplit'] = cost
        self.base_color = color
        self.update_color()

    # (Update your specific fivesplit methods to use the helper)
    def set_fivesplit1(self): self.set_fivesplit(1, colors.FIVESPLIT_1)
    def set_fivesplit2(self): self.set_fivesplit(2, colors.FIVESPLIT_2)
    def set_fivesplit3(self): self.set_fivesplit(3, colors.FIVESPLIT_3)
    def set_fivesplit4(self): self.set_fivesplit(4, colors.FIVESPLIT_4)

    def set_grayscale_cost(self, gray_value, layer_type):
        """
        Maps a 0-255 grayscale pixel to a specific heuristic layer.
        :param layer_type: String indicating the dictionary key (e.g., 'height', 'slope')
        """
        if gray_value == 0:
            self.set_barrier()
        else:
            self.costs[layer_type] = 255 - gray_value
            self.is_barrier_node = False

    def set_start(self):
        self.is_start_node = True
        self.update_color()
    
    def set_end(self):
        self.is_end_node = True
        self.update_color()

    def set_path(self):
        self.is_start_node = False
        self.is_end_node = False
        self.is_path_node = True
        self.update_color()

    # --- RESET METHODS ---
    def reset(self):
        """Hard reset: Clears everything, turning the node back into plain white terrain."""
        self.is_barrier_node = False
        for key in self.costs:
            self.costs[key] = 0
        self.base_color = colors.WHITE
        self.reset_search_state()
        
    def reset_search_state(self):
        """Soft reset: Clears only A* artifacts, preserving the underlying terrain."""
        self.is_start_node = False
        self.is_end_node = False
        self.is_path_node = False
        self.is_open_node = False
        self.is_closed_node = False
        self.update_color()

    # --- MASTER RENDER LOGIC ---
    def update_color(self, show_search=True):
        """
        Calculates the color of the node based on priority.
        UI Elements (Start/End) > Path > Search Area > Base Terrain.
        """
        # 1. Highest Priority: Explicit Pathing Markers
        if self.is_start_node:
            self.color = colors.ORANGE
        elif self.is_end_node:
            self.color = colors.TURQUOISE
        elif self.is_path_node:
            self.color = colors.PURPLE
            
        # 2. Search Artifacts (Toggleable)
        elif show_search and self.is_closed_node:
            self.color = colors.RED
        elif show_search and self.is_open_node:
            self.color = colors.GREEN
            
        # 3. Base Terrain
        else:
            # Fall back to whatever the node remembers its original terrain color being
            self.color = self.base_color

    def draw(self, win):
        """Draws the node as a colored square on the given pygame surface"""
        # Only draw if the color is DIFFERENT from the background image
        # In this case, we always want to draw the node's color unless it's perfectly white
        if self.color != colors.WHITE:
            pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid, heuristic):
        """Updates the list of valid neighboring nodes based on movement rules."""
        self.neighbors = []
        if heuristic == "manhattan":
            directions = [
                (1, 0), (-1, 0), (0, 1), (0, -1)  # Cardinal directions only
            ]
        else:
            directions = [
                (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal directions
                (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal directions
            ]
        for drow, dcol in directions:
            new_row, new_col = self.row + drow, self.col + dcol
            if 0 <= new_row < self.total_rows and 0 <= new_col < self.total_rows:
                if abs(drow) + abs(dcol) == 2:  # Diagonal movement
                    if not grid[new_row][new_col].is_barrier() and not (grid[self.row][new_col].is_barrier() or grid[new_row][self.col].is_barrier()):
                        self.neighbors.append(grid[new_row][new_col])
                else:
                    if not grid[new_row][new_col].is_barrier():
                        self.neighbors.append(grid[new_row][new_col])

    def __lt__(self, other):
        return False