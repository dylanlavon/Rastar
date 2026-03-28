"""
Defines the Node class used to represent individual cells in the grid-based
pathfinding visualization.

Each Node stores its grid position, visual state, traversal cost, and neighbor
relationships, and provides helper methods for A* search, visualization, and
terrain handling.
"""

# External dependencies
import pygame

# Internal dependencies
import colors

class Node:
    """
    Represents a single cell in the grid used for pathfinding.

    A Node tracks its position, visual state, traversal cost, and valid neighbors.
    It supports weighted terrain, barriers, start/end nodes, and path visualization
    for algorithms such as A*.
    """
    def __init__(self, row, col, width, total_rows):
        """
        Initializes a grid node with position, size, and default traversal state.
        
        :param row: Row index of the node in the grid
        :param col: Column index of the node in the grid
        :param width: Pixel width of the node when drawn
        :param total_rows: Total number of rows/columns in the grid
        """
        self.row = row
        self.col = col
        self.width = width
        self.total_rows = total_rows
        self.x = row * width
        self.y = col * width
        
        # --- LOGICAL TERRAIN STATE ---
        self.is_barrier_node = False
        self.extra_cost = 0
        
        # --- A* SEARCH STATE ---
        self.is_start_node = False
        self.is_end_node = False
        self.is_path_node = False
        self.is_open_node = False
        self.is_closed_node = False
        
        # --- RENDER STATE ---
        self.color = colors.WHITE
        self.neighbors = []
        
    def get_pos(self):
        '''Returns the (row, column) position of the node.'''
        return self.row, self.col
    
    # --- STATE GETTERS ---
    def is_closed(self):
        return self.is_closed_node
    
    def is_open(self):
        return self.is_open_node
    
    def is_barrier(self):
        return self.is_barrier_node
    
    def is_start(self):
        return self.is_start_node
    
    def is_end(self):
        return self.is_end_node
    
    def is_path(self):
        return self.is_path_node

    # --- STATE SETTERS ---
    def set_closed(self):
        self.is_closed_node = True
        self.update_color()

    def set_open(self):
        self.is_open_node = True
        self.update_color()
    
    def set_barrier(self):
        self.reset_search_state() # Barriers shouldn't have search data
        self.extra_cost = 0
        self.is_barrier_node = True
        self.update_color()

    def set_fivesplit1(self):
        self.is_barrier_node = False
        self.extra_cost = 1
        self.update_color()
    
    def set_fivesplit2(self):
        self.is_barrier_node = False
        self.extra_cost = 2
        self.update_color()

    def set_fivesplit3(self):
        self.is_barrier_node = False
        self.extra_cost = 3
        self.update_color()

    def set_fivesplit4(self):
        self.is_barrier_node = False
        self.extra_cost = 4
        self.update_color()

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
        self.extra_cost = 0
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
        UI Elements (Start/End) > Path > Search Area > Terrain Weight > Empty Space.
        
        :param show_search: If False, hides the red/green A* search artifacts.
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
        elif self.is_barrier_node:
            self.color = colors.BLACK
        elif self.extra_cost == 1:
            self.color = colors.FIVESPLIT_1
        elif self.extra_cost == 2:
            self.color = colors.FIVESPLIT_2
        elif self.extra_cost == 3:
            self.color = colors.FIVESPLIT_3
        elif self.extra_cost == 4:
            self.color = colors.FIVESPLIT_4
            
        # 4. Default Space
        else:
            self.color = colors.WHITE

    def draw(self, win):
        """
        Draws the node as a colored square on the given pygame surface
        """
        if self.color != colors.WHITE:
            pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid, heuristic):
        """
        Updates the list of valid neighboring nodes based on movement rules.
        """
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

    def set_grayscale_cost(self, gray_value):
        """
        Maps a 0-255 grayscale pixel to a dynamic pathfinding cost.
        0 = Impassable Barrier
        1-255 = Traversable, where 255 is cost 0, and 1 is cost 254.
        """
        if gray_value == 0:
            self.set_barrier()
        else:
            self.extra_cost = 255 - gray_value
            self.is_barrier_node = False
            
            # Unpack the gray value into RGB so Pygame can draw it
            self.color = (gray_value, gray_value, gray_value)

    def __lt__(self, other):
        """Dummy comparison function required for PriorityQueue compatibility."""
        return False