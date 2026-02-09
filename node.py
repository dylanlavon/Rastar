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
        Initializs a grid node with position, size, and default traversal state.
        
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
        self.color = colors.WHITE
        self.neighbors = []
        self.extra_cost = 0
        self.prev_color = self.color
        self.node_type = "untraversed"
        
    def get_pos(self):
        '''Returns the (row, column) position of the node.'''
        return self.row, self.col
    
    def is_closed(self):
        '''Returns True if the node has been fully explored by the algorithm. (Node color is red)'''
        return self.color == colors.RED
    
    def is_open(self):
        '''Returns True if the node is currently in the open set. (Node color is green)'''
        return self.color == colors.GREEN
    
    def is_barrier(self):
        '''Returns True if the node is an impassable barrier (Node color is black)'''
        return self.color == colors.BLACK
    
    def is_fivesplit1(self):
        '''Returns True if the node color matches FIVESPLIT_1'''
        return self.color == colors.FIVESPLIT_1

    def is_fivesplit2(self):
        '''Returns True if the node color matches FIVESPLIT_2'''
        return self.color == colors.FIVESPLIT_2

    def is_fivesplit3(self):
        '''Returns True if the node color matches FIVESPLIT_3'''
        return self.color == colors.FIVESPLIT_3

    def is_fivesplit4(self):
        '''Returns True if the node color matches FIVESPLIT_4'''
        return self.color == colors.FIVESPLIT_4
    
    def is_start(self):
        '''Returns True if the node is the start position (Node color is orange)'''
        return self.color == colors.ORANGE
    
    def is_end(self):
        '''Returns True if the node is the end position (Node color is turquoise)'''
        return self.color == colors.TURQUOISE
    
    def is_path(self):
        '''Returns True if the node is part of the final reconstructed path (Node color is purple)'''
        return self.color == colors.PURPLE
    
    def reset(self):
        """Resets the node to its default untraversed state (color=WHITE, extra_cost=0, node_type=untraversed)"""
        self.color = colors.WHITE
        self.extra_cost = 0
        self.node_type = "untraversed"

    def set_closed(self):
        """Marks the node as closed and fully explored (color=RED, node_type=traversed)"""
        self.color = colors.RED
        self.node_type = "traversed"

    def set_open(self):
        """Marks the node as open and discovered, but not fully explored (color=GREEN, node_type=traversed)"""
        self.color = colors.GREEN
        self.node_type = "traversed"
    
    def set_barrier(self):
        """Marks the node as an impassable barrier (color=BLACK, node_type=barrier)"""
        self.color = colors.BLACK
        self.node_type = "barrier"

    def set_fivesplit1(self):
        """Marks the node as low-cost weighted terrain (color=FIVESPLIT_1, extra_cost=1, node_type=untraversed)"""
        self.color = colors.FIVESPLIT_1
        self.extra_cost = 1
        self.node_type = "untraversed"
    
    def set_fivesplit2(self):
        """Marks the node as moderately weighted terrain (color=FIVESPLIT_2, extra_cost=2, node_type=untraversed)"""
        self.color = colors.FIVESPLIT_2
        self.extra_cost = 2
        self.node_type = "untraversed"

    def set_fivesplit3(self):
        """Marks the node as high-cost weighted terrain (color=FIVESPLIT_3, extra_cost=3, node_type=untraversed)"""
        self.color = colors.FIVESPLIT_3
        self.extra_cost = 3
        self.node_type = "untraversed"

    def set_fivesplit4(self):
        """Marks the node as very high-cost weighted terrain (color=FIVESPLIT_4, extra_cost=4, node_type=untraversed)"""
        self.color = colors.FIVESPLIT_4
        self.extra_cost = 4
        self.node_type = "untraversed"

    def set_start(self):
        """Marks the node as the start of the path (color=ORANGE, node_type=path)"""
        self.color = colors.ORANGE
        self.node_type = "path"
    
    def set_end(self):
        """Marks the node as the end of the path (color=TURQUOISE, node_type=path)"""
        self.color = colors.TURQUOISE
        self.node_type = "path"

    def set_path(self):
        """Marks the node as part of the final reconstructed path (color=PURPLE, node_type=path)"""
        self.color = colors.PURPLE
        self.node_type = "path"

    def draw(self, win):
        """
        Draws the node as a colored square on the given pygame surface
        
        :param win: Pygame surface to draw on
        """
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid, heuristic):
        """
        Updates the list of valid neighboring nodes based on movement rules.

        Cardinal movement is always allowed. Diagonal movement is allowed
        unless using the Manhattan heuristic, and is blocked if it would
        cut through a barrier corner.
        
        :param grid: The nxn matrix of nodes
        :param heuristic: Heuristic being used (either 'manhattan', 'euclidean', or 'octile')
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

    def __lt__(self, other):
        """Dummy comparison function required for PriorityQueue compatibility."""
        return False