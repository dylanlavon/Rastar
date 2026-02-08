import pygame
import colors

class Node:
    def __init__(self, row, col, width, total_rows):
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
        return self.row, self.col
    
    def is_closed(self):
        return self.color == colors.RED
    
    def is_open(self):
        return self.color == colors.GREEN
    
    def is_barrier(self):
        return self.color == colors.BLACK
    
    def is_fivesplit1(self):
        return self.color == colors.FIVESPLIT_1

    def is_fivesplit2(self):
        return self.color == colors.FIVESPLIT_2

    def is_fivesplit3(self):
        return self.color == colors.FIVESPLIT_3

    def is_fivesplit4(self):
        return self.color == colors.FIVESPLIT_4
    
    def is_start(self):
        return self.color == colors.ORANGE
    
    def is_end(self):
        return self.color == colors.TURQUOISE
    
    def is_path(self):
        return self.color == colors.PURPLE
    
    def reset(self):
        self.color = colors.WHITE
        self.extra_cost = 0
        self.node_type = "untraversed"

    def set_closed(self):
        self.color = colors.RED
        self.node_type = "traversed"

    def set_open(self):
        self.color = colors.GREEN
        self.node_type = "traversed"
    
    def set_barrier(self):
        self.color = colors.BLACK
        self.node_type = "barrier"

    def set_fivesplit1(self):
        self.color = colors.FIVESPLIT_1
        self.extra_cost = 1
        self.node_type = "untraversed"
    
    def set_fivesplit2(self):
        self.color = colors.FIVESPLIT_2
        self.extra_cost = 2
        self.node_type = "untraversed"

    def set_fivesplit3(self):
        self.color = colors.FIVESPLIT_3
        self.extra_cost = 3
        self.node_type = "untraversed"

    def set_fivesplit4(self):
        self.color = colors.FIVESPLIT_4
        self.extra_cost = 4
        self.node_type = "untraversed"

    def set_start(self):
        self.color = colors.ORANGE
        self.node_type = "path"
    
    def set_end(self):
        self.color = colors.TURQUOISE
        self.node_type = "path"

    def set_path(self):
        self.color = colors.PURPLE
        self.node_type = "path"

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid, heuristic):
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