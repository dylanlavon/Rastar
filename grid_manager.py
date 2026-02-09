"""
Utility functions for rendering and interacting with the grid-based pathfinding
visualization using Pygame.

This module is responsible for:
- Creating and initializing the grid of Node objects
- Drawing nodes, gridlines, and per-frame UI elements
- Translating mouse input into grid coordinates
- Toggling visualization layers for explored/search states
- Loading terrain and obstacle data from pre-defined map images

These functions contain no pathfinding logic; they strictly support visualization,
user interaction, and grid state management for the A* algorithm.
"""

# External dependencies
import pygame

# Internal dependencies
from node import Node
import colors

def toggle_search_area(grid):
    """
    Toggle all 'traversed' nodes between their 'searched' colors (red/green) and the original map's colors.
    
    :param grid: Grid of nodes in which to toggle
    """
    for row in grid:
        for node in row:
            if node.node_type == "traversed":
                if node.color == colors.GREEN or node.color == colors.RED:
                    node.prev_color = node.color
                    if node.extra_cost == 0:
                        node.color = colors.WHITE
                    elif node.extra_cost == 1:
                        node.color = colors.FIVESPLIT_1
                    elif node.extra_cost == 2:
                        node.color = colors.FIVESPLIT_2
                    elif node.extra_cost == 3:
                        node.color = colors.FIVESPLIT_3
                    elif node.extra_cost == 4:
                        node.color = colors.FIVESPLIT_4
                else:
                    node.color = node.prev_color

def make_grid(rows, width):
    """
    Creates an nxn matrix of nodes
    
    :param rows: Number of rows/columns in the grid
    :param width: Width/height of the pygame window being rendered to
    """
    grid = []
    gap = width // rows # Width/height of each node being rendered
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid

def draw_gridlines(win, rows, width):
    """
    Draws horizontal/vertical lines across whole window, defining the grid.
    
    :param win: The pygame window in which to draw lines
    :param rows: Number of rows/columns of nodes in the grid
    :param width: Width/height of the pygame window being rendered to
    """
    gap = width // rows # Width/height of each node being rendered
    for i in range(rows):
        pygame.draw.line(win, colors.GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, colors.GREY, (j * gap, 0), (j * gap, width))

def draw(win, grid, rows, width):
    """
    Master function to draw each frame in the window.

    Layers each aspect on top of each other.

    White base -> Nodes -> Gridlines -> Coordinate text
    
    :param win: The pygame window in which to draw
    :param grid: The nxn matrix of nodes 
    :param rows: Number of rows of nodes in the grid
    :param width: Width/height of the pygame window being rendered to
    """
    # Initially reset the entire window to white
    win.fill(colors.WHITE)

    # Draw each node in the grid
    for row in grid:
        for node in row:
            node.draw(win)

    # Draw the gridlines on top of the nodes
    draw_gridlines(win, rows, width)

    # Draw coordinates of hovered node
    mouse_x, mouse_y = pygame.mouse.get_pos()
    if 0 <= mouse_x < width and 0 <= mouse_y < width:
        row, col = get_clicked_pos((mouse_x, mouse_y), rows, width)
        coord_text = f"({row}, {col})"
        font = pygame.font.SysFont(None, 24)
        text_surf = font.render(coord_text, True, (0, 0, 0))

        # Draw at different offset based on mouse x/y to prevent it rendering outside of the window
        x_offset = -50 if row >= rows / 2 else 15
        y_offset = -20 if col >= rows / 2 else 10
        text_rect = text_surf.get_rect(topleft=(mouse_x + x_offset, mouse_y + y_offset))
        win.blit(text_surf, text_rect)
    
    # Finally, render the updated window frame
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    """
    Get the row and column in the grid of the node that was just clicked
    
    :param pos: The y,x position of the cursor in the window provided by pygame
    :param rows: Number of rows of nodes in the grid
    :param width: Width/height of the pygame window being rendered to
    """
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col

def load_map(grid, map_img):
    """
    Load in data from a pre-made map
    
    :param grid: The nxn matrix of nodes
    :param map_img: PIL image object of the map
    """
    map_pixels = map_img.load()
    for y in range(map_img.height):
        for x in range(map_img.width):
            if map_pixels[x,y] == colors.BLACK:
                grid[x][y].set_barrier() 
            elif map_pixels[x,y] == colors.FIVESPLIT_4:
                grid[x][y].set_fivesplit4()
            elif map_pixels[x,y] == colors.FIVESPLIT_3:
                grid[x][y].set_fivesplit3()
            elif map_pixels[x,y] == colors.FIVESPLIT_2:
                grid[x][y].set_fivesplit2()
            elif map_pixels[x,y] == colors.FIVESPLIT_1:
                grid[x][y].set_fivesplit1()