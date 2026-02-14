"""
Utility class for rendering and interacting with the grid-based pathfinding
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

class Grid:
    def __init__(self, rows, width, win, map_img=None):
        """
        Initialize the Grid class with required attributes.
        
        :param rows: Number of rows/columns in the grid
        :param width: Width/height of the pygame window being rendered to
        :param win: The pygame window surface
        :param map_img: Optional PIL image object of the map to load
        """
        self.rows = rows
        self.width = width
        self.win = win
        self.map_img = map_img
        self.grid = []
        
        # Generate the grid immediately upon instantiation
        self.make_grid()

    def toggle_search_area(self):
        """
        Toggle all 'traversed' nodes between their 'searched' colors (red/green) 
        and the original map's colors.
        """
        for row in self.grid:
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

    def make_grid(self):
        """
        Creates an nxn matrix of nodes and assigns it to self.grid.
        """
        self.grid = []
        gap = self.width // self.rows  # Width/height of each node being rendered
        for i in range(self.rows):
            self.grid.append([])
            for j in range(self.rows):
                node = Node(i, j, gap, self.rows)
                self.grid[i].append(node)

    def draw_gridlines(self):
        """
        Draws horizontal/vertical lines across the whole window, defining the grid.
        """
        gap = self.width // self.rows  # Width/height of each node being rendered
        for i in range(self.rows):
            pygame.draw.line(self.win, colors.GREY, (0, i * gap), (self.width, i * gap))
            for j in range(self.rows):
                pygame.draw.line(self.win, colors.GREY, (j * gap, 0), (j * gap, self.width))

    def draw(self):
        """
        Master function to draw each frame in the window.

        Layers each aspect on top of each other:
        White base -> Nodes -> Gridlines -> Coordinate text
        """
        # Initially reset the entire window to white
        self.win.fill(colors.WHITE)

        # Draw each node in the grid
        for row in self.grid:
            for node in row:
                node.draw(self.win)

        # Draw the gridlines on top of the nodes
        self.draw_gridlines()

        # Draw coordinates of hovered node
        mouse_x, mouse_y = pygame.mouse.get_pos()
        if 0 <= mouse_x < self.width and 0 <= mouse_y < self.width:
            row, col = self.get_clicked_pos((mouse_x, mouse_y))
            coord_text = f"({row}, {col})"
            font = pygame.font.SysFont(None, 24)
            text_surf = font.render(coord_text, True, (0, 0, 0))

            # Draw at different offset based on mouse x/y to prevent it rendering outside of the window
            x_offset = -50 if row >= self.rows / 2 else 15
            y_offset = -20 if col >= self.rows / 2 else 10
            text_rect = text_surf.get_rect(topleft=(mouse_x + x_offset, mouse_y + y_offset))
            self.win.blit(text_surf, text_rect)
        
        # Finally, render the updated window frame
        pygame.display.update()

    def get_clicked_pos(self, pos):
        """
        Get the row and column in the grid of the node that was just clicked
        
        :param pos: The y,x position of the cursor in the window provided by pygame
        """
        gap = self.width // self.rows
        y, x = pos
        row = y // gap
        col = x // gap
        return row, col

    def load_map(self):
        """
        Load in data from the pre-made map stored in self.map_img.
        Does nothing if self.map_img is not set.
        """
        if not self.map_img:
            return

        map_pixels = self.map_img.load()
        for y in range(self.map_img.height):
            for x in range(self.map_img.width):
                if map_pixels[x, y] == colors.BLACK:
                    self.grid[x][y].set_barrier() 
                elif map_pixels[x, y] == colors.FIVESPLIT_4:
                    self.grid[x][y].set_fivesplit4()
                elif map_pixels[x, y] == colors.FIVESPLIT_3:
                    self.grid[x][y].set_fivesplit3()
                elif map_pixels[x, y] == colors.FIVESPLIT_2:
                    self.grid[x][y].set_fivesplit2()
                elif map_pixels[x, y] == colors.FIVESPLIT_1:
                    self.grid[x][y].set_fivesplit1()

    def sensor_sweep(self, center_node, radius=1):
        """
        Simulate a rover's sensor sweep by reading the ground-truth image (self.map_img)
        within a given radius around the center_node and updating the grid nodes 
        to reflect the actual terrain.
        
        :param center_node: The Node object representing the rover's current position
        :param radius: How many nodes out the sensor can "see" (default 1 means a 3x3 area)
        """
        if not self.map_img:
            return

        map_pixels = self.map_img.load()
        cx, cy = center_node.row, center_node.col

        # Loop through a bounding box defined by the radius
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                x = cx + i
                y = cy + j

                # Check if x and y are within grid boundaries
                if 0 <= x < self.rows and 0 <= y < self.rows:
                    node = self.grid[x][y]
                    
                    # Prevent overwriting the rover's start/end markers
                    # (Assume start/end logic has a node.is_start() or node.is_end() equivalent; 
                    # otherwise, we check color or just proceed carefully)
                    if node.color == colors.ORANGE or node.color == colors.TURQUOISE:
                        continue
                        
                    pixel_color = map_pixels[x, y]
                    
                    if pixel_color == colors.BLACK:
                        node.set_barrier()
                    elif pixel_color == colors.FIVESPLIT_4:
                        node.set_fivesplit4()
                    elif pixel_color == colors.FIVESPLIT_3:
                        node.set_fivesplit3()
                    elif pixel_color == colors.FIVESPLIT_2:
                        node.set_fivesplit2()
                    elif pixel_color == colors.FIVESPLIT_1:
                        node.set_fivesplit1()