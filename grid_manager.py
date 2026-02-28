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
        """
        self.rows = rows
        self.width = width
        self.win = win
        self.map_img = map_img
        self.grid = []
        
        self.show_search = True 
        
        # Guideline state tracking
        self.show_guideline = True
        self.global_route = []
        
        self.bg_surface = None
        if self.map_img:
            mode = self.map_img.mode
            size = self.map_img.size
            data = self.map_img.tobytes()
            surface = pygame.image.fromstring(data, size, mode)
            self.bg_surface = pygame.transform.scale(surface, (self.width, self.width))
        
        self.make_grid()

    def toggle_search_area(self):
        """
        Toggle all A* search artifacts (red/green) on or off.
        Then, update their displayed color.
        """
        self.show_search = not self.show_search
        
        for row in self.grid:
            for node in row:
                node.update_color(show_search=self.show_search)

    def toggle_guideline(self):
        """Toggle the guideline when observing the dynamic map."""
        self.show_guideline = not self.show_guideline

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
        """
        # 1. Draw the background surface if it exists
        if self.bg_surface:
            self.win.blit(self.bg_surface, (0, 0))
        else:
            self.win.fill(colors.WHITE)

        # 2. Draw each node
        for row in self.grid:
            for node in row:
                node.draw(self.win)

        # 3. Draw the gridlines overlay
        self.draw_gridlines()

        # 4. Draw the grid coordinates and selected brush near the mouse cursor
        mouse_x, mouse_y = pygame.mouse.get_pos()
        if 0 <= mouse_x < self.width and 0 <= mouse_y < self.width:
            row, col = self.get_clicked_pos((mouse_x, mouse_y))
            
            # Fetch the current brush state (defaulting to Barrier if not set)
            brush_type = getattr(self, 'selected_brush', 'Barrier (5)')
            coord_text = f"[{row}, {col}] | Brush: {brush_type}"
            
            font = pygame.font.SysFont(None, 24)
            text_surf = font.render(coord_text, True, (0, 0, 0))

            # Dynamically shift the text left based on its exact width to prevent clipping
            x_offset = -(text_surf.get_width() + 10) if row >= self.rows / 2 else 15
            y_offset = -20 if col >= self.rows / 2 else 15
            
            text_rect = text_surf.get_rect(topleft=(mouse_x + x_offset, mouse_y + y_offset))
            
            # Optional UI Polish: Draw a subtle white background box so text is readable over dark terrain
            bg_rect = text_rect.copy()
            bg_rect.inflate_ip(6, 4) # Add 3px padding on sides, 2px on top/bottom
            pygame.draw.rect(self.win, (255, 255, 255), bg_rect)
            pygame.draw.rect(self.win, (0, 0, 0), bg_rect, 1) # Add a 1px black border
            
            self.win.blit(text_surf, text_rect)
            
        # 5. Draw the guideline
        if getattr(self, 'show_guideline', False) and getattr(self, 'global_route', None) and len(self.global_route) > 1:
            pygame.draw.lines(self.win, (255, 215, 0), False, self.global_route, 4)
        
        # 6. Finally, update the display
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
                    if node.is_start() or node.is_end():
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