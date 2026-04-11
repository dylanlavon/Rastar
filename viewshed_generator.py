#!/usr/bin/env python3

import os
import argparse
import math
import sys
from PIL import Image

def get_bresenham_line(x0, y0, x1, y1):
    """
    Generates the integer coordinates along a line from (x0, y0) to (x1, y1)
    using Bresenham's Line Algorithm.
    """
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
            
    points.append((x, y))
    return points

def main():
    parser = argparse.ArgumentParser(description="Generate a Comms Line-of-Sight (Viewshed) Map.")
    parser.add_argument("heightmap", type=str, help="Filename of the heightmap in /heightmaps")
    parser.add_argument("x", type=int, help="X coordinate of the Lander/Antenna")
    parser.add_argument("y", type=int, help="Y coordinate of the Lander/Antenna")
    parser.add_argument("--antenna", type=float, default=5.0, help="Height of the antenna above the terrain (Default: 5.0)")
    parser.add_argument("--z_factor", type=float, default=1.0, help="Vertical exaggeration to apply to terrain (Default: 1.0)")
    
    args = parser.parse_args()

    # --- Directory Setup ---
    input_dir = "heightmaps"
    output_dir = "viewshed_conversions"
    os.makedirs(output_dir, exist_ok=True)

    input_path = os.path.join(input_dir, args.heightmap)
    if not os.path.exists(input_path):
        parser.error(f"ERR: Could not find heightmap at {input_path}")

    # --- Load Image ---
    print(f"Loading heightmap: {args.heightmap}...")
    img = Image.open(input_path).convert('L')
    width, height = img.size
    pixels = img.load()

    if not (0 <= args.x < width and 0 <= args.y < height):
        parser.error(f"ERR: Antenna coordinates ({args.x}, {args.y}) are out of bounds for a {width}x{height} map.")

    # --- Setup Output Image ---
    viewshed_img = Image.new('L', (width, height), color=0) # Default to pure black (blocked)
    out_pixels = viewshed_img.load()

    # Calculate observer 3D position
    obs_z = (pixels[args.x, args.y] * args.z_factor) + args.antenna

    print(f"Generating viewshed from ({args.x}, {args.y}) with antenna height {args.antenna}...")

    # --- Core Viewshed Logic ---
    total_pixels = width * height
    processed = 0

    for ty in range(height):
        for tx in range(width):
            processed += 1
            
            # Progress Bar
            if processed % 10000 == 0:
                percent = (processed / total_pixels) * 100
                sys.stdout.write(f"\rProgress: [{percent:5.1f}%]")
                sys.stdout.flush()

            # The observer can always see themselves
            if tx == args.x and ty == args.y:
                out_pixels[tx, ty] = 255
                continue

            target_z = pixels[tx, ty] * args.z_factor
            dist_to_target = math.hypot(tx - args.x, ty - args.y)
            
            # The mathematical slope (rise over run) from the antenna to the target pixel
            target_slope = (target_z - obs_z) / dist_to_target

            line_points = get_bresenham_line(args.x, args.y, tx, ty)
            
            is_visible = True
            
            # Walk the line from the observer to the target
            # We skip the very first point (observer) and the very last point (target)
            for cx, cy in line_points[1:-1]:
                current_z = pixels[cx, cy] * args.z_factor
                dist_to_current = math.hypot(cx - args.x, cy - args.y)
                
                current_slope = (current_z - obs_z) / dist_to_current
                
                # If any terrain between the antenna and the target has a steeper slope,
                # the line of sight is blocked by that terrain.
                if current_slope >= target_slope:
                    is_visible = False
                    break 

            if is_visible:
                out_pixels[tx, ty] = 255 # Pure white (Clear LOS)
            else:
                out_pixels[tx, ty] = 0   # Pure black (Blocked LOS)

    print("\nProcessing complete.")

    # --- Save Output ---
    output_filename = f"viewshed_X{args.x}_Y{args.y}_A{int(args.antenna)}_{args.heightmap}"
    output_path = os.path.join(output_dir, output_filename)
    viewshed_img.save(output_path)
    print(f"Viewshed map successfully saved to: {output_path}")

if __name__ == "__main__":
    main()