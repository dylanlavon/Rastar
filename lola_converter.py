import rasterio
import numpy as np
from PIL import Image
from matplotlib.colors import LightSource
import argparse
import os

INPUT_DIR = "dems"
OUTPUT_DIR = "lola_conversions"

def convert_lola_tif(input_tif, output_filename, mode='ldem', azimuth=315, altitude=45, vert_exag=1.0):
    """
    Converts 32-bit float NASA LOLA .tif files into usable .png maps.
    Automatically routes outputs to the /lola_conversions directory.
    """
    # Ensure the input/output directories exist
    os.makedirs(INPUT_DIR, exist_ok=True)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Strip any extra folder paths the user might have accidentally typed
    # and lock the final output path into our dedicated directory
    safe_filename = os.path.basename(output_filename)
    final_output_path = os.path.join(OUTPUT_DIR, safe_filename)

    print(f"Reading {input_tif}...")
    
    with rasterio.open(input_tif) as src:
        data = src.read(1)
        nodata = src.nodata
        dx, dy = src.res # Get pixel resolution for accurate light bouncing

    # 1. Mask out "NoData" areas
    if nodata is not None:
        mask = (data == nodata) | np.isnan(data)
    else:
        mask = np.isnan(data)
        
    valid_data = np.ma.masked_array(data, mask)

    # 2. Process based on the selected mode
    if mode == 'ldem':
        print("Normalizing LDEM elevation data to grayscale...")
        min_val = valid_data.min()
        max_val = valid_data.max()
        normalized = ((valid_data - min_val) / (max_val - min_val) * 255)
        img_array = normalized.filled(0).astype(np.uint8)

    elif mode == 'slope':
        print("Converting Slope data to visual gradient...")
        capped_slope = np.clip(valid_data, 0, 30)
        normalized = 255 - ((capped_slope / 30.0) * 255)
        img_array = normalized.filled(0).astype(np.uint8)
        
    elif mode == 'hillshade':
        print(f"Rendering Hillshade (Azimuth: {azimuth}°, Altitude: {altitude}°)...")
        # Create a simulated sun
        ls = LightSource(azdeg=azimuth, altdeg=altitude)
        
        # Calculate shadows and highlights
        shaded = ls.hillshade(valid_data, vert_exag=vert_exag, dx=dx, dy=dy, fraction=1.0)
        
        # Convert the float output (0.0 to 1.0) to an 8-bit image (0 to 255)
        img_array = (shaded * 255).astype(np.uint8)
        
        # Handle the empty space mask
        img_array[mask] = 0

    # 3. Save the final image
    img = Image.fromarray(img_array, mode='L') 
    img.save(final_output_path)
    print(f"SUCCESS: Saved to {final_output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NASA LOLA .tif to .png Converter")
    parser.add_argument("input", type=str, help="Path to the input .tif file")
    parser.add_argument("output", type=str, help="Name of the output .png file (will be saved in /lola_conversions)")
    parser.add_argument("--mode", type=str, choices=['ldem', 'slope', 'hillshade'], default='ldem', 
                        help="Data type: 'ldem' (heightmap), 'slope' (cost map), 'hillshade' (visual map).")
    
    # Lighting arguments (only used if --mode hillshade is selected)
    parser.add_argument("-a", "--azimuth", type=float, default=315, help="Sun direction in degrees (default: 315 NW)")
    parser.add_argument("-e", "--elevation", type=float, default=45, help="Sun altitude in degrees (default: 45)")
    parser.add_argument("-z", "--zoom", type=float, default=2.0, help="Vertical exaggeration for deeper shadows (default: 2.0)")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"ERR: File not found -> {args.input}")
    else:
        convert_lola_tif(args.input, args.output, args.mode, args.azimuth, args.elevation, args.zoom)