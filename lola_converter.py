#!/usr/bin/env python3
import rasterio
import numpy as np
from PIL import Image
from matplotlib.colors import LightSource
import matplotlib.pyplot as plt
import argparse
import os

# Define the target directories for inputs and outputs
INPUT_DIR = "dems"
OUTPUT_DIR = "lola_conversions"

def convert_lola_tif(input_tif, output_filename, mode='ldem', cmap='gray', azimuth=315, altitude=45, vert_exag=1.0, show_hist=False, user_min=None, user_max=None):
    """
    Converts 32-bit float NASA LOLA .tif files into usable .png maps.
    Supports absolute min/max clipping to lock color scales to physical measurements.
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    safe_filename = os.path.basename(output_filename)
    final_output_path = os.path.join(OUTPUT_DIR, safe_filename)

    print(f"Reading {input_tif}...")
    
    with rasterio.open(input_tif) as src:
        data = src.read(1)
        nodata = src.nodata
        dx, dy = src.res 

    # 1. Mask out "NoData" areas
    if nodata is not None:
        mask = (data == nodata) | np.isnan(data)
    else:
        mask = np.isnan(data)
        
    valid_data = np.ma.masked_array(data, mask)
    raw_flat_data = valid_data.compressed()

    # Extract the raw 1D float array for the pre-conversion histogram (ignoring empty space)
    raw_flat_data = valid_data.compressed()

    # ---> NEW: Count and print the number of unique values
    unique_values = np.unique(raw_flat_data)
    print(f"Total distinct values in raw data: {len(unique_values):,}")

    # 2. Process based on the selected mode
    if mode == 'ldem':
        # Apply absolute bounds if provided, otherwise default to 1st/99th percentiles
        calc_min = user_min if user_min is not None else np.percentile(raw_flat_data, 1)
        calc_max = user_max if user_max is not None else np.percentile(raw_flat_data, 99)
        
        print(f"Normalizing LDEM elevation data (Colormap: {cmap})...")
        print(f"Data Range Locked to: {calc_min:.2f} -> {calc_max:.2f}")
        
        clipped_data = np.clip(valid_data, calc_min, calc_max)
        
        # Normalize to 0.0 - 1.0 for the colormap
        normalized = (clipped_data - calc_min) / (calc_max - calc_min)
        
        # Apply the colormap
        colormap = plt.get_cmap(cmap)
        rgba_data = colormap(normalized.filled(0))
        rgba_data[mask] = [0, 0, 0, 1] 
        
        img_array = (rgba_data * 255).astype(np.uint8)
        img_mode = 'RGBA'

    elif mode == 'slope':
        # Allow overriding the 0-30 degree default slope bounds too!
        calc_min = user_min if user_min is not None else 0.0
        calc_max = user_max if user_max is not None else 30.0
        
        print(f"Converting Slope data to visual gradient (Colormap: {cmap})...")
        print(f"Data Range Locked to: {calc_min:.2f} -> {calc_max:.2f} degrees")
        
        clipped_slope = np.clip(valid_data, calc_min, calc_max)
        
        # 1.0 is white (low slope), 0.0 is black (high slope)
        normalized = 1.0 - ((clipped_slope - calc_min) / (calc_max - calc_min))
        
        colormap = plt.get_cmap(cmap)
        rgba_data = colormap(normalized.filled(0))
        rgba_data[mask] = [0, 0, 0, 1]
        
        img_array = (rgba_data * 255).astype(np.uint8)
        img_mode = 'RGBA'
        
    elif mode == 'hillshade':
        print(f"Rendering Hillshade (Azimuth: {azimuth}°, Altitude: {altitude}°)...")
        ls = LightSource(azdeg=azimuth, altdeg=altitude)
        shaded = ls.hillshade(valid_data, vert_exag=vert_exag, dx=dx, dy=dy, fraction=1.0)
        img_array = (shaded * 255).astype(np.uint8)
        img_array[mask] = 0
        img_mode = 'L'

    # 3. Generate Histograms if requested
    if show_hist:
        print("Generating distribution histograms...")
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Pre-conversion Plot
        ax1.hist(raw_flat_data, bins=100, color='royalblue', alpha=0.8)
        ax1.set_title(f"Pre-Conversion: Raw {mode.upper()} Data (32-bit Float)")
        ax1.set_xlabel("True Value (Meters / Degrees)")
        ax1.set_ylabel("Pixel Count")
        ax1.grid(True, alpha=0.3)
        
        # Highlight the custom clipping range on the raw histogram
        if mode in ['ldem', 'slope']:
            ax1.axvline(calc_min, color='red', linestyle='dashed', linewidth=1.5, label='Clip Min')
            ax1.axvline(calc_max, color='orange', linestyle='dashed', linewidth=1.5, label='Clip Max')
            ax1.legend()
        
        # Post-conversion Plot
        if mode in ['ldem', 'slope']:
            final_flat_data = (normalized[~mask] * 255).astype(np.uint8)
        else:
            final_flat_data = img_array[~mask] 
            
        ax2.hist(final_flat_data, bins=50, color='seagreen', alpha=0.8)
        ax2.set_title("Post-Conversion: Clamped Math Data (0-255 Range)")
        ax2.set_xlabel("Data Bin")
        ax2.set_ylabel("Pixel Count")
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        base_name = os.path.splitext(safe_filename)[0]
        hist_path = os.path.join(OUTPUT_DIR, f"{base_name}_histogram.png")
        plt.savefig(hist_path)
        print(f"SUCCESS: Saved histogram to {hist_path}")
        plt.close()

    # 4. Save the final image
    img = Image.fromarray(img_array, mode=img_mode) 
    img.save(final_output_path)
    print(f"SUCCESS: Saved map to {final_output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NASA LOLA .tif to .png Converter")
    parser.add_argument("input", type=str, help="Name of the input .tif file (must be in /dems)")
    parser.add_argument("output", type=str, help="Name of the output .png file (will be saved in /lola_conversions)")
    parser.add_argument("--mode", type=str, choices=['ldem', 'slope', 'hillshade'], default='ldem', 
                        help="Data type: 'ldem' (heightmap), 'slope' (cost map), 'hillshade' (visual map).")
    
    parser.add_argument("-c", "--cmap", type=str, default='gray', help="Matplotlib colormap.")
    
    # Custom data ranges
    parser.add_argument("--min", type=float, default=None, help="Absolute minimum data value to clip to.")
    parser.add_argument("--max", type=float, default=None, help="Absolute maximum data value to clip to.")
    
    parser.add_argument("-a", "--azimuth", type=float, default=315)
    parser.add_argument("-e", "--elevation", type=float, default=45)
    parser.add_argument("-z", "--zoom", type=float, default=2.0)
    parser.add_argument("--histogram", action="store_true")
    
    args = parser.parse_args()
    
    safe_input_name = os.path.basename(args.input)
    input_path = os.path.join(INPUT_DIR, safe_input_name)
    
    if not os.path.exists(input_path):
        print(f"ERR: Input file not found -> {input_path}")
    else:
        convert_lola_tif(input_path, args.output, args.mode, args.cmap, args.azimuth, args.elevation, args.zoom, args.histogram, args.min, args.max)