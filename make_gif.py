import os
import re
from PIL import Image
import numpy as np
import sys

def natural_sort_key(s):
    """
    Key function for natural sorting of filenames
    Allows correct ordering of files like image1.png, image2.png, image10.png
    """
    return [int(text) if text.isdigit() else text.lower() 
            for text in re.split(r'(\d+)', s)]

def png_to_gif(source_folder, target_path, duration=100, loop=0):
    """
    Convert a sequence of PNG files to a GIF
    
    Parameters:
    - source_folder: Path to folder containing PNG files
    - target_path: Full path (including filename) for output GIF
    - duration: Time between frames in milliseconds (default 100)
    - loop: Number of loops (0 = infinite)
    """
    # Validate input paths
    if not os.path.exists(source_folder):
        raise ValueError(f"Source folder does not exist: {source_folder}")
    
    # Create target directory if it doesn't exist
    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    
    # Get all PNG files, sorted naturally
    png_files = [f for f in os.listdir(source_folder) if f.lower().endswith('.png')]
    png_files.sort(key=natural_sort_key)
    
    # Full paths to PNG files
    full_paths = [os.path.join(source_folder, f) for f in png_files]
    
    # Validate file list
    if not png_files:
        raise ValueError(f"No PNG files found in {source_folder}")
    
    # Open images
    images = [Image.open(path) for path in full_paths]
    
    # Save as GIF
    images[0].save(
        target_path, 
        save_all=True, 
        append_images=images[1:], 
        duration=duration, 
        loop=loop
    )
    
    print(f"GIF created: {target_path}")
    print(f"Number of frames: {len(images)}")

# Example usage
if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Usage: python make_gif.py <source_folder> <target_gif> [duration] [loop]")
        sys.exit(1)

    source_folder = sys.argv[1]
    target_gif = sys.argv[2]
    duration = int(sys.argv[3]) if len(sys.argv) > 3 else 100
    loop = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    try:
        png_to_gif(source_folder, target_gif, duration=duration, loop=loop)
    except Exception as e:
        print(f"Error creating GIF: {e}")