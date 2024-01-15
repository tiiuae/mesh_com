import os
import sys

# Get the path to dir containing current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Construct the path to the cbma directory
cbma_dir = os.path.join(current_dir, "../../features/cbma")

# Add the cbma dir to sys.path
sys.path.insert(0, cbma_dir)