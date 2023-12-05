import os
import sys

# Get the path of the jamming_avoidance scripts current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Construct the path to the parent director
parent_dir = os.path.dirname(current_dir)

# Construct the path to the jamming_detection directory
jamming_detection_dir = os.path.join(parent_dir, "/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/jamming")

# Add the jamming_detection directory to sys.path
sys.path.append(jamming_detection_dir)
