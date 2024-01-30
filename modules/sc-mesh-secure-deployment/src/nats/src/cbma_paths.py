from os import sys, path

parent_dir = path.dirname(path.dirname(path.abspath(__file__)))
ms20_features_dir= path.join(parent_dir, "../2_0/features")
cbma_dir  = path.join(parent_dir, "../2_0/features/cbma")
src_dir = path.join(parent_dir, "src")

sys.path.append(ms20_features_dir)
sys.path.append(cbma_dir)
sys.path.append(src_dir)

# Make it possible to run this module also in other folder (e.g. /opt/nats):
sys.path.append("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features")
sys.path.append("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/cbma")
