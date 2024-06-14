from os import sys, path

parent_dir = path.dirname(path.dirname(path.abspath(__file__)))
cbma_dir  = path.join(parent_dir, "../nats/cbma")
src_dir = path.join(parent_dir, "src")

sys.path.append(cbma_dir)
sys.path.append(src_dir)

# Make it possible to run this module also in other folder (e.g. /opt/nats):
sys.path.append("/opt/mesh_com/modules/sc-mesh-secure-deployment/src/nats/cbma")

