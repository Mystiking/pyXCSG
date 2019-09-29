#!/usr/bin/python3
import sys, os
import numpy as np
# Add pyigl to the path
from csgxml import parse_csg_graph
from utils import save_to_stl
import trimesh
import stl

FNAME = sys.argv[1]
# Step 0: Load in an XML file robot as CSG
csg_graph, last_name = parse_csg_graph(FNAME)
robot_name = last_name
VLast, FLast = csg_graph[robot_name]
# Face normals

## Vertices
V = np.array(VLast)
## Faces
F = np.array(FLast, dtype=np.int32)

save_to_stl(sys.argv[2], V, F)
