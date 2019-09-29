#!/usr/bin/python3
import sys, os
import numpy as np
# Add pyigl to the path
PATH_TO_LIBIGL = '/home/max/diku/2018/project2018b2/SoftRoboticDesign/robot_designs/libigl/python'
sys.path.insert(0, PATH_TO_LIBIGL)
import pyigl as igl
from csgxml import parse_csg_graph
from utils import save_to_stl
import trimesh
import stl

if len(sys.argv) < 2:
    print("Usage: verify_feasibility.py [filename] (Optional: robot name in XML file)")
    exit()

FNAME = sys.argv[1]
# Step 0: Load in an XML file robot as CSG
csg_graph, last_name = parse_csg_graph(FNAME)
robot_name = last_name
VLast, FLast = csg_graph[robot_name]
# Face normals
FN = igl.eigen.MatrixXd()
igl.per_face_normals(VLast, FLast, FN)

## Vertices
V = np.array(VLast)
## Faces
F = np.array(FLast, dtype=np.int32)
## Face Normals
FN = np.array(FN)
## Making the mesh
robot = trimesh.base.Trimesh(V, F, FN)

# Step 1: Show the robot design to the user (using libigl to show triangle mesh)
robot.show()
