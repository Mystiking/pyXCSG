# pyXCSG

A tool for creating 3D models through CSG.
The tool relies on libigl for boolean operations.

## How to use the tool

Create an xml file of your robot, as in the example 'example.xml'.
Assuming the file is called 'robot.xml', then do
    `python3 inspect_mesh.py robot.xml`
to inspect the 3D model.
Once you are satisifed with the model, then do
    `python3 save_to_stl.py robot.xml output.stl`
to save the robot mesh to an stl file named 'output.stl'.
