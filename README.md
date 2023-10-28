# PoC
This repo contains the MATLAB files for the proof of concept for the final year project, human-to-robot handovers using 5 DoF robot arm

Current tasks:
- Split into two modes of approach and grasp
- Get real data from a gyroscope and use it to generate a trajectory for the simulation arm to attempt to grasp
- Add 3d model to represent the robot arm for visualization purposes
- start from a random trajec and then try to follow then path of the object being handed (make sure to visualize the path of the object) [add a movement constraint (cant move more than theta degrees per sec)]
- add more visulisations

Completed tasks:
- Add orientation constraint
- Increase the size of frames (alternate solution of adding red dots to frames edges is done to improve visibility)
- Have the IK solver
- Can track a trajectory given the trajectory's position in x,y,z
- Visible robot arm
