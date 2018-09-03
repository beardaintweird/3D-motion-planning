# FCND - 3D Motion Planning - Samee Khan submission

## Step by step breakdown of rubric requirements

### README
This README titled 'README_SUBMISSION'.

### Explain the starter code
The motion_planning.py script defines its namesake class and initializes values relevant to flight. The callbacks are largely unchanged from the backyard_flyer.py script.
Plan path changes the state of the drone's state machine. It creates a 2D configuration space out of the provided collider data. A sample start and goal are created to run a 2D version of A*. The points in the path are converted to match the drone's local position. Setting the waypoints allows the waypoint_transition method to function successfully. This method is called again at each waypoint until the final waypoint is reached.

### Implement planning algorithm
Each step is completed in the plan_path method.
- note: I chose to proceed with a graph representation of the search space.

### Executing the flight
The flights run successfully.
- Bug: Hitting obstacles on landing
-- Have a plan for fixing it, may adjust in a future resubmission


cheers :)
