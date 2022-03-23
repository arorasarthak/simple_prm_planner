Path Planning Assignment
---------------------------

This is a solution to the path planning assignment in the form of a ros package. 

The package is implemented in python. The algorithm used is the Probabilistic Roadmap with Djikstra's search.

The `scripts` folder consists of four python programs:
- PRMPlanner.py : Implementation class for the PRM algorithms
- goal_node.py : A helper node to manage `/start` and `/goal` topics
- main.py : Main application program
- ogm_to_mat.py : Helper class to parse/process occupancy grid data from the `map_server` pkg.
 
# Usage
 `roslaunch planning_assignment planning_assignment.launch`
`roslaunch planning_assignment planning_assignment.launch map:=/path/to/map/yaml`

The solution uses rviz `2D Pose Estimate` and `2D Nav Goal` tools for obtaining start and goal points on the map. 