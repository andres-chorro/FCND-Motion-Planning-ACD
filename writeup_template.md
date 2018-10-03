## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes using `A*` to navigate a grid to a hard-coded location.

`motion_planning.py` comes with an implementation of an event-driven state machine that handles all the transitions for arming, takeoff, and navigating to waypoints. The `plan_path()` function sends a list of waypoints to the state machine to execute.

`planning_utils.py` comes with some helper fuctions to help with the planning of a path.
* `create_grid()` creates a grid of the 2D configuration space at a certain altitude. In this grid, '0's represent valid positions for the drone while '1's represend invalid positions.
* `Action` enum represents the four actions the drone can take in the grid. This will need to be expanded to allow diagonal movement.
* `valid_actions()` returns a list of the valid actions the drone can take from a give position in the grid. Again, this will need to be modified for diagonal movement.
* `a_star()` is a basic implementation of `A*` in a grid.
* `heuristic()` gives a simple euclidean distance cost from a position to the goal.



### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I did this by reading the first line using standard file `open()`, and then by splitting first at the comma, and then at the space. Casting the second element of the resulting strings to floats gives the result:
```python
with open('colliders.csv') as f:
    first_line = f.readline()
coord_strings = first_line.split(',')
lat0, lon0 = [float(x.split()[1]) for x in coord_strings]
```

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


