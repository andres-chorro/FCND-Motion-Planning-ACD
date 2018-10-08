## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

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

self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
Once global home is set, I can get the local position relative to it by using `global_to_local`:
`initial_local_pos = global_to_local(initial_global_pos, self.global_home)`

#### 3. Set grid start position from local position
I got the local grid position by rounding the local position, casting it to an integer, and subtracking the offset values:

```python
grid_start = (int(round(self.local_position[0] - north_offset)),
              int(round(self.local_position[1] - east_offset)))
```

#### 4. Set grid goal position from geodetic coords
I did this by using global_to_local against the global home, and then following the same procedure to get the grid values as I did for the start position: round, cast, offset.

```python
# Set goal as some arbitrary position on the grid
lat_goal = 37.794173
lon_goal = -122.399278
goal_position = [lon_goal, lat_goal, 0]

# adapt to set goal as latitude / longitude position and convert
local_goal = global_to_local(goal_position, self.global_home)
grid_goal = (int(round(local_goal[0] - north_offset)),
             int(round(local_goal[1] - east_offset)))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Here I first saved an approximation of sqrt(2) as a global constant:
`ROOT_TWO = 1.4142857`
Then I added 4 values to the Actions enum for each diagonal direction:
```python
NORTHEAST = (-1, 1, ROOT_TWO)
NORTHWEST = (-1, -1, ROOT_TWO)
SOUTHEAST = (1, 1, ROOT_TWO)
SOUTHWEST = (1, -1, ROOT_TWO)
```
Finally `valid_actions` had to be modified to eliminate the directions if they're invalid. I followed the structure of what was already done for the other four directions:
```python
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
```

#### 6. Cull waypoints 
Here I used the simplest 2D colinearity test for integers, using 
arithmetic to calculate the determinate. I added a few functions to `planning_utils.py`:
```python
def prune_path(path):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]

        if collinearity_int(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def collinearity_int(p1, p2, p3): 
    determinant = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    if determinant == 0:
        return True
    else:
        return False
```

And called `prune_path()` on the path immediately after A*:
```python
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
path = prune_path(path)
```

Here's a picture a test flightpath:
![Path Image](./path.png)