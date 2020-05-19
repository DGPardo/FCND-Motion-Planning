## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

### README


#### 1. `motion_planning.py` and `planning_utils.py`
These scripts contain a basic event driven planning implementation which is handled by the class `MotionPlanning`. It derives from `udacidrone.Drone` and it is capable of handling different flight plans as well as taking off, landing, arming and disarming the flying vehicle.
This class will be used to mainly command and control autonomously a drone.

`planning_utils.py` is a collection of helper functions which can be used as the building blocks of a planning algorithm.

#### 1. Set your global home position
In order to find out the coordinates of the origin of our local NED coordinate system, please refer to `colliders.py`.

The class `Colliders` implements a few methods to easily interact with the csv file provided. In order to parse what the
home latitude and longitude are, please refer to the method called `Colliders.get_reference_lat_lon()`. Here, regex is used to be a bit flexible with different headers in the future.

Any header satisfying `re.search('lat0 (.*\d+.\d+.*), lon0 (.*\d+.\d+.*)', first_line)` should produce a result otherwise
it would raise an error. There are not specific error classes nor error-handling as it is not the scope of this project.


This is picture of San Francisco centered around the colliders.csv `lat0` and `lon0`
![Map of SF](./misc/home.png)

#### 2. Set your current local position
"Home" will become the referenced `lat0` and `lon0` (obtained in the previous step) at zero altitude.
Using this point as reference (home) it is possible to determine our position relative to it in the local
NED system using the provided `global_to_local` function.

The code essentially looks like:
```
        latitude, longitude = self.colliders.get_reference_lat_lon()

        self.set_home_position(longitude=longitude, latitude=latitude, altitude=0)

        global_position = self.global_position

        local_position = global_to_local(global_position, self.global_home)
```

#### 3. Set grid start position from local position
In order to transfer local NED coordinates to grid indices, a new function within `planning_utils.py` called
`local_to_grid_coordinates` as well as `grid_to_local_coordinates` has been created. These methods take care of
the north/east offsets required to locate the drone within the grid of obstacles. They could be extended to handled
non-square unit size grids but it is not the scope of this project.

For this example case, the grid start is set to be the home location.

#### 4. Set grid goal position from geodetic coords
Once more, using `global_to_local` as well as the newly implemented `local_to_grid_coordinates` it is possible to find out
what the goal location is in grid indices. We have now got a grid of obstacles together with the start and end locations in grid coordinates.

#### 5.A. Modify A* to include diagonal motion (or replace A* altogether)
In order to handle diagonal motion, the class `Action` has been extended with new actions such as NORTH_EAST, NORTH_WEST etc..

The new actions are implemented as illustrated in the code block below. Note that I avoided calling sqrt(2) but no significant speed improvement is noted.

```
class Action(Enum):
...
    NORTH_EAST = (-1, +1, 1.41421356237)  #1.41421356237 = sqrt(2)
...
```

Also, a new static method has been included within `Action` in order to "add" actions and/or nodes as shown below
```
    @staticmethod
    def add(tuple1, tuple2):
        return tuple1[0] + tuple2[0], tuple1[1] + tuple2[1]
```

Last but not least, `valid_actions` is modified in order to handle not only the new actions but any action we wish to implement.
The final implementation iterates over all actions and checks whether an obstacles or grid dimensions could restrict the possible actions

```
def valid_actions(grid, current_node):
...
    for action in Action:
        i, j = Action.add(action.delta, current_node)
        if not (0 <= i < n and 0 <= j < m and grid[i, j] == 0):
            valid_actions.remove(action)
```
Notice that the order of the conditions matters as `grid[i,j]` could not be evaluated if the preceding conditions are not matched.

#### 5.B Search Implementation
In order to speed-up the planning problem, I explored the proposed random sampling technique explained in the lessons.
However it is not always perfect and it trades subsampling speed with a higher number of KDTree searches. An example of the random subsampling can be found in `random_grid_sampling`

A better distribution of points could be obtained by clustering cells without obstacles. This is implemented in `cluster_grid_sampling` (included in `planning_utils.py`) using
`sklearn.clusters.MiniBatchKMeans`. It produces a better distribution but is considerably slower than random subsampling.

An intermediate solution between KMeans Clustering and Random Sampling is implemented in `subsample_grid` (included in `planning_utils.py`)
This is what is used in the final implementation. It creates random grid indices and then checks the proximity to previously created grid indices using integer arithmetic only.
It also checks whether or not the indices belong to a cell that contains an obstacle.

```
def subsample_grid(grid, min_distance, max_iter=None, seed=0, max_points=1E4):
```

It is significantly faster than `MiniBatchKMeans` and covers the grid considerably better than Random Sampling.
The sampeld points are then added to a `networks` Graph. Finally, A* search is performed on this graph.

In order to find the connectivity between edges of the graph, we have limited the number of edges a node can be connected to.
This has accelerated the time it takes to build the graph and perform searches while not having a significant impact on the results

#### 6. Cull waypoints
Although I have implemented `prune_collinear_points` within `planning_utils.py`, I am happier with the results of something I called "line_of_sigth_smoothing"
It is a simple algorithm that mainly avoids unnecessary zigzagging.

1) For three points, it checks (employing Bresenham algorithm) if the 3rd point can be reached from the first without crashing with an obstacle.
2) If 1->3 route is possible, removes the point in the middle
3) if the "greedy" mode is passed as True (default is False), it restarts the algorithm from the starting node of the path. Otherwise it continues traversing the path

![](./example_routes_found/routes_gif.gif)

### Execute the flight
It works! I have been able to fly from multiple origins to multiple targets successfully. It is so nice to see this working nicely

Below there is a picture of the grid subsampling, graph edges and final route plan after "line_of_sight_smoothing"
As you can see, there are no nodes of the graph that are almost overlapping on top of each other.

