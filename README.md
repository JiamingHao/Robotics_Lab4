# COMS W4733 Robotics Lab 4

## Usage
#### 1. Extract lab4_UNI1_UNI2.tar.gz:


#### 2. Run demo.py with different command line arguments:
```shell
$ python demo.py
$ python demo.py --birrt
$ python demo.py --birrt --smoothing
```

## Methods

demo.py:
------
```python
def rrt(start_conf, goal_conf, step_size, collision_fn):
  """
  Implement the Rapidly-exploring Random Trees algorithm to
  find the path from start to goal.
  """
```
```python
def birrt(start_conf, goal_conf, step_size, collision_fn):
  """ 
  Implement the BiDirectional Rapidly-exploring Random Trees 
  algorithm to find the path from start to goal.
  """
```
```python
def birrt_smoothing(start_conf, goal_conf, step_size, collision_fn, N):
  """ 
  Implement the path smoothing algorithm on the path found by
  the BiDirectional Rapidly-exploring Random Trees algorithm.
  """
```
```python
def generate_random_point():
  """
  Randomly generate a new configuration point
  """
```
```python
def get_closest_neighbor(T, q):
  """
  Given the existing vertices of a tree and a new generated point,
  find the vertex that has the closest distance to the point.
  """
```
```python
def is_collision_free(q1, q2, n, collision_fn):
  """
  Recursively sample the midpoint segment q1q2 to check if there
  the path is collision free.
  """
```
```python
def add_edge(T, edge):
  """
  Add edge to the given tree.
  """
```
```python
def tree_dist(T1_T2):
  """
  Find the minmimum distance between Tree 1 and Tree 2, return
  the minimum distance and the pair of vertices producing this 
  distance.
  """
```
```python
def progress_along(q_near, q_rand, step_size):
  """
  Progress along the segment q_near q_rand using the step size,
  return a new vertex on the segment.
  """
```
```python
def draw_line(q_near, q_rand, color, width):
  """
  Draw a line between q_near and q_rand using the given color and 
  width.
  """
```
```python
def draw_solution_path(path_conf, color, width):
  """
  Display the final paths found between the start and the goal
  using the given color and width.
  """
```
## Video
![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/jAYv7HyTwEY/0.jpg)](https://www.youtube.com/watch?v=jAYv7HyTwEY)

![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/1LhXQazLctc/0.jpg)](https://www.youtube.com/watch?v=1LhXQazLctc)

![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/Rm8dTaxIRkk/0.jpg)](https://www.youtube.com/watch?v=Rm8dTaxIRkk)



