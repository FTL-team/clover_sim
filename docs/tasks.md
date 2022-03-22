# Tasks
## Tasks creation
To create a task:
1. Navigate to the `clover_sim/tasks` directory
2. Copy `example_task` folder and rename it
3. Change name in `package.xml` and in `CMakeLists.txt` in line `project(example_task)`

> Task name can contain only lowercase ASCII letters, numbers and underscores

4. Write a task based on example task

> After changing something in task you should change something in `package.xml` and restart clover_sim(task should be rebuilt)
## Randomizations

To import randomizations use this command:

```python
from cloversim.randomization import create_random_string, randfloat, randbool, create2d_postions
```
A little about each method:

- `create_random_string(length)` returns random string with specified length
- `randfloat(lowest_border, upper_border)` returns random float in range between `lowest_border` and `upper_border`
- `randbool(p)` returns random boolean, with chance p, this param is **optional**, default value is 0.5
- `create2d_postions(objs_radius, field_size, field_start=(0,0))` returns array of 2d coordinates, `objs_radius` is radius of circle circumscribed about objects(objects shouln'd intersect), `field_size` is array [X_size, Y_size], `field_start` is **optional**, default value is (0, 0)


## Task checker

To import randomizations use this command:

```python
from cloversim.checker import get_clover_position, is_clover_armed, is_clover_still, match_cordinates
```

A little about each method:

- `get_clover_position()` - returns object with copter position
- `is_clover_armed()` - returns boolean value is clover armed or not
- `is_clover_still(pos)` - returns is clover still at this point, `pos` argument is what will return `get_clover_position()`
- `match_cordinates(correct, prediction)` - returns array of boolean values(does points from correct array match with coords from user program predicted coords)