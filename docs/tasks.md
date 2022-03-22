# Tasks
## Tasks creation
To create a task:
1. Navigate to the `clover_sim/tasks` directory
2. Copy `example_task` folder and rename it
3. Change name in `package.xml` and in `CMakeLists.txt` in line `project(example_task)`

> Task name can contain only lowercase ASCII letters, numbers and underscores

4. Write a task based on example task

> After changes in task you should change something in `package.xml`(for example just add extra space) and restart clover_sim(task should be rebuilt)

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


## World creation

- Import necessary functions:

```python
from cloversim.generation import World, Include, Box, Cylinder
from cloversim.generation import ColorMaterial, ImageTextures, ArucoMap, generate_aruco_map
```

- Create new world object:

```python
WORLD = World()
```

- Add basic elements:

```python
# add sun
WORLD.add(Include("model://sun"))                        
# add plane                                    
WORLD.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))

# generate aruco_map object
aruco_map = generate_aruco_map()           
# add markers                                                     
aruco_map += generate_aruco_map(start_marker=101, pos=(5, 10, 0), number_of_markers=(1, 10))
# add aruco map
WORLD.add(ArucoMap("aruco_map", aruco_map).generate())                                  
```

- To add some colorfull objects you can use prepared code(just put it into `colors.py` file at the same directory):

```python
colors = {
  'black': (0, 0, 0), 
  'blue': (0, 0, 1), 
  'green': (0, 1, 0), 
  'cyan': (0, 1, 1), 
  'red': (1, 0, 0), 
  'magenta': (1, 0, 1),
  'yellow': (1, 1, 0), 
  'white': (1, 1, 1), 
  'gray': (0.5, 0.5, 0.5),
  'orange' : (1, 0.5, 0)
}

```

Import colors this way:

```python
from .colors import colors
```

Create object of this format `'red': ColorMaterial((1, 0, 0))`

```python
color_materials = {k: ColorMaterial(v) for k, v in colors.items()}
```

- Add boxes

```python
# Apply one color to the hole object
WORLD.add(
    Box("color_box",
        size=(0.32, 0.32, 0.08),
        mass=1,
        pose=(5, 11.5, 0.05),
        material=color_materials["red"], 
        static=True
    )
)


# Apply different colors to the hole object
WORLD.add(
    Box(
        "multi_color_box",
        size=(1, 2, 0.5),
        pose=(6, random_object_posB, 0.25),
        material=(tuple(list(color_materials.values())[1:7])),
    )
)
```

- Add QR code:

```python
import qrcode
qrcode_texture = qrcode.make("Information you want to encode in QR code").get_image()
WORLD.add(
    Box("qrcode",
        size=(1, 1, 0.001),
        pose=(5, 20, 0.001),
        mass=0.1,
        material = qrcode_texture,
        static=True
    )
)
```

- Add cylinder:
```python
WORLD.add(
    Cylinder("cylinder",
            radius=0.4,
            length=0.3,
            pose=(marker_positions[0][0], marker_positions[0][1], 0.15),
            material=(
                ColorMaterial((0.5, 0.5, 0.5)),
                ColorMaterial((1, 0.5, 0)),
                ColorMaterial((1, 1, 0)),
        )
    )
)
```