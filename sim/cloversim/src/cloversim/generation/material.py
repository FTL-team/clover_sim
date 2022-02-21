def validate_color(color):
  if type(color) is not tuple:
    raise ValueError("color must be a tuple of 3 or 4 floats from 0 to 1")

  if len(color) not in [3, 4]:
    raise ValueError("color must be a tuple of 3 or 4 floats from 0 to 1")
  for c in color:
    if type(c) is not float and type(c) is not int:
      raise ValueError("color must be a tuple of 3 or 4 floats from 0 to 1")
    if c < 0 or c > 1:
      raise ValueError("color must be a tuple of 3 or 4 floats from 0 to 1")


def add_transparent_color(color):
  if len(color) == 3:
    return color + (1, )
  return color


def color_to_string(color):
  return ' '.join([str(a) for a in color])


class ColorMaterial:

  def __init__(self, ambient, diffuse=None):
    if diffuse is None:
      diffuse = ambient

    validate_color(ambient)
    validate_color(diffuse)

    ambient = add_transparent_color(ambient)
    diffuse = add_transparent_color(diffuse)

    self.ambient = ambient
    self.diffuse = diffuse
    self.specular = (0, 0, 0, 0)
    self.emissive = (0, 0, 0, 1)

  def xml(self):
    return f"""
    <material>
      <ambient>{color_to_string(self.ambient)}</ambient>
      <diffuse>{color_to_string(self.diffuse)}</diffuse>
      <specular>{color_to_string(self.specular)}</specular>
      <emissive>{color_to_string(self.emissive)}</emissive>
    </material>
    """

class OgreMaterial:
  def __init__(self, script_uri, script_name):
    self.script_uri = script_uri
    self.script_name = script_name


  def xml(self):
    return f"""
    <material>
      <script>
        <uri>{self.script_uri}</uri>
        <name>{self.script_name}</name>
      </script>
    </material>
    """