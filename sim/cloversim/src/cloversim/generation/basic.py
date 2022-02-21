from .validate import validate_pose, validate_size
import math


def pose_to_string(pose):
  if len(pose) == 3:
    pose = (pose[0], pose[1], pose[2], 0, 0, 0)
  return ' '.join([str(a) for a in pose])


class Include():

  def __init__(self, uri, name=None, pose=(0, 0, 0, 0, 0, 0)):
    if name is not None and name is not str:
      raise ValueError("Name must be a string or None")

    validate_pose(pose)

    self.uri = uri
    self.name = name
    self.pose = pose

  def xml(self):
    additional = ""
    if self.name is not None:
      additional += f"<name>{self.name}</name>\n"
    if self.pose is not None:
      additional += f"<pose>{pose_to_string(self.pose)}</pose>\n"
    return f"""
    <include>
        <uri>{self.uri}</uri>
        {additional}
    </include>
    """


class Model():

  def __init__(self, name, pose=(0, 0, 0, 0, 0, 0), mass=10, material=None, static=False):
    validate_pose(pose)
    if type(mass) is not float and type(mass) is not int:
      raise ValueError("Mass must be int or float")

    self.name = name
    self.pose = pose
    self.mass = float(mass)
    if type(material) is not tuple:
      material = (material, )
    self.material = material
    self.static = static

  def get_inertia(self):
    raise NotImplementedError("Model inertia is not implemented")

  def get_geometry(self):
    raise NotImplementedError("Model geometry is not implemented")

  def create_visual_links(self):
    raise NotImplementedError("Visual is not implemented")

  def xml(self):
    return f"""
    <model name="{self.name}">
      <pose>{pose_to_string(self.pose)}</pose>
      <static>{self.static}</static>
      <link name="link">
        <inertial>
          <mass>{self.mass}</mass>
          {self.get_inertia()}
        </inertial>
        <collision name="collision">
          {self.get_geometry()}
          <surface>
            <contact>
              <ode>
                <min_depth>0.01</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        {self.create_visual_links()}
      </link>
    </model>
    """


class Box(Model):

  def __init__(self,
               name,
               size=(1, 1, 1),
               pose=(0, 0, 0, 0, 0, 0),
               mass=10,
               material=None, static=False):
    if material is tuple and len(material) not in [3, 6]:
      raise ValueError(
          "Material must be single material or tuple of materials with length 3 or 6"
      )

    super().__init__(name, pose, mass, material, static)

    validate_size(size)
    self.size = size

  def get_inertia(self):
    w, h, d = self.size
    mass = self.mass

    return f"""
    <inertia>
      <ixx>{mass * (h * h + d * d) / 12}</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>{mass * (w * w + d * d) / 12}</iyy>
      <iyz>0</iyz>
      <izz>{mass * (w * w + h * h) / 12}</izz>
    </inertia>
    """

  def get_geometry(self):
    return f"""
    <geometry>
      <box>
        <size>{self.size[0]} {self.size[1]} {self.size[2]}</size>
      </box>
    </geometry>
    """

  def create_visual_links(self):
    if type(self.material) is not tuple:
      self.material = (self.material, )

    if len(self.material) == 1:
      return f"""
      <visual name="{self.name}_visual">
        {self.get_geometry()}
        {self.material[0].xml() if self.material[0] is not None else ""}
      </visual>
      """

    if len(self.material) == 3:
      material = (self.material[0], self.material[1], self.material[2],
                  self.material[0], self.material[1], self.material[2])
    else:
      material = self.material

    if len(material) == 6:
      current_visual = 0

      def create_plane(size, pos, material):
        nonlocal current_visual
        current_visual += 1
        return f"""
         <visual name="{self.name}_visual_{current_visual}">
          <geometry>
            <box>
              <size>{size[0]} {size[1]} {size[2]}</size>
            </box>
          </geometry>
          <pose>
            {' '.join([str(a) for a in pos])}
          </pose>
          {material.xml()}
        </visual>
        """

      visuals = []
      for current_plane in range(6):
        pos = [0, 0, 0, 0, 0, 0]
        size = list(self.size)

        coef = 1 if current_plane < 3 else -1
        c = current_plane % 3
        pos[c] = coef * self.size[c] / 2
        size[c] = 1e-4

        visuals.append(create_plane(size, pos, material[current_plane]))

      return ''.join(visuals)
    else:
      return super().get_visual_geometries()
