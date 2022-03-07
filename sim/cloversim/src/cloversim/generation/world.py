from ..task import WORLD_PATH


class World:
  SCENE_OPTIONS = """
  <scene>
    <ambient>0.8 0.8 0.8 1</ambient>
    <background>0.8 0.9 1 1</background>
    <shadows>false</shadows>
    <grid>false</grid>
    <origin_visual>false</origin_visual>
  </scene>
  """

  PHYSICS_OPTIONS = """
  <physics name='default_physics' default='0' type='ode'>
    <gravity>0 0 -9.8066</gravity>
    <ode>
      <solver>
        <type>quick</type>
        <iters>10</iters>
        <sor>1.3</sor>
        <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
      </solver>
      <constraints>
        <cfm>0</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
    <max_step_size>0.004</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>250</real_time_update_rate>
    <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
  </physics>
  """

  ADDITIONAL_XML = ""

  def save(self):
    contents = f"""
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    {self.ADDITIONAL_XML}
    {self.SCENE_OPTIONS}
    {self.PHYSICS_OPTIONS}
  </world>
</sdf>
"""
    f = open(WORLD_PATH, "w")
    f.write(contents)
    f.close()

  def add(self, obj):
    if obj.xml is None:
      raise ValueError("Invalid object, it must have an xml callable function")
    self.ADDITIONAL_XML += obj.xml()