import cv2
import numpy as np
from pytest import mark
from .image import ImageTextures

def validate_markers(markers):
  ERROR_MSG = "Markers must be a list of tuples of length 4 (id, x, y, z)"
  if not isinstance(markers, list):
    raise ValueError(ERROR_MSG)
  for marker in markers:
    if not isinstance(marker, tuple) or len(marker) != 4:
      raise ValueError(ERROR_MSG)


def generate_aruco_map(start_marker=1,
                       pos=(0, 0, 0),
                       number_of_markers=(10, 10),
                       distance=(1, 1)):
  markers = []
  current_marker = start_marker

  for y in range(number_of_markers[1]):
    for x in range(number_of_markers[0]):
      pos_x = pos[0] + x * distance[0]
      pos_y = pos[1] + y * distance[1]
      markers.append((current_marker, pos_x, pos_y, pos[2]))
      current_marker += 1
  return markers



class ArucoMap():
  def __init__(self, name, markers, marker_size=0.33):
    validate_markers(markers)
    self.name = name
    self.markers = markers
    self.marker_size = marker_size

  def generate(self):
    markers_imgs = {}

    output = open("/home/clover/catkin_ws/src/clover/aruco_pose/map/map.txt", "w")
    output.write('# id\tlength\tx\ty\tz\trot_z\trot_y\trot_x\n')

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    marker_bits = aruco_dict.markerSize
    marker_outer_bits = marker_bits + 2
    marker_border_bits = marker_bits + 4
    for marker in self.markers:
      marker_id, x, y, z = marker
      marker_image = np.zeros((marker_border_bits, marker_border_bits), dtype=np.uint8)
      marker_image[:,:] = 255
      marker_image[1:marker_border_bits - 1, 1:marker_border_bits - 1] = 0
      marker_image[1:marker_border_bits - 1, 1:marker_border_bits - 1] = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_outer_bits)
      markers_imgs["aruco_" + str(marker_id)] = marker_image
      output.write(f'{marker_id}\t{self.marker_size}\t{x}\t{y}\t{z}\t0\t0\t0\n')
    self.materials = ImageTextures(markers_imgs)
    self.materials.generate_materials()
    self.real_marker_size = self.marker_size / marker_outer_bits * marker_border_bits

    output.close()
    return self


  def xml(self):
    return f"""
    <model name="{self.name}">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        {self.create_visual_links()}
      </link>
    </model>
    """

  def create_visual_links(self):
    marker_visuals = []
    for marker in self.markers:
      marker_id, x, y, z = marker
      marker_visuals.append(f"""
      <visual name="visual_marker_${marker_id}">
        <pose>{x} {y} {z} 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <box>
            <size>{self.real_marker_size} {self.real_marker_size} 1e-3</size>
          </box>
        </geometry>
        {self.materials["aruco_" + str(marker_id)].xml()}
      </visual>
      """)
    return ''.join(marker_visuals)
