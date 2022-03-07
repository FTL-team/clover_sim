import PIL.Image as Image
import numpy as np
import os
from ..task import  TASK_PATH
from .material import OgreMaterial

cur_gen = 0

def save_cv2_image(path, img):
  import cv2
  cv2.imwrite(path, img)


GENERATED_MATERIALS_PATH = TASK_PATH + "/generated_materials/"

class ImageTextures:
  def __init__(self, imgs):
    for k, v in imgs.items():
      if type(v) is Image.Image: continue
      if type(v) is str: continue
      if type(v) is np.ndarray:
        if len(v.shape) in [2, 3]:
          continue
      raise ValueError('Invalid image type: ' + str(type(v)) + ', supported types are string path to image, pil image or cv2 image(numpy array)')

    self.imgs = imgs

    global cur_gen
    self.material_path = GENERATED_MATERIALS_PATH + str(cur_gen) + "/"
    os.makedirs(self.material_path, exist_ok=True)
    cur_gen+=1

  def generate_materials(self):
    material_file = open(self.material_path + "images.material", "w")
    self.materials = {}
    for k, v in self.imgs.items():
      if type(v) is str:
        path = v
      else:
        path = self.material_path + f"{k}.png"

      if type(v) is np.ndarray: save_cv2_image(path, v)
      if type(v) is Image.Image: v.save(path)
      material_file.write(f"""
material {k}
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture {path}
        filtering none
      }}
    }}
  }}
}}
""")
      self.materials[k] = OgreMaterial(self.material_path, k)
    material_file.close()

  def __getitem__(self, key):
    return self.materials[key]
