import itertools
from typing import Any, Dict, Tuple
import numpy as np
import cv2

TICI = False

## -- hardcoded hardware params --
eon_f_focal_length = 910.0
eon_d_focal_length = 650.0
tici_f_focal_length = 2648.0
tici_e_focal_length = tici_d_focal_length = 567.0 # probably wrong? magnification is not consistent across frame

eon_f_frame_size = (1164, 874)
eon_d_frame_size = (816, 612)
tici_f_frame_size = tici_e_frame_size = tici_d_frame_size = (1928, 1208)

eon_f_frame_size = (1164, 874)
# aka 'K' aka camera_frame_from_view_frame
eon_fcam_intrinsics = np.array([
  [eon_f_focal_length,  0.0,  float(eon_f_frame_size[0])/2],
  [0.0,  eon_f_focal_length,  float(eon_f_frame_size[1])/2],
  [0.0,  0.0,                                          1.0]])
eon_intrinsics = eon_fcam_intrinsics # xx
tici_f_frame_size = tici_e_frame_size = tici_d_frame_size = (1928, 1208)

# aka 'K' aka camera_frame_from_view_frame
eon_fcam_intrinsics = np.array([
  [eon_f_focal_length,  0.0,  float(eon_f_frame_size[0])/2],
  [0.0,  eon_f_focal_length,  float(eon_f_frame_size[1])/2],
  [0.0,  0.0,                                          1.0]])
eon_intrinsics = eon_fcam_intrinsics # xx

eon_dcam_intrinsics = np.array([
  [eon_d_focal_length,  0.0,  float(eon_d_frame_size[0])/2],
  [0.0,  eon_d_focal_length,  float(eon_d_frame_size[1])/2],
  [0.0,  0.0,                                          1.0]])

tici_fcam_intrinsics = np.array([
  [tici_f_focal_length,  0.0,  float(tici_f_frame_size[0])/2],
  [0.0,  tici_f_focal_length,  float(tici_f_frame_size[1])/2],
  [0.0,  0.0,                                            1.0]])

tici_dcam_intrinsics = np.array([
  [tici_d_focal_length,  0.0,  float(tici_d_frame_size[0])/2],
  [0.0,  tici_d_focal_length,  float(tici_d_frame_size[1])/2],
  [0.0,  0.0,                                            1.0]])

tici_ecam_intrinsics = tici_dcam_intrinsics

# aka 'K_inv' aka view_frame_from_camera_frame
eon_fcam_intrinsics_inv = np.linalg.inv(eon_fcam_intrinsics)
eon_intrinsics_inv = eon_fcam_intrinsics_inv # xx

tici_fcam_intrinsics_inv = np.linalg.inv(tici_fcam_intrinsics)
tici_ecam_intrinsics_inv = np.linalg.inv(tici_ecam_intrinsics)

if not TICI:
  FULL_FRAME_SIZE = eon_f_frame_size
  FOCAL = eon_f_focal_length
  fcam_intrinsics = eon_fcam_intrinsics
else:
  FULL_FRAME_SIZE = tici_f_frame_size
  FOCAL = tici_f_focal_length
  fcam_intrinsics = tici_fcam_intrinsics

W, H = FULL_FRAME_SIZE[0], FULL_FRAME_SIZE[1]

# device/mesh : x->forward, y-> right, z->down
# view : x->right, y->down, z->forward
device_frame_from_view_frame = np.array([
  [ 0.,  0.,  1.],
  [ 1.,  0.,  0.],
  [ 0.,  1.,  0.]
])
view_frame_from_device_frame = device_frame_from_view_frame.T

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

_FULL_FRAME_SIZE = {}
_BB_TO_FULL_FRAME = {}
_CALIB_BB_TO_FULL = {}
_FULL_FRAME_TO_BB = {}
_INTRINSICS = {}

eon_f_qcam_frame_size = (480, 360)
tici_f_qcam_frame_size = (528, 330)

cams = [(eon_f_frame_size, eon_f_focal_length, eon_f_frame_size),
        (tici_f_frame_size, tici_f_focal_length, tici_f_frame_size),
        (eon_f_qcam_frame_size, eon_f_focal_length, eon_f_frame_size),
        (tici_f_qcam_frame_size, tici_f_focal_length, tici_f_frame_size)]
for size, focal, full_size in cams:
  sz = size[0] * size[1]
  _BB_SCALE = size[0] / 640.
  _BB_TO_FULL_FRAME[sz] = np.asarray([
      [_BB_SCALE, 0., 0.],
      [0., _BB_SCALE, 0.],
      [0., 0., 1.]])
  calib_scale = full_size[0] / 640.
  _CALIB_BB_TO_FULL[sz] = np.asarray([
      [calib_scale, 0., 0.],
      [0., calib_scale, 0.],
      [0., 0., 1.]])
  _FULL_FRAME_TO_BB[sz] = np.linalg.inv(_BB_TO_FULL_FRAME[sz])
  _FULL_FRAME_SIZE[sz] = (size[0], size[1])
  _INTRINSICS[sz] = np.array([
    [focal, 0., full_size[0] / 2.],
    [0., focal, full_size[1] / 2.],
    [0., 0., 1.]])

# aka 'extrinsic_matrix'
# road : x->forward, y -> left, z->up
def get_view_frame_from_road_frame(roll, pitch, yaw, height):
  rot = np.array([[ 9.99744033e-01 , 1.37415130e-02,-1.79733113e-02],
                   [-1.37419091e-02, 9.99905570e-01, 1.01469225e-04],
                   [ 1.79730084e-02, 1.45544358e-04, 9.99838462e-01]])
  device_from_road = rot.dot(np.diag([1, -1, -1]))
  view_from_road = view_frame_from_device_frame.dot(device_from_road)
  return np.hstack((view_from_road, [[0], [height], [0]]))


# aka 'extrinsic_matrix'
def get_view_frame_from_calib_frame(roll, pitch, yaw, height):
  rot = np.array([[ 9.99744033e-01 , 1.37415130e-02,-1.79733113e-02],
                   [-1.37419091e-02, 9.99905570e-01, 1.01469225e-04],
                   [ 1.79730084e-02, 1.45544358e-04, 9.99838462e-01]])
  device_from_calib= rot
  view_from_calib = view_frame_from_device_frame.dot(device_from_calib)
  return np.hstack((view_from_calib, [[0], [height], [0]]))


class Calibration:
  def __init__(self, num_px, rpy, intrinsic):
    self.intrinsic = intrinsic
    self.extrinsics_matrix = get_view_frame_from_calib_frame(rpy[0], rpy[1], rpy[2], 0.0)[:,:3]
    self.zoom = _CALIB_BB_TO_FULL[num_px][0, 0]

  def car_space_to_ff(self, x, y, z):
    car_space_projective = np.column_stack((x, y, z)).T

    ep = self.extrinsics_matrix.dot(car_space_projective)
    kep = self.intrinsic.dot(ep)
    return (kep[:-1, :] / kep[-1, :]).T

  def car_space_to_bb(self, x, y, z):
    pts = self.car_space_to_ff(x, y, z)
    return pts / self.zoom

def draw_path(path, color, img, calibration, z_off=0):
  x, y, z = np.asarray(path.x), np.asarray(path.y), np.asarray(path.z) + z_off
  pts = calibration.car_space_to_bb(x, y, z)
  pts = np.round(pts).astype(int)

  height, width = img.shape[:2]
  for x, y in pts:
    if 1 < x < width - 1 and 1 < y < height - 1:
      for a, b in itertools.permutations([-1, 0, -1], 2):
        img[y + a, x + b] = color

def plot_model(m, img, calibration):
  if calibration is None:
    return

  for path, prob, _ in zip(m.laneLines, m.laneLineProbs, m.laneLineStds):
    color = (0, int(255 * prob), 0)
    draw_path(path, color, img, calibration)

  for edge, std in zip(m.roadEdges, m.roadEdgeStds):
    prob = max(1 - std, 0)
    color = (int(255 * prob), 0, 0)
    draw_path(edge, color, img, calibration)

  color = (255, 0, 0)
  draw_path(m.position, color, img, calibration, 1.22)


def plot_nav(nav, img):
  print (nav)
  text = nav.maneuverType + "," + nav.maneuverModifier + "," + str(nav.maneuverDistance)
  cv2.putText(img, text, (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 200)
