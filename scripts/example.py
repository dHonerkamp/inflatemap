from typing import Union, List, Tuple, Optional, Callable
import numpy as np
from pybindings import get_inflated_map
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Pose, Point, Quaternion
from PIL import Image, ImageDraw, ImageOps


def _build_occgrid_msg(floorplan_img: Union[Image.Image, np.ndarray], origin: Tuple[float, float] = None,
                       q: Quaternion = None, resolution: float = None) -> OccupancyGrid:
    if origin is None:
        origin_W_meter, origin_H_meter = self._origin_W_meter, self._origin_H_meter
    else:
        assert len(origin) == 2, origin
        origin_W_meter, origin_H_meter = origin

    if q is None:
        q = Quaternion(0, 0, 0, 1)
    if resolution is None:
        resolution = self.global_map_resolution

    if isinstance(floorplan_img, Image.Image):
        floorplan_img = np.asarray(floorplan_img)

    occ_grid = OccupancyGrid()
    occ_grid.header.frame_id = "world"
    occ_grid.header.seq = 1
    # m/cell
    occ_grid.info.resolution = resolution
    occ_grid.info.width = floorplan_img.shape[1]
    occ_grid.info.height = floorplan_img.shape[0]
    # origin of cell (0, 0)
    occ_grid.info.origin = Pose(Point(-origin_W_meter, -origin_H_meter, 0), q)
    occ_grid.data = np.flipud(floorplan_img.astype(np.int)).flatten().tolist()
    return occ_grid


def get_inflated_map(self, cost_scaling_factor: float = 10.0, inflation_radius_meter: float = None, ignore_below_height: float = 0.0, floorplan_img=None) -> np.ndarray:
    if inflation_radius_meter is None:
        inflation_radius_meter = self.inflation_radius_meter
    elif inflation_radius_meter == 0.0:
        return np.asarray(self.binary_floorplan(0.0))

    ignore_below_height = max(ignore_below_height, 0.001)

    if floorplan_img is None:
        floorplan_img = self.binary_floorplan(ignore_below_height=ignore_below_height)
    occ_grid = self._build_occgrid_msg(floorplan_img)

    origin_vector = [occ_grid.info.origin.position.x, occ_grid.info.origin.position.y, occ_grid.info.origin.position.z,
                     occ_grid.info.origin.orientation.x, occ_grid.info.origin.orientation.y, occ_grid.info.origin.orientation.z, occ_grid.info.origin.orientation.w]

    inflated_occ_grid = get_inflated_map(occ_grid.data,
                                         occ_grid.info.resolution,
                                         occ_grid.info.width,
                                         occ_grid.info.height,
                                         origin_vector,
                                         occ_grid.header.frame_id,
                                         inflation_radius_meter,
                                         cost_scaling_factor)
    inflated_occ_grid = np.array(inflated_occ_grid).reshape((floorplan_img.size[1], floorplan_img.size[0]))
    inflated_occ_grid = np.flipud(inflated_occ_grid)
    # into [0, 1] range
    assert np.max(inflated_occ_grid) <= 100, np.max(inflated_occ_grid)
    inflated_occ_grid = inflated_occ_grid / 100
    # cache the inflated map
    return inflated_occ_grid
