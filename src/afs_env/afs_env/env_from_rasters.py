#!/usr/bin/env python3
import os, json
import numpy as np
import rasterio
from scipy import ndimage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import MultiArrayDimension as Dim

# ---- helpers ----
def _fa(arr: np.ndarray) -> Float32MultiArray:
    """Convert 2D numpy array to Float32MultiArray with row/col dims."""
    msg = Float32MultiArray()
    msg.layout.dim = [
        Dim(label='rows', size=arr.shape[0], stride=arr.size),
        Dim(label='cols', size=arr.shape[1], stride=arr.shape[1]),
    ]
    msg.data = arr.astype(np.float32).ravel().tolist()
    return msg

def _read_band(path: str) -> np.ndarray:
    with rasterio.open(path) as src:
        band = src.read(1).astype('float32')
    # Sentinel-style scaling
    if band.max() > 1.0:
        band = band / 10000.0
    return band

def _block_stat(arr: np.ndarray, rows: int, cols: int, func=np.nanmean) -> np.ndarray:
    H, W = arr.shape
    bh, bw = H // rows, W // cols
    out = np.zeros((rows, cols), dtype=float)
    for i in range(rows):
        for j in range(cols):
            rs, re = i*bh, H if i==rows-1 else (i+1)*bh
            cs, ce = j*bw, W if j==cols-1 else (j+1)*bw
            block = arr[rs:re, cs:ce]
            val = func(block) if block.size else np.nan
            out[i,j] = val
    return out

def _compute_ndvi(nir: np.ndarray, red: np.ndarray) -> np.ndarray:
    return (nir - red) / (nir + red + 1e-6)

def _classify_seasonal(ndvi_s: np.ndarray, ndvi_w: np.ndarray) -> np.ndarray:
    # bare=1, deciduous=2, conifer=3, sparse=4
    cls = np.zeros(ndvi_s.shape, dtype=np.uint8)
    cls[ndvi_s < 0.5] = 1
    mask4 = (ndvi_s >= 0.5) & (ndvi_s < 0.65)
    cls[mask4] = 4
    forest = ndvi_s >= 0.65
    amp = ndvi_s - ndvi_w
    cls[forest & (amp > 0.2)]  = 2
    cls[forest & (amp <= 0.2)] = 3
    return cls

VEG_LABELS = {1:'bare', 2:'deciduous', 3:'conifer', 4:'sparse'}
VEG_FUEL_MULT = {'bare':0.3, 'sparse':1.0, 'deciduous':0.8, 'conifer':1.4}

# ---- main node ----
class EnvFromRasters(Node):
    def __init__(self):
        super().__init__('env_from_rasters')

        # Parameters
        self.rows = int(self.declare_parameter('rows', 40).value)
        self.cols = int(self.declare_parameter('cols', 40).value)

        self.summer_red = self.declare_parameter('summer_red', '').value
        self.summer_nir = self.declare_parameter('summer_nir', '').value
        self.winter_red = self.declare_parameter('winter_red', '').value
        self.winter_nir = self.declare_parameter('winter_nir', '').value
        self.dem_path   = self.declare_parameter('dem_tif', '').value

        for p in [self.summer_red, self.summer_nir, self.winter_red, self.winter_nir, self.dem_path]:
            if not p or not os.path.exists(p):
                self.get_logger().fatal(f"Missing or invalid raster path: {p}")
                raise SystemExit(1)

        # Latched QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Publishers
        self.pub_veg   = self.create_publisher(String,            '/grid/vegetation', qos)
        self.pub_fuel  = self.create_publisher(Float32MultiArray, '/grid/fuel_load',  qos)
        self.pub_elev  = self.create_publisher(Float32MultiArray, '/grid/elevation',  qos)
        self.pub_slope = self.create_publisher(Float32MultiArray, '/grid/slope',      qos)
        self.pub_aspect= self.create_publisher(Float32MultiArray, '/grid/aspect',     qos)

        # Build & publish once
        self._build_and_publish()
        self.get_logger().info(f"afs_env: published rasters → {self.rows}x{self.cols} grids")

    def _build_and_publish(self):
        # 1. NDVI seasonal classification
        sr, sn = _read_band(self.summer_red), _read_band(self.summer_nir)
        wr, wn = _read_band(self.winter_red), _read_band(self.winter_nir)
        ndvi_s, ndvi_w = _compute_ndvi(sn, sr), _compute_ndvi(wn, wr)
        cls_full = _classify_seasonal(ndvi_s, ndvi_w)

        # 2. DEM → slope/aspect
        with rasterio.open(self.dem_path) as src:
            dem = src.read(1).astype('float32')
            tr = src.transform
        xres, yres = tr.a, -tr.e if tr.e < 0 else tr.e
        dzdx = ndimage.sobel(dem, axis=1)/(8*max(xres,1e-6))
        dzdy = ndimage.sobel(dem, axis=0)/(8*max(yres,1e-6))
        slope  = np.degrees(np.arctan(np.hypot(dzdx, dzdy))).astype(np.float32)
        aspect = (np.degrees(np.arctan2(dzdy, -dzdx)) + 360) % 360

        # 3. Block downsample to grid
        veg_grid_idx = _block_stat(cls_full, self.rows, self.cols,
                                   func=lambda a: np.bincount(a.astype(np.uint8).ravel(), minlength=5).argmax())
        elev_grid    = _block_stat(dem,    self.rows, self.cols)
        slope_grid   = _block_stat(slope,  self.rows, self.cols)
        aspect_grid  = _block_stat(aspect, self.rows, self.cols)

        # 4. Labels & fuel load
        veg_labels = np.vectorize(lambda k: VEG_LABELS.get(int(k), 'sparse'))(veg_grid_idx)
        veg_json   = json.dumps(veg_labels.tolist())
        base_fuel  = np.vectorize(lambda s: VEG_FUEL_MULT.get(s, 1.0))(veg_labels).astype(np.float32)
        fuel       = 0.4 + 1.2 * (base_fuel - base_fuel.min()) / (base_fuel.max() - base_fuel.min() + 1e-6)

        # Publish
        self.pub_veg.publish(String(data=veg_json))
        self.pub_fuel.publish(_fa(fuel))
        self.pub_elev.publish(_fa(elev_grid.astype(np.float32)))
        self.pub_slope.publish(_fa(slope_grid.astype(np.float32)))
        self.pub_aspect.publish(_fa(aspect_grid.astype(np.float32)))

# ---- main ----
def main():
    rclpy.init()
    node = EnvFromRasters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
