#!/usr/bin/env python3
import os
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class MapPublisher(Node):
    """
    Loads a PNG (white=free, black=obstacle) and publishes it as /map (OccupancyGrid).
    Coordinate convention:
      - Grid index: (row, col) with row=0 at TOP of image, col=0 at LEFT.
      - World frame: origin at bottom-left cell corner, +x right, +y up.
      - Cell center at ( (c+0.5)*res, (r+0.5)*res )  after flipping rows.
    """
    def __init__(self):
        super().__init__('world_map_pub')

        # Parameters
        self.declare_parameter('map_png', 'config/map_40x40.png')
        self.declare_parameter('resolution', 0.25)  # meters per cell
        self.declare_parameter('origin_xy', [0.0, 0.0])  # bottom-left corner
        self.declare_parameter('occupied_threshold', 0.5)  # 0..1 threshold on grayscale
        self.declare_parameter('frame_id', 'map')

        map_path = self.get_parameter('map_png').value
        res      = float(self.get_parameter('resolution').value)
        ox, oy   = [float(v) for v in self.get_parameter('origin_xy').value]
        occ_thr  = float(self.get_parameter('occupied_threshold').value)
        frame_id = self.get_parameter('frame_id').value

        # Load image → grayscale [0..1]
        if not os.path.isabs(map_path):
            map_path = os.path.join(os.path.dirname(__file__), '..', map_path)
            map_path = os.path.abspath(map_path)

        img = Image.open(map_path).convert('L')  # 0..255
        g = np.asarray(img, dtype=np.float32) / 255.0

        # Binary occupancy:
        # black (0.0) → occupied (100), white (1.0) → free (0)
        occ = np.where(g <= occ_thr, 100, 0).astype(np.int8)

        # Flip rows so world +y is "up" while images have row0 at top
        occ = np.flipud(occ)

        # Prepare OccupancyGrid
        h, w = occ.shape
        meta = MapMetaData()
        meta.resolution = res
        meta.width = w
        meta.height = h
        meta.origin = Pose()
        meta.origin.position.x = ox
        meta.origin.position.y = oy
        meta.origin.position.z = 0.0
        # quaternion stays identity (no rotation)

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = frame_id
        msg.info = meta
        msg.data = occ.flatten().tolist()  # row-major

        # Latched publisher (transient local)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)

        # Publish once (and on startup timer to ensure latching)
        self.pub.publish(msg)
        self.get_logger().info(f"Published /map from '{map_path}' ({w}x{h}) @ {res} m/cell, frame='{frame_id}'")

def main():
    rclpy.init()
    node = MapPublisher()
    rclpy.spin(node)  # keep alive so late subscribers receive the latched map
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
