#!/usr/bin/env python3
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MapPublisher(Node):
    def __init__(self):
        super().__init__('world_map_pub', automatically_declare_parameters_from_overrides=True)

        def _p(name, default):
            p = self.get_parameter(name)
            return (p.value if p.type_ is not None else default)

        self.map_png  = str(_p('map_png', ''))
        self.res      = float(_p('resolution', 0.25))
        origin_xy     = _p('origin_xy', [0.0, 0.0])
        self.origin_x = float(origin_xy[0] if isinstance(origin_xy, (list,tuple)) and len(origin_xy)>=2 else 0.0)
        self.origin_y = float(origin_xy[1] if isinstance(origin_xy, (list,tuple)) and len(origin_xy)>=2 else 0.0)
        self.frame_id = str(_p('frame_id', 'map'))
        self.occ_thr  = float(_p('occupied_threshold', 0.5))

        if not self.map_png or not os.path.exists(self.map_png):
            self.get_logger().fatal(f"map_png missing/invalid: {self.map_png}")
            raise SystemExit(1)

        img = cv2.imread(self.map_png, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().fatal(f"failed to read map_png: {self.map_png}")
            raise SystemExit(1)

        h, w = img.shape
        # 0..100 occupancy; >thr → occupied
        thr = int(np.clip(self.occ_thr, 0.0, 1.0) * 255)
        occ = (img < thr).astype(np.uint8) * 100  # black = occupied
        occ = occ.astype(np.int8)  # OccupancyGrid wants int8

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)

        meta = MapMetaData()
        meta.resolution = self.res
        meta.width  = int(w)
        meta.height = int(h)
        meta.origin.position.x = self.origin_x
        meta.origin.position.y = self.origin_y
        meta.origin.orientation.w = 1.0

        msg = OccupancyGrid()
        msg.header = Header(frame_id=self.frame_id)
        msg.info = meta
        msg.data = occ.flatten().tolist()

        self.pub.publish(msg)
        self.get_logger().info(f"afs_world: published map {w}x{h} res={self.res} frame={self.frame_id}")

def main():
    rclpy.init()
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
