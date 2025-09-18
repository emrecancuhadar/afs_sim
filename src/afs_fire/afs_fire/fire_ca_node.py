#!/usr/bin/env python3
import json, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from std_msgs.msg import MultiArrayDimension as Dim

FUEL, BURNING, BURNT = 0, 1, 2

def to_i32_grid(arr2d: np.ndarray):
    msg = Int32MultiArray()
    h, w = arr2d.shape
    msg.layout.dim = [
        Dim(label='rows', size=h, stride=h*w),
        Dim(label='cols', size=w, stride=w),
    ]
    msg.data = arr2d.astype(np.int32).ravel().tolist()
    return msg

def to_f32_grid(arr2d: np.ndarray):
    msg = Float32MultiArray()
    h, w = arr2d.shape
    msg.layout.dim = [
        Dim(label='rows', size=h, stride=h*w),
        Dim(label='cols', size=w, stride=w),
    ]
    msg.data = arr2d.astype(np.float32).ravel().tolist()
    return msg

class FireCANode(Node):
    def __init__(self):
        super().__init__('fire_ca', automatically_declare_parameters_from_overrides=True)

        def _p(name, default):
            p = self.get_parameter(name)
            return (p.value if p.type_ is not None else default)

        self.rows        = int(_p('rows', 40))
        self.cols        = int(_p('cols', 40))
        self.dt          = float(_p('dt', 0.25))
        self.seed        = int(_p('seed', 17))
        self.wind_dir    = str(_p('wind_dir', 'W')).upper()
        self.wind_ft_min = float(_p('wind_ft_min', 4800.0))
        self.dwell_steps = int(_p('dwell_steps', 8))
        self.A           = float(_p('A_ros_like', 0.8))
        self.B           = float(_p('B_wind_align', 0.15))
        self.C           = float(_p('C_fwi_like', 0.05))
        self.seeded = False

        ignitions = _p('ignitions', [])
        # Accept either direct [[r,c], ...] or JSON strings ["[r,c]", ...]
        if ignitions and isinstance(ignitions[0], str):
            parsed = []
            for s in ignitions:
                try:
                    pair = json.loads(s)
                    if isinstance(pair, list) and len(pair) == 2:
                        parsed.append(pair)
                except Exception:
                    pass
            ignitions = parsed or ignitions
        self.ignitions = []
        if isinstance(ignitions, (list, tuple)):
            for item in ignitions:
                if isinstance(item, (list, tuple)) and len(item) == 2:
                    self.ignitions.append([int(item[0]), int(item[1])])

        # QoS
        self.env_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        self.out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscriptions (environment)
        self.fuel = None; self.veg = None; self.slope = None; self.aspect = None
        self.create_subscription(Float32MultiArray, '/grid/fuel_load', self._fuel_cb,  self.env_qos)
        self.create_subscription(String,            '/grid/vegetation', self._veg_cb,   self.env_qos)
        self.create_subscription(Float32MultiArray, '/grid/slope',      self._slope_cb, self.env_qos)
        self.create_subscription(Float32MultiArray, '/grid/aspect',     self._aspect_cb,self.env_qos)

        # External extinguish (for robots later): Int32MultiArray [row, col, radius]
        self.create_subscription(Int32MultiArray, '/fire/extinguish', self._extinguish_cb, self.out_qos)

        # Publishers
        self.pub_state = self.create_publisher(Int32MultiArray,  '/grid/fire_state', self.out_qos)
        self.pub_count = self.create_publisher(Float32MultiArray, '/grid/fire_count', self.out_qos)

        # State
        self.rng = np.random.default_rng(self.seed)
        self.state = np.full((self.rows, self.cols), FUEL, dtype=np.uint8)
        self.dwell = np.zeros((self.rows, self.cols), dtype=np.int32)
        self.wind_unit = self._wind_vec(self.wind_dir)

        # Timer
        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info("fire_ca: scenario ready; waiting for /grid/* layers")

    # --- Callbacks (env) ---
    def _fuel_cb(self, msg):   self.fuel   = np.array(msg.data, dtype=np.float32).reshape(self.rows, self.cols)
    def _slope_cb(self, msg):  self.slope  = np.array(msg.data, dtype=np.float32).reshape(self.rows, self.cols)
    def _aspect_cb(self, msg): self.aspect = np.array(msg.data, dtype=np.float32).reshape(self.rows, self.cols)
    def _veg_cb(self, msg: String):
        try:
            labels = np.array(json.loads(msg.data))
            mult = np.vectorize({'bare':0.3,'sparse':1.0,'deciduous':0.8,'conifer':1.4}.get)(labels)
            self.veg = mult.astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"vegetation parse failed: {e}")

    # --- External extinguish ---
    def _extinguish_cb(self, msg: Int32MultiArray):
        r, c, rad = msg.data
        r0, r1 = max(0, r-rad), min(self.rows-1, r+rad)
        c0, c1 = max(0, c-rad), min(self.cols-1, c+rad)
        self.state[r0:r1+1, c0:c1+1] = BURNT
        self.dwell[r0:r1+1, c0:c1+1] = 0
        self._publish()

    # --- Simulation tick ---
    def _step(self):
        """Advance the fire cellular automaton by one tick."""

        # wait until environment layers are available
        if any(v is None for v in [self.fuel, self.veg, self.slope, self.aspect]):
            return

        # Seed ignitions once
        if not self.seeded:
            for r, c in self.ignitions:
                if 0 <= r < self.rows and 0 <= c < self.cols:
                    self.state[r, c] = BURNING
            self.seeded = True
            self.get_logger().info(f"fire_ca: ignitions set -> {self.ignitions}")
            self._publish()
            return

        # Copy state for update
        new_state = self.state.copy()

        # Iterate all burning cells
        burning_cells = np.argwhere(self.state == BURNING)
        for (r, c) in burning_cells:
            # Decrement dwell counter
            self.dwell[r, c] -= 1
            if self.dwell[r, c] <= 0:
                new_state[r, c] = BURNT

            # Spread to neighbors
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.rows and 0 <= nc < self.cols:
                        if self.state[nr, nc] == FUEL:
                            # Base probability scaled by fuel load
                            p = self.A * self.fuel[nr, nc]

                            # Wind alignment factor
                            dot = dr * self.wind_unit[0] + dc * self.wind_unit[1]
                            if dot > 0:
                                p += self.B * dot

                            # Small FWI-like term
                            p += self.C

                            if np.random.rand() < p:
                                new_state[nr, nc] = BURNING
                                self.dwell[nr, nc] = self.dwell_steps

        # Update and publish
        self.state = new_state
        self._publish()


    # --- helpers ---
    def _wind_vec(self, d: str):
        rt = math.sqrt(0.5)
        dirs = {
            'N':  np.array([0.0,  1.0]),
            'NE': np.array([ rt,  rt]),
            'E':  np.array([1.0,  0.0]),
            'SE': np.array([ rt, -rt]),
            'S':  np.array([0.0, -1.0]),
            'SW': np.array([-rt, -rt]),
            'W':  np.array([-1.0, 0.0]),
            'NW': np.array([-rt, rt]),
        }
        v = dirs.get(d, np.array([-1.0, 0.0]))
        return v / (np.linalg.norm(v) + 1e-6)

    def _wind_scale(self):
        return float(np.clip(self.wind_ft_min / 9842.52, 0.0, 1.0) ** 0.7)

    def _publish(self):
        self.pub_state.publish(to_i32_grid(self.state))
        self.pub_count.publish(to_f32_grid((self.state == BURNING).astype(np.float32)))

def main():
    rclpy.init()
    node = FireCANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
