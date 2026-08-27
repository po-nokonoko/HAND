import socket
import struct
import selectors
from queue import Queue
from typing import Any, cast

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # type: ignore

from scipy.optimize import least_squares, OptimizeResult

import hand_data_pb2  # Generated Protocol Buffers file

# Enable debug output if necessary
DEBUG_ENABLE: bool = False

def debug_print(arg: str) -> None:
    if DEBUG_ENABLE:
        print(arg)

HOST: str = "0.0.0.0"
PORT: int = 8055

sel: selectors.DefaultSelector = selectors.DefaultSelector()

# Protocol Mappings
data_type_map = {
    hand_data_pb2.UINT8: "UINT8",
    hand_data_pb2.UINT16: "UINT16",
    hand_data_pb2.INT16: "INT16",
    hand_data_pb2.INT32: "INT32",
    hand_data_pb2.INT64: "INT64",
    hand_data_pb2.FLOAT: "FLOAT",
    hand_data_pb2.DOUBLE: "DOUBLE",
    hand_data_pb2.CH101_SIMPLE: "CH101_SIMPLE",
    hand_data_pb2.CH101_AMP: "CH101_AMP",
    hand_data_pb2.CH101_IQ: "CH101_IQ",
}

source_map= {
    hand_data_pb2.ESP32_S3_MINI_MAIN: "ESP32_S3_MINI_MAIN",
    hand_data_pb2.BQ27427_BATTERY: "BQ27427_BATTERY",
    hand_data_pb2.KX132_1211_SENSOR1: "KX132_1211_SENSOR1",
    hand_data_pb2.KX132_1211_SENSOR2: "KX132_1211_SENSOR2",
    hand_data_pb2.KX132_1211_SENSOR3: "KX132_1211_SENSOR3",
    hand_data_pb2.KX132_1211_SENSOR4: "KX132_1211_SENSOR4",
    hand_data_pb2.VL53L1X_SENSOR1: "VL53L1X_SENSOR1",
    hand_data_pb2.VL53L1X_SENSOR2: "VL53L1X_SENSOR2",
    hand_data_pb2.CH101_SENSOR1: "CH101_SENSOR1",
    hand_data_pb2.CH101_SENSOR2: "CH101_SENSOR2",
    hand_data_pb2.CH101_SENSOR3: "CH101_SENSOR3",
    hand_data_pb2.CH101_SENSOR4: "CH101_SENSOR4",
    hand_data_pb2.BOS1901_ACTUATOR1: "BOS1901_ACTUATOR1",
    hand_data_pb2.BOS1901_ACTUATOR2: "BOS1901_ACTUATOR2",
    hand_data_pb2.BOS1901_ACTUATOR3: "BOS1901_ACTUATOR3",
    hand_data_pb2.BOS1901_ACTUATOR4: "BOS1901_ACTUATOR4",
    hand_data_pb2.BMI323_IMU: "BMI323_IMU",
    hand_data_pb2.TCA6408A_CH101: "TCA6408A_CH101",
    hand_data_pb2.TCA6408A_OTHER: "TCA6408A_OTHER",
}

log_queue: Queue[Any] = Queue()

# Sensor 3D Coordinates (dev0, dev2, dev3) - Excluding dev1
SENSOR_POSITIONS: dict[str, np.ndarray] = {
    "CH101_SENSOR1": np.array([0.0, 0.0, 0.0]),     # dev0
    "CH101_SENSOR3": np.array([68.5, 0.0, 0.0]),   # dev2
    "CH101_SENSOR4": np.array([68.5, 11.0, 0.0]),  # dev3
}

# 3D Restrictive Writing Space Constraints (mm)
BOUNDING_BOX: dict[str, tuple[float, float]] = {
    "x": (0.0, 68.5),
    "y": (0.0, 11.0),
    "z": (10.0, 20.0)
}

# Extended Kalman Filter Setup
class FingertipEKF:
    def __init__(self, dt: float = 0.05) -> None:
        self.dt: float = dt
        self.x: np.ndarray = np.array([34.25, 5.5, 30.0, 0.0, 0.0, 0.0], dtype=np.float64) # [x, y, z, vx, vy, vz]
        self.cov_P: np.ndarray = np.eye(6) * 10.0
        self.Q: np.ndarray = np.eye(6) * 0.1
        self.R_base: float = 2.0

    def predict(self) -> np.ndarray:
        F = np.array([
            [1, 0, 0, self.dt, 0,       0],
            [0, 1, 0, 0,       self.dt, 0],
            [0, 0, 1, 0,       0,       self.dt],
            [0, 0, 0, 1,       0,       0],
            [0, 0, 0, 0,       1,       0],
            [0, 0, 0, 0,       0,       1]
        ], dtype=np.float64)
        self.x = F @ self.x
        self.cov_P = F @ self.cov_P @ F.T + self.Q
        return self.x[:3]

    def update(self, z: np.ndarray, weights: np.ndarray) -> np.ndarray:
        # z: ranges [r0, r2, r3]
        pos = self.x[:3]
        sensors = [SENSOR_POSITIONS["CH101_SENSOR1"], 
                   SENSOR_POSITIONS["CH101_SENSOR3"], 
                   SENSOR_POSITIONS["CH101_SENSOR4"]]
        
        h = np.zeros(3, dtype=np.float64)
        H = np.zeros((3, 6), dtype=np.float64)
        
        for i, p_s in enumerate(sensors):
            dist = float(np.linalg.norm(pos - p_s))
            dist = max(dist, 1e-5)
            h[i] = dist
            H[i, 0] = (pos[0] - p_s[0]) / dist
            H[i, 1] = (pos[1] - p_s[1]) / dist
            H[i, 2] = (pos[2] - p_s[2]) / dist

        # Apply measurement noise inverted by weights
        R = np.diag([self.R_base / max(float(w), 1e-3) for w in weights])
        
        y = z - h
        S = H @ self.cov_P @ H.T + R
        K = self.cov_P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.cov_P = (np.eye(6) - K @ H) @ self.cov_P
        return self.x[:3]

ekf: FingertipEKF = FingertipEKF()

# Visualizer Class using Matplotlib
class RealtimeVisualizer:
    def __init__(self) -> None:
        plt.ion()
        self.fig = plt.figure(figsize=(8, 6))
        self.ax: Any = cast(Axes3D, self.fig.add_subplot(111, projection='3d'))
        self.trajectory: list[np.ndarray] = []
        
        # Plot Sensors
        for name, pos in SENSOR_POSITIONS.items():
            self.ax.scatter(pos[0], pos[1], pos[2], color='red', marker='^', s=80, label=name)
        
        self.ax.set_xlim(BOUNDING_BOX["x"])
        self.ax.set_ylim(BOUNDING_BOX["y"])
        self.ax.set_zlim(BOUNDING_BOX["z"])
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_title("Real-Time Fingertip Writing Tracking")
        self.point = None
        self.line = None

    def update_position(self, pos: np.ndarray) -> None:
        self.trajectory.append(pos)
        if len(self.trajectory) > 50:
            self.trajectory.pop(0)
            
        traj_arr = np.array(self.trajectory)
        
        if self.point is not None:
            self.point.remove()
        if self.line is not None:
            self.line[0].remove()
            
        self.line = self.ax.plot(traj_arr[:, 0], traj_arr[:, 1], traj_arr[:, 2], 'b-')
        self.point = self.ax.scatter(pos[0], pos[1], pos[2], color='green', s=100)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

visualizer: RealtimeVisualizer | None = None

# SECTION 1-1 & 1-2: Decoding Logic with Stream Separation
def decode_data(data_msg: Any) -> list[dict[str, list[Any]]]:
    if data_msg.data_type == hand_data_pb2.CH101_SIMPLE:
        return custom_decode(hand_data_pb2.CH101_SIMPLE, data_msg.data)
    elif data_msg.data_type == hand_data_pb2.CH101_AMP:
        return custom_decode(hand_data_pb2.CH101_AMP, data_msg.data)
    elif data_msg.data_type == hand_data_pb2.CH101_IQ:
        return custom_decode(hand_data_pb2.CH101_IQ, data_msg.data)
    else:
        print(f"Unknown data type: {data_msg.data_type}")
        return []

def custom_decode(data_type: int, data: bytes) -> list[dict[str, list[Any]]]:
    if data_type == hand_data_pb2.CH101_SIMPLE:
        struct_format = "<qHHf"
        unit_size = struct.calcsize(struct_format)
        decoded_data: dict[str, list[Any]] = {"timestamps": [], "sample_num": [], "amp": [], "range": []}
        for i in range(0, len(data), unit_size):
            if i + unit_size <= len(data):
                unit = struct.unpack(struct_format, data[i:i + unit_size])
                decoded_data["timestamps"].append(unit[0])
                decoded_data["sample_num"].append(unit[1])
                decoded_data["amp"].append(unit[2])
                decoded_data["range"].append(unit[3])
        return [decoded_data]
    
    elif data_type == hand_data_pb2.CH101_AMP:
        struct_format = "<H"
        unit_size = struct.calcsize(struct_format)
        decoded_data_amp: dict[str, list[Any]] = {"amp_data": []}
        for i in range(0, len(data), unit_size):
            if i + unit_size <= len(data):
                unit = struct.unpack(struct_format, data[i:i + unit_size])
                decoded_data_amp["amp_data"].append(unit[0])
        return [decoded_data_amp]
    
    elif data_type == hand_data_pb2.CH101_IQ:
        struct_format = "<hh"
        unit_size = struct.calcsize(struct_format)
        decoded_data_iq: dict[str, list[Any]] = {"i": [], "q": []}
        for i in range(0, len(data), unit_size):
            if i + unit_size <= len(data):
                unit = struct.unpack(struct_format, data[i:i + unit_size])
                decoded_data_iq["i"].append(unit[0])
                decoded_data_iq["q"].append(unit[1])
        return [decoded_data_iq]
    
    return []

def is_within_bounds(pos: np.ndarray) -> bool:
    return bool(BOUNDING_BOX["x"][0] <= pos[0] <= BOUNDING_BOX["x"][1] and
                BOUNDING_BOX["y"][0] <= pos[1] <= BOUNDING_BOX["y"][1] and
                BOUNDING_BOX["z"][0] <= pos[2] <= BOUNDING_BOX["z"][1])

def process_sensor_frame(sensor_data_store: dict[str, dict[str, Any]]) -> None:
    """Computes multilateration, updates EKF, and renders tracking."""
    active_sensors = ["CH101_SENSOR1", "CH101_SENSOR3", "CH101_SENSOR4"]
    ranges_list: list[float] = []
    amps_list: list[float] = []
    iq_amps_list: list[float] = []
    
    for s_name in active_sensors:
        if s_name not in sensor_data_store:
            return
        
        s_info = sensor_data_store[s_name]
        r = float(s_info.get("range", 0.0))
        amp = float(s_info.get("amp", 0.0))
        i_val = float(s_info.get("i", 0.0))
        q_val = float(s_info.get("q", 0.0))
        iq_amp = float(np.sqrt(i_val**2 + q_val**2))
        
        ranges_list.append(r)
        amps_list.append(amp)
        iq_amps_list.append(iq_amp)
        
    ranges = np.array(ranges_list, dtype=np.float64)
    amps = np.array(amps_list, dtype=np.float64)
    iq_amps = np.array(iq_amps_list, dtype=np.float64)
    
    # Calculate Weights excluding dev1
    alpha = 0.5
    raw_weights: np.ndarray = alpha * amps + (1 - alpha) * iq_amps
    total_w = float(np.sum(raw_weights))
    weights: np.ndarray = raw_weights / total_w if total_w > 0 else np.ones(3) / 3.0
    
    # Multilateration Initial Guess
    def trilateration_residuals(pos: np.ndarray) -> np.ndarray:
        res: list[float] = []
        for idx, key in enumerate(active_sensors):
            p_s = SENSOR_POSITIONS[key]
            res.append(float(np.linalg.norm(pos - p_s)) - ranges[idx])
        return np.array(res, dtype=np.float64)

    p_pred = ekf.predict()
    opt_res: OptimizeResult = least_squares(trilateration_residuals, p_pred, method='lm')
    raw_pos: np.ndarray = opt_res.x
    
    # Validation against 3D Restrictive Space
    if not is_within_bounds(raw_pos):
        print(f"[REJECTED] Out of writing bounds: {raw_pos}")
        return
        
    # EKF Update
    updated_pos = ekf.update(ranges, weights)
    
    print(f"[TRACKING] Fingertip Position (X, Y, Z): ({updated_pos[0]:.2f}, {updated_pos[1]:.2f}, {updated_pos[2]:.2f}) mm")
    
    if visualizer is not None:
        visualizer.update_position(updated_pos)

# SECTION 2: Real-time Data Receiver and Handler
def read(conn: socket.socket, mask: int) -> None:
    try:
        initial_data = conn.recv(5)
        if len(initial_data) < 5:
            sel.unregister(conn)
            conn.close()
            return

        remaining_bytes = struct.unpack("<I", initial_data[1:5])[0] - 5

        data = bytearray()
        while len(data) < remaining_bytes:
            packet = conn.recv(remaining_bytes - len(data))
            if not packet:
                sel.unregister(conn)
                conn.close()
                return
            data.extend(packet)

        complete_data = initial_data + data
        msg = hand_data_pb2.HandMsg()
        msg.ParseFromString(complete_data)

        sensor_data_store: dict[str, dict[str, Any]] = {}

        # Separation and extraction mechanism matching timestamps
        for data_msg in msg.data_wrapper.data_msgs_simple:
            source_str = source_map.get(data_msg.source, "Unknown")
            if source_str == "CH101_SENSOR2":  # Skip dev1
                continue
            decoded = decode_data(data_msg)
            if decoded:
                sensor_data_store.setdefault(source_str, {})["range"] = decoded[0]["range"][-1] if decoded[0]["range"] else 0.0
                sensor_data_store.setdefault(source_str, {})["amp"] = decoded[0]["amp"][-1] if decoded[0]["amp"] else 0.0

        for data_msg in msg.data_wrapper.data_msgs_iq:
            source_str = source_map.get(data_msg.source, "Unknown")
            if source_str == "CH101_SENSOR2":  # Skip dev1
                continue
            decoded = decode_data(data_msg)
            if decoded:
                sensor_data_store.setdefault(source_str, {})["i"] = decoded[0]["i"][-1] if decoded[0]["i"] else 0.0
                sensor_data_store.setdefault(source_str, {})["q"] = decoded[0]["q"][-1] if decoded[0]["q"] else 0.0

        process_sensor_frame(sensor_data_store)

    except Exception as e:
        debug_print(f"Read Exception: {e}")
        sel.unregister(conn)
        conn.close()

def accept(sock: socket.socket, mask: int) -> None:
    conn, addr = sock.accept()
    print(f"Connected by {addr}")
    conn.setblocking(False)
    sel.register(conn, selectors.EVENT_READ, read)

def main() -> None:
    global visualizer
    visualizer = RealtimeVisualizer()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((HOST, PORT))
    sock.listen()
    sock.setblocking(False)
    print(f"Tracking Server active on {HOST}:{PORT}")

    sel.register(sock, selectors.EVENT_READ, accept)

    try:
        while True:
            events = sel.select(timeout=0.01)
            for key, mask in events:
                callback = key.data
                callback(key.fileobj, mask)
            plt.pause(0.001)
    except KeyboardInterrupt:
        print("\nStopping Fingertip Tracking...")
    finally:
        sel.close()
        sock.close()

if __name__ == "__main__":
    main()