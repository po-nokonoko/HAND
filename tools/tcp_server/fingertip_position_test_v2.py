import socket
import struct
import selectors
import time  # Required for dynamic dt calculation
from typing import Any, cast

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # type: ignore
from scipy.optimize import least_squares, OptimizeResult
import hand_data_pb2  # Generated Protocol Buffers file

DEBUG_ENABLE: bool = False

def debug_print(arg: str) -> None:
    if DEBUG_ENABLE:
        print(arg)

HOST: str = "0.0.0.0"
PORT: int = 8055

sel: selectors.DefaultSelector = selectors.DefaultSelector()

BOUNDING_BOX: dict[str, tuple[float, float]] = {
    "x": (-50.0, 150.0),
    "y": (-50.0, 100.0),
    "z": (0.0, 120.0)
}

SENSOR_POSITIONS: dict[str, np.ndarray] = {
    "CH101_SENSOR1": np.array([0.0, 0.0, 0.0], dtype=np.float64),    # dev0
    "CH101_SENSOR3": np.array([68.5, 0.0, 0.0], dtype=np.float64),   # dev2
    "CH101_SENSOR4": np.array([68.5, 11.0, 0.0], dtype=np.float64),  # dev3
}

source_map = {
    hand_data_pb2.ESP32_S3_MINI_MAIN: "ESP32_S3_MINI_MAIN",
    hand_data_pb2.CH101_SENSOR1: "CH101_SENSOR1",
    hand_data_pb2.CH101_SENSOR2: "CH101_SENSOR2",
    hand_data_pb2.CH101_SENSOR3: "CH101_SENSOR3",
    hand_data_pb2.CH101_SENSOR4: "CH101_SENSOR4",
}

class ConnectionState:
    def __init__(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        self.conn: socket.socket = conn
        self.addr: tuple[str, int] = addr
        self.recv_buffer: bytearray = bytearray()

class BistaticFingertipEKF:
    def __init__(self) -> None:
        # 1. Initialize timing for dynamic dt
        self.last_time: float = time.time()
        self.x: np.ndarray = np.array([34.25, 5.5, 50.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.cov_P: np.ndarray = np.eye(6, dtype=np.float64) * 10.0
        
        # 2. Modulate Covariance Matrices for highly dynamic tracking
        self.Q: np.ndarray = np.eye(6, dtype=np.float64) * 1.0     # Increased process noise (allows fast acceleration)
        self.R_base: float = 2                                   # Lowered measurement noise (trusts raw sensors more)

    def predict(self) -> np.ndarray:
        # 3. Dynamic Delta-Time update
        now = time.time()
        dt = max(now - self.last_time, 1e-4)
        self.last_time = now
        
        F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ], dtype=np.float64)
        
        self.x = F @ self.x
        self.cov_P = F @ self.cov_P @ F.T + self.Q
        return self.x[:3]

    def update(self, raw_path_lengths: np.ndarray, active_sensor_keys: list[str], tx_key: str, weights: np.ndarray) -> np.ndarray:
        pos = self.x[:3]
        tx_pos = SENSOR_POSITIONS[tx_key]
        
        num_meas = len(active_sensor_keys)
        h = np.zeros(num_meas, dtype=np.float64)
        H = np.zeros((num_meas, 6), dtype=np.float64)
        
        d_tx = float(np.linalg.norm(pos - tx_pos))
        d_tx = max(d_tx, 1e-5)
        u_tx = (pos - tx_pos) / d_tx

        for i, s_key in enumerate(active_sensor_keys):
            p_s = SENSOR_POSITIONS[s_key]
            if s_key == tx_key:
                h[i] = 2.0 * d_tx
                H[i, 0:3] = 2.0 * u_tx
            else:
                d_rx = float(np.linalg.norm(pos - p_s))
                d_rx = max(d_rx, 1e-5)
                u_rx = (pos - p_s) / d_rx
                h[i] = d_tx + d_rx
                H[i, 0:3] = u_tx + u_rx

        R = np.diag([self.R_base / max(float(w), 1e-3) for w in weights])
        
        y = raw_path_lengths - h
        S = H @ self.cov_P @ H.T + R
        K = self.cov_P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.cov_P = (np.eye(6, dtype=np.float64) - K @ H) @ self.cov_P
        return self.x[:3]

ekf: BistaticFingertipEKF = BistaticFingertipEKF()

class RealtimeVisualizer:
    def __init__(self) -> None:
        plt.ion()
        self.fig = plt.figure(figsize=(8, 6))
        self.ax: Any = cast(Axes3D, self.fig.add_subplot(111, projection='3d'))
        self.trajectory: list[np.ndarray] = []
        
        for name, pos in SENSOR_POSITIONS.items():
            self.ax.scatter(pos[0], pos[1], pos[2], color='red', marker='^', s=80, label=name)
        
        self.ax.set_xlim(BOUNDING_BOX["x"])
        self.ax.set_ylim(BOUNDING_BOX["y"])
        self.ax.set_zlim(BOUNDING_BOX["z"])
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_title("Real-Time Fingertip Writing Tracking")
        
        # 4. Use persistent plot objects for rapid updating
        self.line, = self.ax.plot([], [], [], 'b-', lw=2.0)
        self.point = self.ax.scatter([], [], [], color='green', s=100)
        self.last_plot_time = time.time()

    def update_position(self, pos: np.ndarray) -> None:
        self.trajectory.append(pos)
        
        # 5. Expand trace buffer to prevent disappearing trails
        # 5000 frames @ 50Hz = ~100 seconds of continuous writing trace
        if len(self.trajectory) > 5000:
            self.trajectory.pop(0)

        # 6. Throttle GUI Rendering (~15 FPS) to prevent TCP socket starvation
        now = time.time()
        if now - self.last_plot_time < 0.066:
            return
        self.last_plot_time = now

        traj_arr = np.array(self.trajectory)
        
        # Efficient GUI updates instead of clearing and redrawing
        self.line.set_data(traj_arr[:, 0], traj_arr[:, 1])
        self.line.set_3d_properties(traj_arr[:, 2])
        self.point._offsets3d = (traj_arr[-1:, 0], traj_arr[-1:, 1], traj_arr[-1:, 2])
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

visualizer: RealtimeVisualizer | None = None

def clamp_to_bounds(pos: np.ndarray) -> np.ndarray:
    clamped_x = float(np.clip(pos[0], BOUNDING_BOX["x"][0], BOUNDING_BOX["x"][1]))
    clamped_y = float(np.clip(pos[1], BOUNDING_BOX["y"][0], BOUNDING_BOX["y"][1]))
    clamped_z = float(np.clip(pos[2], BOUNDING_BOX["z"][0], BOUNDING_BOX["z"][1]))
    return np.array([clamped_x, clamped_y, clamped_z], dtype=np.float64)

def decode_data(data_msg: Any) -> list[dict[str, list[Any]]]:
    if data_msg.data_type == hand_data_pb2.CH101_SIMPLE:
        return custom_decode(hand_data_pb2.CH101_SIMPLE, data_msg.data)
    elif data_msg.data_type == hand_data_pb2.CH101_IQ:
        return custom_decode(hand_data_pb2.CH101_IQ, data_msg.data)
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

def process_sensor_frame(sensor_data_store: dict[str, dict[str, Any]]) -> None:
    active_sensors = [k for k in ["CH101_SENSOR1", "CH101_SENSOR3", "CH101_SENSOR4"] if k in sensor_data_store]
    
    if len(active_sensors) < 3:
        return

    path_lengths_list, amps_list, iq_amps_list = [], [], []
    for s_name in active_sensors:
        s_info = sensor_data_store[s_name]
        r_path = float(s_info.get("range", 0.0))
        amp = float(s_info.get("amp", 0.0))
        i_val = float(s_info.get("i", 0.0))
        q_val = float(s_info.get("q", 0.0))
        iq_amp = float(np.hypot(i_val, q_val))
        
        if r_path <= 0.0:
            return
            
        path_lengths_list.append(r_path)
        amps_list.append(amp)
        iq_amps_list.append(iq_amp)

    raw_paths = np.array(path_lengths_list, dtype=np.float64)
    amps = np.array(amps_list, dtype=np.float64)
    iq_amps = np.array(iq_amps_list, dtype=np.float64)

    best_tx_key = active_sensors[0]
    best_residuals_sum = float('inf')
    p_pred = ekf.predict()

    for cand_tx_key in active_sensors:
        cand_tx_pos = SENSOR_POSITIONS[cand_tx_key]

        def bistatic_residuals(pos: np.ndarray) -> np.ndarray:
            res = []
            d_tx = float(np.linalg.norm(pos - cand_tx_pos))
            for idx, key in enumerate(active_sensors):
                if key == cand_tx_key:
                    h_val = 2.0 * d_tx
                else:
                    h_val = d_tx + float(np.linalg.norm(pos - SENSOR_POSITIONS[key]))
                res.append(h_val - raw_paths[idx])
            return np.array(res, dtype=np.float64)

        # Force the LM solver to exit early once it reaches 1mm precision
        # This prevents the solver from blocking the socket read loop
        opt_res: OptimizeResult = least_squares(
            bistatic_residuals, 
            p_pred,      # Use p_pred as the starting point (x0) for the local optimization
            method='lm',
            ftol=1e-3,      # Cost function tolerance
            xtol=1e-3,      # Step size tolerance
            max_nfev=20     # Maximum number of function evaluations
        )
        res_norm = float(np.linalg.norm(opt_res.fun))
        
        if res_norm < best_residuals_sum:
            best_residuals_sum = res_norm
            best_tx_key = cand_tx_key
            best_raw_pos = opt_res.x

    best_raw_pos = clamp_to_bounds(best_raw_pos)

    alpha = 0.5
    raw_weights = alpha * (amps / (np.max(amps) + 1e-5)) + (1.0 - alpha) * (iq_amps / (np.max(iq_amps) + 1e-5))
    total_w = float(np.sum(raw_weights))
    weights = raw_weights / total_w if total_w > 0 else np.ones(len(active_sensors)) / len(active_sensors)

    updated_pos = ekf.update(raw_paths, active_sensors, best_tx_key, weights)

    if visualizer is not None:
        visualizer.update_position(updated_pos)

def process_single_payload(payload: bytes) -> None:
    msg = hand_data_pb2.HandMsg()
    msg.ParseFromString(payload)

    sensor_data_store: dict[str, dict[str, Any]] = {}

    for data_msg in msg.data_wrapper.data_msgs_simple:
        source_str = source_map.get(data_msg.source, "Unknown")
        if source_str == "CH101_SENSOR2":
            continue
        decoded = decode_data(data_msg)
        if decoded:
            sensor_data_store.setdefault(source_str, {})["range"] = decoded[0]["range"][-1] if decoded[0]["range"] else 0.0
            sensor_data_store.setdefault(source_str, {})["amp"] = decoded[0]["amp"][-1] if decoded[0]["amp"] else 0.0

    for data_msg in msg.data_wrapper.data_msgs_iq:
        source_str = source_map.get(data_msg.source, "Unknown")
        if source_str == "CH101_SENSOR2":
            continue
        decoded = decode_data(data_msg)
        if decoded:
            sensor_data_store.setdefault(source_str, {})["i"] = decoded[0]["i"][-1] if decoded[0]["i"] else 0.0
            sensor_data_store.setdefault(source_str, {})["q"] = decoded[0]["q"][-1] if decoded[0]["q"] else 0.0

    process_sensor_frame(sensor_data_store)

def read(key: selectors.SelectorKey, mask: int) -> None:
    state: ConnectionState = key.data
    conn: socket.socket = key.fileobj

    try:
        chunk = conn.recv(4096)
        if not chunk:
            print("[INFO] Client gracefully disconnected.")
            close_connection(key)
            return

        state.recv_buffer.extend(chunk)

        while True:
            if len(state.recv_buffer) < 5:
                break 

            msg_len = struct.unpack("<I", state.recv_buffer[1:5])[0]
            if len(state.recv_buffer) < msg_len:
                break 

            payload = bytes(state.recv_buffer[5:msg_len])
            del state.recv_buffer[:msg_len]

            process_single_payload(payload)

    except Exception as e:
        print(f"\n[CRITICAL ERROR] Stream disrupted. Exception: {e}")
        close_connection(key)

def close_connection(key: selectors.SelectorKey) -> None:
    conn: socket.socket = key.fileobj
    try:
        sel.unregister(conn)
        conn.close()
    except Exception:
        pass

def accept(sock: socket.socket, mask: int) -> None:
    conn, addr = sock.accept()
    print(f"Connected by {addr}")
    conn.setblocking(False)
    state = ConnectionState(conn, addr)
    sel.register(conn, selectors.EVENT_READ, data=state)

def main() -> None:
    global visualizer
    visualizer = RealtimeVisualizer()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    sock.listen()
    sock.setblocking(False)
    print(f"Tracking Server active on {HOST}:{PORT}")

    sel.register(sock, selectors.EVENT_READ, data=None)

    try:
        while True:
            events = sel.select(timeout=0.01)
            for key, mask in events:
                if key.data is None:
                    accept(key.fileobj, mask)
                else:
                    read(key, mask)
            # Throttled the pause duration to allow tighter socket processing
            plt.pause(0.001)
    except KeyboardInterrupt:
        print("\nStopping Fingertip Tracking...")
    finally:
        sel.close()
        sock.close()

if __name__ == "__main__":
    main()