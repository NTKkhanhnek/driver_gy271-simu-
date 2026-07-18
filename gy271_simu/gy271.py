# py gy271.py --debug
# py .\gy271.py --debug
import argparse
import math
import re
import time
from collections import deque
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

try:
    import serial
except ImportError:
    serial = None


NUMBER_RE = re.compile(r"[-+]?\d+(?:\.\d+)?")
AXIS_RE = re.compile(
    r"\b([xyz])\s*[:=]\s*([-+]?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)
HEADING_RE = re.compile(
    r"\bheading\s*[:=]\s*([-+]?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


@dataclass
class MagnetometerSample:
    x: float
    y: float
    z: float

    @property
    def vector(self):
        return np.array([self.x, self.y, self.z], dtype=float)


class GY271Reader:
    def __init__(self, port, baudrate, simulate=False):
        self.simulate = simulate
        self.started_at = time.monotonic()
        self.serial_port = None

        if not simulate:
            if serial is None:
                raise RuntimeError(
                    "Chua cai pyserial. Hay chay: py -m pip install pyserial matplotlib numpy"
                )

            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.05,
            )
            self.serial_port.reset_input_buffer()

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def read_sample(self):
        if self.simulate:
            return self._read_simulated_sample()

        while True:
            raw = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                return None

            sample = parse_sample(raw)
            if sample is not None:
                return sample

            print(f"Bo qua dong khong doc duoc: {raw}")

    def read_raw_line(self):
        if self.simulate:
            sample = self._read_simulated_sample()
            return f"X:{sample.x:.1f} Y:{sample.y:.1f} Z:{sample.z:.1f}"

        return self.serial_port.readline().decode("utf-8", errors="ignore").strip()

    def _read_simulated_sample(self):
        t = time.monotonic() - self.started_at
        heading = t * 0.8
        wobble = math.sin(t * 1.7) * 0.35
        noise = np.random.normal(0, 0.04, 3)
        vector = np.array(
            [
                math.cos(heading) * math.cos(wobble),
                math.sin(heading) * math.cos(wobble),
                math.sin(wobble),
            ]
        )
        time.sleep(0.03)
        vector = (vector + noise) * 250.0
        return MagnetometerSample(vector[0], vector[1], vector[2])


def parse_sample(line):
    axis_values = {}
    for axis, value in AXIS_RE.findall(line):
        axis_values[axis.lower()] = float(value)

    if {"x", "y", "z"}.issubset(axis_values):
        return MagnetometerSample(axis_values["x"], axis_values["y"], axis_values["z"])

    heading_match = HEADING_RE.search(line)
    if heading_match:
        heading = math.radians(float(heading_match.group(1)))
        return MagnetometerSample(math.cos(heading), -math.sin(heading), 0.0)

    numbers = [float(value) for value in NUMBER_RE.findall(line)]
    if len(numbers) < 3:
        return None
    return MagnetometerSample(numbers[0], numbers[1], numbers[2])


def heading_degrees(vector):
    heading = math.degrees(math.atan2(-vector[1], vector[0]))
    return (heading + 360.0) % 360.0


def normalize(vector):
    length = float(np.linalg.norm(vector))
    if length < 1e-9:
        return np.array([0.0, 0.0, 0.0])
    return vector / length


def collect_calibration(reader, seconds):
    print(f"Xoay cam bien cham theo nhieu huong trong {seconds:.1f} giay de can chinh...")
    deadline = time.monotonic() + seconds
    samples = []

    while time.monotonic() < deadline:
        sample = reader.read_sample()
        if sample is not None:
            samples.append(sample.vector)

    if not samples:
        print("Khong thu duoc mau can chinh, dung offset = 0.")
        return np.array([0.0, 0.0, 0.0])

    data = np.vstack(samples)
    mins = data.min(axis=0)
    maxs = data.max(axis=0)
    offset = (mins + maxs) / 2.0
    print(f"Offset: X={offset[0]:.1f}, Y={offset[1]:.1f}, Z={offset[2]:.1f}")
    return offset


def run_visualizer(args):
    print(
        f"Dang chay {'du lieu gia lap' if args.simulate else args.port} "
        f"@ {args.baudrate} baud. Matplotlib backend: {plt.get_backend()}"
    )
    reader = GY271Reader(args.port, args.baudrate, simulate=args.simulate)
    offset = np.array([args.offset_x, args.offset_y, args.offset_z], dtype=float)

    try:
        if args.monitor:
            run_monitor(reader, args.monitor_seconds)
            return

        if args.calibrate_seconds > 0:
            offset = collect_calibration(reader, args.calibrate_seconds)

        samples = deque(maxlen=args.trail)
        last_sample = MagnetometerSample(1.0, 0.0, 0.0)
        last_debug_at = 0.0

        fig = plt.figure("GY-271 3D heading visualizer")
        ax = fig.add_subplot(111, projection="3d")
        fig.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.92)
        print("Dang mo cua so 3D: GY-271 3D heading visualizer")

        def setup_axes():
            ax.clear()
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            ax.set_box_aspect((1, 1, 1))
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.view_init(elev=24, azim=38)

            limit = 1.0
            ax.plot([-limit, limit], [0, 0], [0, 0], color="#555555", linewidth=0.8)
            ax.plot([0, 0], [-limit, limit], [0, 0], color="#555555", linewidth=0.8)
            ax.plot([0, 0], [0, 0], [-limit, limit], color="#555555", linewidth=0.8)
            ax.text(1.06, 0, 0, "+X", color="#444444")
            ax.text(0, 1.06, 0, "+Y", color="#444444")
            ax.text(0, 0, 1.06, "+Z", color="#444444")

        def update(_frame):
            nonlocal last_sample, last_debug_at

            for _ in range(args.reads_per_frame):
                sample = reader.read_sample()
                if sample is not None:
                    last_sample = sample

            vector = last_sample.vector - offset
            unit = normalize(vector)
            samples.append(unit)
            heading = heading_degrees(unit)
            now = time.monotonic()

            if args.debug and now - last_debug_at >= 0.5:
                print(
                    f"raw X={last_sample.x:.1f} Y={last_sample.y:.1f} Z={last_sample.z:.1f} "
                    f"| heading={heading:.1f} deg"
                )
                last_debug_at = now

            setup_axes()

            if len(samples) > 1:
                trail = np.vstack(samples)
                ax.plot(
                    trail[:, 0],
                    trail[:, 1],
                    trail[:, 2],
                    color="#4c78a8",
                    linewidth=1.8,
                    alpha=0.65,
                )

            ax.quiver(
                0,
                0,
                0,
                unit[0],
                unit[1],
                unit[2],
                length=0.9,
                normalize=False,
                color="#e45756",
                linewidth=3.0,
                arrow_length_ratio=0.18,
            )

            ax.quiver(
                0,
                0,
                0,
                unit[0],
                unit[1],
                0,
                length=0.75,
                normalize=True,
                color="#54a24b",
                linewidth=2.2,
                arrow_length_ratio=0.16,
            )

            ax.set_title(
                "GY-271 | Heading: "
                f"{heading:6.1f} deg | Raw: X={last_sample.x:.0f} Y={last_sample.y:.0f} Z={last_sample.z:.0f}",
                pad=14,
            )

        animation = FuncAnimation(fig, update, interval=args.interval_ms, cache_frame_data=False)
        fig._gy271_animation = animation
        update(0)
        bring_window_to_front(fig)
        plt.show(block=True)
    finally:
        reader.close()


def bring_window_to_front(fig):
    manager = plt.get_current_fig_manager()
    window = getattr(manager, "window", None)
    if window is None:
        return

    try:
        window.wm_attributes("-topmost", 1)
        window.update()
        window.wm_attributes("-topmost", 0)
        window.lift()
        window.focus_force()
    except Exception:
        pass


def run_monitor(reader, seconds):
    deadline = time.monotonic() + seconds
    print(f"Doc raw data trong {seconds:.1f} giay. Hay xoay cam bien de xem so co doi khong.")

    while time.monotonic() < deadline:
        line = reader.read_raw_line()
        if not line:
            continue

        sample = parse_sample(line)
        if sample is None:
            print(f"RAW: {line}")
        else:
            print(f"RAW: {line}  ->  X={sample.x:.1f} Y={sample.y:.1f} Z={sample.z:.1f}")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Doc cam bien GY-271 qua COM va mo phong vector huong lech 3D."
    )
    parser.add_argument("--port", default="COM5", help="Cong COM dang nhan du lieu.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate serial.")
    parser.add_argument("--simulate", action="store_true", help="Chay du lieu gia lap de test 3D.")
    parser.add_argument("--interval-ms", type=int, default=50, help="Toc do cap nhat khung hinh.")
    parser.add_argument("--reads-per-frame", type=int, default=5, help="So dong serial doc moi khung.")
    parser.add_argument("--trail", type=int, default=80, help="Do dai vet di cua vector.")
    parser.add_argument("--debug", action="store_true", help="In gia tri dang ve ra terminal.")
    parser.add_argument("--monitor", action="store_true", help="Chi doc va in raw data, khong mo cua so 3D.")
    parser.add_argument("--monitor-seconds", type=float, default=8.0, help="So giay doc raw data khi dung --monitor.")
    parser.add_argument("--offset-x", type=float, default=0.0, help="Offset can chinh truc X.")
    parser.add_argument("--offset-y", type=float, default=0.0, help="Offset can chinh truc Y.")
    parser.add_argument("--offset-z", type=float, default=0.0, help="Offset can chinh truc Z.")
    parser.add_argument(
        "--calibrate-seconds",
        type=float,
        default=0.0,
        help="So giay tu can chinh offset min/max truoc khi ve.",
    )
    return parser


if __name__ == "__main__":
    run_visualizer(build_parser().parse_args())
