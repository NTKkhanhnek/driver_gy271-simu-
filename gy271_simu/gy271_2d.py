# py gy271_2d.py --debug
# py .\gy271_2d.py --debug
import argparse
import math
import re
import time
from collections import deque
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from matplotlib.animation import FuncAnimation

try:
    import serial
except ImportError:
    serial = None


VALUE_RE = r"[-+]?\d+(?:\.\d+)?"
COMPASS_RE = re.compile(
    rf"\bcompass_degree\s*[:=]\s*({VALUE_RE})",
    re.IGNORECASE,
)
RAW_HEADING_RE = re.compile(
    rf"\bheading_uncalibrated_raw\s*[:=]\s*({VALUE_RE})",
    re.IGNORECASE,
)
NUMBER_RE = re.compile(VALUE_RE)


@dataclass
class HeadingSample:
    compass_degree: float
    heading_uncalibrated_raw: float | None = None


class GY2712DReader:
    def __init__(self, port, baudrate, simulate=False):
        self.simulate = simulate
        self.started_at = time.monotonic()
        self.serial_port = None

        if not simulate:
            if serial is None:
                raise RuntimeError(
                    "Chua cai pyserial. Hay chay: py -m pip install pyserial matplotlib"
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

            sample = parse_heading_sample(raw)
            if sample is not None:
                return sample

            print(f"Bo qua dong khong doc duoc: {raw}")

    def read_raw_line(self):
        if self.simulate:
            sample = self._read_simulated_sample()
            return (
                f"compass_degree:{sample.compass_degree:.1f} "
                f"heading_uncalibrated_raw:{sample.heading_uncalibrated_raw:.1f}"
            )

        return self.serial_port.readline().decode("utf-8", errors="ignore").strip()

    def _read_simulated_sample(self):
        t = time.monotonic() - self.started_at
        clean = 180.0 + math.sin(t * 0.75) * 80.0 + math.sin(t * 0.18) * 35.0
        noisy = clean + math.sin(t * 11.0) * 18.0 + math.sin(t * 23.0) * 7.0
        time.sleep(0.03)
        return HeadingSample(normalize_degree(clean), normalize_degree(noisy))


def normalize_degree(value):
    return value % 360.0


def parse_heading_sample(line):
    compass_match = COMPASS_RE.search(line)
    raw_match = RAW_HEADING_RE.search(line)

    if compass_match and raw_match:
        return HeadingSample(
            normalize_degree(float(compass_match.group(1))),
            normalize_degree(float(raw_match.group(1))),
        )

    if compass_match:
        return HeadingSample(normalize_degree(float(compass_match.group(1))))

    if raw_match:
        return HeadingSample(normalize_degree(float(raw_match.group(1))))

    numbers = [float(value) for value in NUMBER_RE.findall(line)]
    if len(numbers) >= 2:
        return HeadingSample(normalize_degree(numbers[0]), normalize_degree(numbers[1]))
    if len(numbers) == 1:
        return HeadingSample(normalize_degree(numbers[0]))

    return None


def run_monitor(reader, seconds):
    deadline = time.monotonic() + seconds
    print(f"Doc raw data trong {seconds:.1f} giay.")

    while time.monotonic() < deadline:
        line = reader.read_raw_line()
        if not line:
            continue

        sample = parse_heading_sample(line)
        if sample is None:
            print(f"RAW: {line}")
        elif sample.heading_uncalibrated_raw is None:
            print(f"RAW: {line}  ->  compass_degree={sample.compass_degree:.1f} deg")
        else:
            print(
                f"RAW: {line}  ->  "
                f"compass_degree={sample.compass_degree:.1f} deg | "
                f"heading_uncalibrated_raw={sample.heading_uncalibrated_raw:.1f} deg"
            )


def run_visualizer(args):
    print(
        f"Dang chay {'du lieu gia lap' if args.simulate else args.port} "
        f"@ {args.baudrate} baud. Matplotlib backend: {plt.get_backend()}"
    )
    reader = GY2712DReader(args.port, args.baudrate, simulate=args.simulate)

    try:
        if args.monitor:
            run_monitor(reader, args.monitor_seconds)
            return

        start_time = time.monotonic()
        compass_points = deque(maxlen=args.samples)
        raw_points = deque(maxlen=args.samples)
        time_points = deque(maxlen=args.samples)
        last_sample = HeadingSample(0.0, 0.0)
        last_debug_at = 0.0

        fig, ax = plt.subplots(num="GY-271 2D heading graph")
        fig.subplots_adjust(left=0.09, right=0.98, bottom=0.12, top=0.90)
        print("Dang mo cua so 2D: GY-271 2D heading graph")

        compass_line, = ax.plot(
            [],
            [],
            color="#1f77b4",
            linewidth=2.4,
            label="compass_degree (calib + Kalman)",
        )
        raw_line, = ax.plot(
            [],
            [],
            color="#d62728",
            linewidth=1.5,
            alpha=0.82,
            label="heading_uncalibrated_raw (raw)",
        )

        ax.set_xlabel("Thoi gian (s)")
        ax.set_ylabel("Goc heading (degree)")
        ax.set_ylim(args.y_min, args.y_max)
        ax.yaxis.set_major_locator(MultipleLocator(args.y_major_step))
        ax.yaxis.set_minor_locator(MultipleLocator(args.y_minor_step))
        ax.grid(True, which="major", color="#d0d0d0", linewidth=0.85)
        ax.grid(True, which="minor", color="#eeeeee", linewidth=0.45)
        ax.legend(loc="upper right")

        def update(_frame):
            nonlocal last_sample, last_debug_at

            for _ in range(args.reads_per_frame):
                sample = reader.read_sample()
                if sample is not None:
                    last_sample = sample

            now = time.monotonic()
            elapsed = now - start_time
            time_points.append(elapsed)
            compass_points.append(last_sample.compass_degree)
            has_raw_value = last_sample.heading_uncalibrated_raw is not None
            raw_points.append(
                last_sample.heading_uncalibrated_raw if has_raw_value else math.nan
            )

            compass_line.set_data(time_points, compass_points)
            raw_line.set_data(time_points, raw_points)
            raw_line.set_visible(has_raw_value)

            if time_points:
                right = time_points[-1]
                left = max(0.0, right - args.window_seconds)
                ax.set_xlim(left, max(args.window_seconds, right))

            title = "GY-271 2D | " f"compass_degree={last_sample.compass_degree:6.1f} deg"
            if has_raw_value:
                title += f" | raw={last_sample.heading_uncalibrated_raw:6.1f} deg"
            ax.set_title(title)

            if args.debug and now - last_debug_at >= 0.5:
                debug = f"compass_degree={last_sample.compass_degree:.1f} deg"
                if has_raw_value:
                    debug += (
                        " | "
                        f"heading_uncalibrated_raw={last_sample.heading_uncalibrated_raw:.1f} deg"
                    )
                print(debug)
                last_debug_at = now

            return compass_line, raw_line

        animation = FuncAnimation(
            fig,
            update,
            interval=args.interval_ms,
            cache_frame_data=False,
            blit=False,
        )
        fig._gy271_2d_animation = animation
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


def build_parser():
    parser = argparse.ArgumentParser(
        description="Doc GY-271 qua COM va ve 2 bien heading tren do thi 2D."
    )
    parser.add_argument("--port", default="COM5", help="Cong COM dang nhan du lieu.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate serial.")
    parser.add_argument("--simulate", action="store_true", help="Chay du lieu gia lap de test 2D.")
    parser.add_argument("--interval-ms", type=int, default=50, help="Toc do cap nhat khung hinh.")
    parser.add_argument("--reads-per-frame", type=int, default=5, help="So dong serial doc moi khung.")
    parser.add_argument("--samples", type=int, default=500, help="So diem toi da giu tren do thi.")
    parser.add_argument("--window-seconds", type=float, default=20.0, help="Khoang thoi gian hien tren truc X.")
    parser.add_argument("--y-min", type=float, default=0.0, help="Gia tri nho nhat tren truc Y.")
    parser.add_argument("--y-max", type=float, default=360.0, help="Gia tri lon nhat tren truc Y.")
    parser.add_argument("--y-major-step", type=float, default=45.0, help="Do chia lon tren truc Y.")
    parser.add_argument("--y-minor-step", type=float, default=5.0, help="Do chia nho tren truc Y.")
    parser.add_argument("--debug", action="store_true", help="In gia tri dang ve ra terminal.")
    parser.add_argument("--monitor", action="store_true", help="Chi doc va in raw data, khong mo cua so 2D.")
    parser.add_argument("--monitor-seconds", type=float, default=8.0, help="So giay doc raw data khi dung --monitor.")
    return parser


if __name__ == "__main__":
    run_visualizer(build_parser().parse_args())
