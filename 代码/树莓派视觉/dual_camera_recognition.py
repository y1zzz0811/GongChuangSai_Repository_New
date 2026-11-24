"""双摄像头识别：摄像头 1 读二维码，摄像头 2 定位黑色编号圆环。

在 Thonny 打开本文件并运行。请与原来的三个脚本放在同一个目录。
空格保存两路原图和识别结果；Q/Esc 退出。--no-uart 可关闭二维码串口发送。
"""
import argparse
from collections import Counter
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import numpy as np
import serial

from dual_camera_preview import Camera, command, fit_size, usb_devices
from qr_camera1 import (
    UART_BAUD, UART_DEVICE, decode_frame, label, valid_target_code,
)
from target_center_camera1 import cross, text
from numbered_ring_detector import NumberedRingDetector
from ring_metric import RingMetric, image_center
from route2_vision import RawTarget, RingLink
from stacking_vision import DEFAULT_TOP_SEARCH_MM, StackVision


# 顺序与原来双摄预览一致：第一台 /dev/video0，第二台 /dev/video2。
# 若物理接线相反，交换两个路径。也可以填写 /dev/v4l/by-path/ 下的稳定路径。
CAMERA_DEVICES = ("/dev/video0", "/dev/video2")
# 保留各识别脚本已经使用的分辨率和 YUYV 色彩格式；完整画面只缩放、不裁剪。
CAMERA_MODES = (("YUYV", 640, 480, 10), ("YUYV", 800, 600, 10))
ENABLE_UART = True

# Reuse only the identity, never old geometry or millimeter measurements.
RING_TRACK_MAX_UNREAD_FRAMES = 2
RING_TRACK_MAX_AGE_S = 0.6


@dataclass
class RingTrack:
    center: np.ndarray
    radius: float
    hits: int
    unread: int
    read_at: float
    seen_at: float


def track_ring_identities(rings, history, now):
    """Strict acquisition, short unambiguous continuation, immediate loss reset."""
    counts = Counter(ring.number for ring in rings)
    current, observations = {}, []

    def nearby(ring, track, fraction=0.35):
        return (ring.radius > 0 and track.radius > 0
                and abs(ring.radius / track.radius - 1) <= 0.15
                and np.linalg.norm(np.asarray(ring.center) - track.center)
                < max(5, ring.radius * fraction))

    for index, ring in enumerate(rings):
        number, inferred = ring.number, False
        old = history.get(number)
        if number is None:
            eligible = []
            for candidate, track in history.items():
                if (counts[candidate] or track.hits < 3
                        or track.unread >= RING_TRACK_MAX_UNREAD_FRAMES
                        or not 0 <= now - track.read_at <= RING_TRACK_MAX_AGE_S
                        or not nearby(ring, track, 0.20)):
                    continue
                # Neither two old IDs nor two new circles may match one track.
                matches = [i for i, other in enumerate(rings) if nearby(other, track, 0.20)]
                if matches == [index]:
                    eligible.append((candidate, track))
            if len(eligible) != 1:
                observations.append((ring, False, False))
                continue
            number, old = eligible[0]
            ring = replace(ring, number=number)
            inferred = True
        elif counts[number] != 1:
            observations.append((ring, False, False))
            continue

        continuous = (old is not None and nearby(ring, old)
                      and 0 <= now - old.seen_at <= RING_TRACK_MAX_AGE_S)
        hits = min(old.hits + 1, 3) if continuous else 1
        current[number] = RingTrack(np.asarray(ring.center), ring.radius, hits,
                                   old.unread + 1 if inferred else 0,
                                   old.read_at if inferred else now, now)
        observations.append((ring, hits >= 3, inferred))
    history.clear()
    history.update(current)
    return observations


class QRTransmitter:
    """保留原 QR 协议和三帧稳定、防重复发送逻辑。"""

    def __init__(self, port):
        self.port = port
        self.candidate = None
        self.candidate_frames = 0
        self.last_sent = None
        self.missing_since = None

    def update(self, values, now):
        values = tuple(sorted(set(values)))
        if len(values) == 1 and valid_target_code(values[0]):
            value = values[0]
            self.missing_since = None
            if value == self.candidate:
                self.candidate_frames += 1
            else:
                self.candidate, self.candidate_frames = value, 1
            if self.candidate_frames >= 3 and value != self.last_sent:
                packet = ("QR," + value + "\n").encode("ascii")
                if self.port.write(packet) != len(packet):
                    raise OSError("串口没有完整发送二维码数据")
                self.port.flush()
                self.last_sent = value
                print("串口已发送：", repr(packet), flush=True)
        else:
            self.candidate, self.candidate_frames = None, 0
            if not values:
                if self.missing_since is None:
                    self.missing_since = now
                elif now - self.missing_since >= 1.0:
                    self.last_sent = None
            else:
                self.missing_since = None


def qr_result(frame, detector):
    results = decode_frame(detector, frame)
    display = frame.copy()
    for number, (value, polygon) in enumerate(results, 1):
        points = np.rint(polygon).astype(np.int32)
        cv2.polylines(display, [points], True, (0, 255, 0), 2, cv2.LINE_AA)
        center = tuple(np.rint(polygon.mean(axis=0)).astype(int))
        cv2.drawMarker(display, center, (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        x = max(0, min(int(points[:, 0].min()), frame.shape[1] - 100))
        y = max(50, int(points[:, 1].min()) - 10)
        label(display, f"QR {number}", (x, y))
        preview = value if value.isascii() else "Text decoded - see result below"
        preview = preview.replace("\n", " ").replace("\r", " ")[:65]
        label(display, preview, (8, max(40, frame.shape[0] - 15 - (number-1)*22)))
    label(display, f"Camera 1 | QR codes: {len(results)}", (8, 25))
    return display, results


def circle_result(frame, detector, history, metric=None, now=None, requested_ring=None):
    height, width = frame.shape[:2]
    origin = np.array(image_center((width, height)))
    display = frame.copy()
    now = time.monotonic() if now is None else now
    if requested_ring is None:
        rings = detector.detect(frame)
    else:
        track = history.get(requested_ring)
        hint = None
        if (track and track.hits >= 3 and 0 <= now-track.seen_at <= RING_TRACK_MAX_AGE_S
                and 0 <= now-track.read_at <= RING_TRACK_MAX_AGE_S):
            hint = track.center, track.radius

        def usable(ring):
            return metric.offset(ring, frame.shape) is not None

        rings = detector.detect(frame, target=requested_ring, hint=hint,
                                usable=usable if metric is not None else None)
    counts = Counter(ring.number for ring in rings)
    found = {}
    confirmed_color = (255, 235, 80)
    pending_color = (0, 180, 255)
    for ring, confirmed, inferred in track_ring_identities(rings, history, now):
        number = ring.number
        measured = np.array(ring.center)
        if number is None or (not inferred and counts[number] != 1):
            cross(display, measured, pending_color)
            continue
        color = confirmed_color if confirmed else pending_color
        cross(display, measured, color)
        px, py = np.rint(measured).astype(int)
        name = f"RING {number}" + (" TRACK" if inferred else "")
        text(display, name if confirmed else "CHECKING",
             (max(0, min(px+12, width-105)), max(88, min(py-12, height-30))), color, 0.44)
        if confirmed:
            found[str(number)] = {"center": measured, "offset": measured-origin,
                                  "identity_source": "tracked" if inferred else "digit"}
            if metric is not None:
                found[str(number)]["offset_mm"] = metric.offset(ring, frame.shape)
    for row, number in enumerate((1, 2, 3)):
        target = found.get(str(number))
        if target is None:
            state = "CHECKING" if counts[number] == 1 else "NOT CONFIRMED"
            text(display, f"{number}: {state}", (8, 22+row*23), pending_color, 0.44)
            continue
        if metric is None:
            x, y = target["center"]
            dx, dy = target["offset"]
            line = f"{number}: ({x:.1f},{y:.1f}) dx={dx:+.1f} dy={dy:+.1f}"
        elif target["offset_mm"] is None:
            line = f"{number}: MM UNAVAILABLE"
        else:
            forward, left = target["offset_mm"]
            line = f"{number}: Fwd {forward:+.1f} mm  Left {left:+.1f} mm"
        text(display, line, (8, 22+row*23), confirmed_color, 0.44)
    # 最后绘制中心十字，即使没有识别到圆圈、圆心与中心重合，也保留中心标识。
    cross(display, origin, (0, 255, 0), 18)
    text(display, "IMAGE CENTER", (int(origin[0])+22, int(origin[1])-15), scale=0.4)
    legend = "Camera 2 | dx: right + | dy: down +" if metric is None else "Camera 2 | +Fwd: forward | +Left: left"
    text(display, legend, (8, height-12), scale=0.42)
    return display, found


class RecognitionCamera:
    """每路独立采集和识别，只保留最新画面，Tk 主线程负责显示。"""

    def __init__(self, index, device, mode, enable_uart, ring_metric=None, vision_camera=None,
                 top=None, raw=None, vision_debug=False):
        self.index, self.device = index, device
        self.enable_uart = enable_uart and index == 0
        self.ring_metric = ring_metric
        self.vision_camera = vision_camera
        self.top_detector, self.raw_detector = top, raw
        self.vision_debug = vision_debug
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.capture = Camera(device, mode)
        self.result = None
        self.error = None
        self.processed = 0
        self.uart_status = "串口已关闭"
        self._vision_request = (None, None)
        self._vision_status = ''
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set_vision_request_mode(self, mode, requested_ring=None):
        with self.lock:
            self._vision_request = (mode, requested_ring)

    def vision_request(self):
        with self.lock:
            return self._vision_request

    def set_vision_status(self, status):
        with self.lock:
            self._vision_status = status

    def vision_status(self):
        with self.lock:
            return self._vision_status

    def run(self):
        port = None
        try:
            if self.enable_uart:
                try:
                    port = serial.Serial(UART_DEVICE, UART_BAUD, timeout=0.1, write_timeout=1,
                                         xonxoff=False, rtscts=False, dsrdtr=False)
                    self.uart_status = f"串口 {UART_DEVICE}，{UART_BAUD} 8N1"
                except (serial.SerialException, OSError) as error:
                    self.uart_status = f"串口不可用，仅识别：{error}"
                    print(self.uart_status, flush=True)
            transmitter = QRTransmitter(port) if port is not None else None
            link = (RingLink(port, self.vision_camera, self.top_detector, self.raw_detector, self.vision_debug)
                    if port is not None and self.vision_camera else None)
            detector = cv2.QRCodeDetector() if self.index == 0 else NumberedRingDetector()
            ring_history = {}
            previous_values, last_print = (), 0.0
            previous_sequence = -1
            last_processed = time.monotonic()
            while not self.stop_event.is_set():
                started = time.monotonic()
                if link is not None:
                    try:
                        link.poll(started)
                    except (serial.SerialException, OSError) as error:
                        self.uart_status = f"视觉串口失败，仅识别：{error}"
                        if link is not None:
                            link._clear_pending()
                        transmitter = link = None
                        port.close()
                        port = None
                frame, sequence, capture_status, captured_at = self.capture.timed_snapshot()
                if frame is None or sequence == previous_sequence:
                    if capture_status.startswith("错误："):
                        raise RuntimeError(capture_status)
                    if started - last_processed > 5:
                        raise RuntimeError("超过 5 秒没有新画面，请检查摄像头连接")
                    self.stop_event.wait(0.01)
                    continue
                previous_sequence = sequence
                if self.index == 0:
                    display, codes = qr_result(frame, detector)
                    values = tuple(sorted({value for value, _ in codes}))
                    if values and (values != previous_values or started-last_print >= 5):
                        print("二维码识别：", values, flush=True)
                        last_print = started
                    previous_values = values
                    if transmitter is not None:
                        try:
                            transmitter.update(values, started)
                        except (serial.SerialException, OSError) as error:
                            self.uart_status = f"串口发送失败，仅识别：{error}"
                            print(self.uart_status, flush=True)
                            if link is not None:
                                link._clear_pending()
                            transmitter = link = None
                            port.close()
                            port = None
                    info = "；".join(values)[:300] if values else "尚未识别到二维码"
                    details = f"{self.uart_status}；{info}"
                else:
                    request_mode, requested_ring = self.vision_request()
                    if request_mode in ("TOP", "RAW"):
                        # RingLink owns TOP/RAW work during a tagged request.
                        # Avoid running the unrelated numbered-ring detector
                        # on the same frame while it is doing that work.
                        display, circles = frame.copy(), {}
                        text(display, f"{request_mode} REQUEST ACTIVE", (10, 28),
                             (0, 255, 255), 0.55)
                        text(display, self.vision_status(), (10, 52), (0, 255, 255), 0.42)
                        cross(display, ((frame.shape[1]-1)/2, (frame.shape[0]-1)/2),
                              (0, 255, 0), 18)
                    else:
                        display, circles = circle_result(frame, detector, ring_history, self.ring_metric,
                                                         now=captured_at, requested_ring=requested_ring)
                    h, w = frame.shape[:2]
                    details = f"图像中心 ({(w-1)/2:g}, {(h-1)/2:g})；"
                    if request_mode in ("TOP", "RAW"):
                        details += self.vision_status() or f"{request_mode} 请求处理中"
                    else:
                        details += "、".join(sorted(circles)) + " 号圆环已确认" if circles else "等待编号圆环确认"
                elapsed = max(time.monotonic() - started, 0.001)
                self.processed += 1
                with self.lock:
                    self.result = dict(raw=frame, display=display, sequence=sequence,
                                       captured_at=captured_at, rings=circles if self.index == 1 else {},
                                       status=f"{capture_status}；识别用时 {elapsed*1000:.0f} ms",
                                       details=details, timestamp=time.monotonic())
                last_processed = time.monotonic()
                self.stop_event.wait(max(0, 0.1 - elapsed))
        except Exception as error:
            with self.lock:
                self.error = str(error)
                self.result = None
            print(f"摄像头 {self.index+1} 错误：{error}", flush=True)
        finally:
            self.capture.stop_event.set()
            if port is not None:
                port.close()

    def snapshot(self):
        with self.lock:
            return self.result, self.error

    def stop(self):
        self.stop_event.set()
        self.capture.stop_event.set()

    def join(self):
        self.thread.join(timeout=2)
        self.capture.thread.join(timeout=1)


class App:
    def __init__(self, root, cameras, duration=0):
        self.root, self.cameras = root, cameras
        self.closed = False
        self.after_id = None
        self.duration_id = None
        self.photos = [None, None]
        self.render_keys = [None, None]
        self.labels, self.statuses, self.details = [], [], []
        root.title("双摄像头识别：1 二维码 / 2 编号圆环")
        root.geometry("1280x720")
        root.minsize(800, 480)
        root.protocol("WM_DELETE_WINDOW", self.close)
        for key in ("<Escape>", "q", "Q"):
            root.bind(key, lambda event: self.close())
        root.bind("<space>", lambda event: self.save())
        header = ttk.Frame(root, padding=8)
        header.pack(fill="x")
        directions = ("毫米偏差以前、左为正" if cameras[1].ring_metric else "像素 dx 向右为正，dy 向下为正")
        ttk.Label(header, text=f"摄像头 2 绿色十字为图像中心。{directions}。").pack(side="left")
        ttk.Button(header, text="退出", command=self.close).pack(side="right")
        ttk.Button(header, text="拍照（空格）", command=self.save).pack(side="right", padx=5)
        body = ttk.Frame(root)
        body.pack(fill="both", expand=True)
        body.rowconfigure(0, weight=1)
        for index, camera in enumerate(cameras):
            body.columnconfigure(index, weight=1, uniform="camera")
            panel = ttk.Frame(body, padding=5)
            panel.grid(row=0, column=index, sticky="nsew")
            role = "二维码识别" if index == 0 else "黑色编号圆环 + 中心十字"
            ttk.Label(panel, text=f"摄像头 {index+1} — {role} — {camera.device}", anchor="center").pack(fill="x")
            status = tk.StringVar(value="正在打开摄像头…")
            ttk.Label(panel, textvariable=status, anchor="center", wraplength=580).pack(fill="x")
            image = tk.Label(panel, background="#151515", foreground="white", text="等待画面…")
            image.pack(fill="both", expand=True)
            details = tk.StringVar(value="")
            ttk.Label(panel, textvariable=details, wraplength=580, anchor="w").pack(fill="x")
            self.labels.append(image)
            self.statuses.append(status)
            self.details.append(details)
        self.notice = tk.StringVar(value="空格保存两路原图和识别结果，Q / Esc 退出。")
        ttk.Label(root, textvariable=self.notice, padding=6).pack(fill="x")
        if duration > 0:
            self.duration_id = root.after(round(duration*1000), self.close)
        self.refresh()

    def refresh(self):
        if self.closed:
            return
        for index, camera in enumerate(self.cameras):
            result, error = camera.snapshot()
            if result is None or time.monotonic()-result["timestamp"] > 5:
                message = f"错误：{error}" if error else "等待新画面…"
                self.statuses[index].set(message)
                self.details[index].set("")
                self.photos[index] = None
                self.render_keys[index] = None
                self.labels[index].configure(image="", text=message)
                continue
            self.statuses[index].set(result["status"])
            self.details[index].set(result["details"])
            label_widget = self.labels[index]
            box_w, box_h = label_widget.winfo_width(), label_widget.winfo_height()
            if min(box_w, box_h) < 10:
                continue
            render_key = (result["sequence"], box_w, box_h)
            if self.render_keys[index] == render_key:
                continue
            frame = result["display"]
            h, w = frame.shape[:2]
            new_w, new_h = fit_size(w, h, box_w, box_h)
            display = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            ppm = f"P6\n{new_w} {new_h}\n255\n".encode() + rgb.tobytes()
            self.photos[index] = tk.PhotoImage(data=ppm, format="PPM")
            label_widget.configure(image=self.photos[index], text="")
            self.render_keys[index] = render_key
        self.after_id = self.root.after(70, self.refresh)

    def save(self):
        folder = Path.home()/"Pictures"/"dual_recognition"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        saved = 0
        try:
            folder.mkdir(parents=True, exist_ok=True)
            for index, camera in enumerate(self.cameras, 1):
                result, _ = camera.snapshot()
                if result is None or time.monotonic()-result["timestamp"] > 5:
                    continue
                for suffix, key in (("raw", "raw"), ("result", "display")):
                    path = folder/f"{stamp}_camera{index}_{suffix}.jpg"
                    if not cv2.imwrite(str(path), result[key]):
                        raise OSError(f"保存失败：{path}")
                    saved += 1
            self.notice.set(f"已保存 {saved} 张照片：{folder}" if saved else "还没有可保存的画面。")
            print(self.notice.get(), flush=True)
        except OSError as error:
            messagebox.showerror("保存照片", str(error), parent=self.root)

    def close(self):
        if self.closed:
            return
        self.closed = True
        for callback in (self.after_id, self.duration_id):
            if callback is not None:
                self.root.after_cancel(callback)
        for camera in self.cameras:
            camera.stop()
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-uart", action="store_true", help="只识别，不打开串口、不发送数据")
    parser.add_argument("--duration", type=float, default=0, help="运行指定秒数后退出；默认持续运行")
    parser.add_argument("--headless", action="store_true", help="无窗口检查双路采集识别，必须配合 --duration")
    parser.add_argument("--ring-metric", type=Path, help="当前固定观察姿态的圆环毫米标定 JSON")
    parser.add_argument("--route2-vision", action="store_true", help="使用现有串口应答路线2视觉请求")
    parser.add_argument("--top-search-mm", type=float, default=None,
                        help="兼容旧参数；统一入口TOP不再按此值筛选回传")
    parser.add_argument("--vision-debug", action="store_true", help="在终端输出定位请求、TOP检测状态和回包")
    parser.add_argument("--pickup-test", action="store_true",
                        help="单件抓取预设：启用串口视觉、图像中心顶面零点和定位日志")
    parser.add_argument("--raw-config", type=Path, help="原料区抓取窗口、颜色和时序 JSON；夹爪零点固定为图像中心")
    args = parser.parse_args(argv)
    if args.pickup_test:
        args.route2_vision = args.vision_debug = True
        if args.ring_metric is None:
            args.ring_metric = Path(__file__).with_name("ring_metric_Z184_image_center.json")
    if args.headless and args.duration <= 0:
        parser.error("--headless 需要 --duration 指定正数秒数")
    if args.route2_vision and (not args.ring_metric or args.no_uart or not ENABLE_UART):
        parser.error("--route2-vision 需要启用串口并指定 --ring-metric")
    if args.raw_config and not args.route2_vision:
        parser.error("--raw-config 需要 --route2-vision")
    top_search_mm = DEFAULT_TOP_SEARCH_MM  # Legacy StackVision map size only.
    cv2.setNumThreads(2)
    cameras = []
    root = None
    app = None
    try:
        metric = RingMetric.load(args.ring_metric) if args.ring_metric else None
        top = (StackVision(metric, 10, 60, 30, top_search_mm, min_top_score=5.0)
               if args.route2_vision else None)
        if top is not None:
            u, v = metric.zero_pixel
            print(f"视觉联调：夹爪零点=图像中心 ({u:g}, {v:g})，统一TOP回传仅受协议每轴±500mm限制", flush=True)
        raw = RawTarget.load(metric, args.raw_config) if args.raw_config else None
        devices = CAMERA_DEVICES or usb_devices(command("v4l2-ctl", "--list-devices"))[:2]
        if len(devices) != 2 or len({Path(d).resolve() for d in devices}) != 2:
            raise RuntimeError("需要两个不同的 USB 摄像头，请检查 CAMERA_DEVICES")
        for index, device in enumerate(devices):
            if not Path(device).exists():
                raise RuntimeError(f"没有检测到摄像头 {index+1}：{device}")
        for index in (1, 0):
            device, mode = devices[index], CAMERA_MODES[index]
            vision_camera = cameras[0] if index == 0 and args.route2_vision else None
            cameras.insert(0, RecognitionCamera(index, device, mode, ENABLE_UART and not args.no_uart,
                                                ring_metric=metric if index == 1 else None,
                                                vision_camera=vision_camera,
                                                top=top if index == 0 else None,
                                                raw=raw if index == 0 else None,
                                                vision_debug=args.vision_debug))
            print(f"摄像头 {index+1}：{device}，请求 {mode[1]}x{mode[2]} {mode[0]} {mode[3]} FPS", flush=True)
        if args.headless:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                if any(camera.snapshot()[1] for camera in cameras):
                    break
                time.sleep(0.1)
        else:
            root = tk.Tk()
            app = App(root, cameras, args.duration)
            root.mainloop()
        for camera in cameras:
            result, error = camera.snapshot()
            print(f"摄像头 {camera.index+1} 已处理 {camera.processed} 帧；错误：{error}", flush=True)
            if error or camera.processed == 0:
                raise RuntimeError(error or f"摄像头 {camera.index+1} 未获取到有效画面")
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"运行失败：{error}", flush=True)
        if root is not None and (app is None or not app.closed):
            messagebox.showerror("双摄像头识别", str(error), parent=root)
        return 1
    finally:
        if app is not None:
            app.close()
        elif root is not None:
            root.destroy()
        for camera in cameras:
            camera.stop()
        for camera in cameras:
            camera.join()


if __name__ == "__main__":
    raise SystemExit(main())
