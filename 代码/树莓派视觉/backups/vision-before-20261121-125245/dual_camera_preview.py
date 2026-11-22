"""在树莓派 Thonny 中运行：两台 USB 摄像头完整画面预览。

最高分辨率不等于更宽视角；实际视角由镜头和摄像头输出模式决定。
本程序不裁剪，只等比例缩小画面以适应窗口。
依赖：python3-opencv、python3-tk、v4l-utils。
"""
import re
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import cv2

# None 表示自动选择两个不同 USB 设备。
# 也可手动填写 ["/dev/video0", "/dev/video2"]。
CAMERA_DEVICES = None
TARGET_FPS = 10
PREFER_YUYV = False  # 最高分辨率双路使用 MJPG，减少 USB 传输带宽。
CAPTURE_SIZE = None  # None 自动选最高分辨率；也可改成 (800, 600) 比较 4:3 视角。


def command(*args):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT, timeout=10)


def usb_devices(text):
    devices = []
    for block in re.split(r"\n\s*\n", text.strip()):
        if "(usb-" not in block.lower():
            continue
        nodes = re.findall(r"/dev/video\d+", block)
        if nodes:
            devices.append(nodes[0])  # 每台设备只取首个视频节点，避免把元数据节点当第二台。
    return devices


def modes(text):
    result = []
    fourcc = None
    current = None
    for line in text.splitlines():
        match = re.search(r"\[\d+\]: '(.{4})'", line)
        if match:
            fourcc = match.group(1)
        match = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if match and fourcc:
            current = [fourcc, int(match.group(1)), int(match.group(2)), []]
            result.append(current)
        match = re.search(r"\(([\d.]+) fps\)", line)
        if match and current is not None:
            current[3].append(float(match.group(1)))
    return result


def highest_mode(text):
    available = modes(text)
    if not available:
        raise RuntimeError("没有找到离散分辨率，请提供 v4l2-ctl --list-formats-ext 的输出。")
    matching = [m for m in available if (m[1], m[2]) == CAPTURE_SIZE]
    if matching:
        available = matching
    # 分辨率优先，再选择图像格式；帧率只能从设备支持的值中选择。
    preferred = "YUYV" if PREFER_YUYV else "MJPG"
    mode = max(available, key=lambda m: (m[1]*m[2], m[0] == preferred))
    rates = mode[3]
    slower = [rate for rate in rates if rate <= TARGET_FPS + .01]
    fps = max(slower) if slower else min(rates) if rates else TARGET_FPS
    return mode[0], mode[1], mode[2], fps


def fit_size(width, height, box_width, box_height):
    scale = min(box_width / width, box_height / height)
    return max(1, round(width*scale)), max(1, round(height*scale))


class Camera:
    def __init__(self, device, mode):
        self.device, self.mode = device, mode
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame = None
        self.sequence = 0
        self.captured_at = 0.0
        self.status = "正在打开摄像头…"
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError("无法打开，请退出其他占用摄像头的程序。")
            fourcc, width, height, fps = self.mode
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            last = time.monotonic()
            count = 0
            measured_fps = 0.0
            while not self.stop_event.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("读取失败，检查连接、USB 带宽及供电。")
                count += 1
                now = time.monotonic()
                if now - last >= 1:
                    measured_fps = count / (now-last)
                    last, count = now, 0
                h, w = frame.shape[:2]
                actual = f"{w}×{h}，采集 {measured_fps:.1f} FPS"
                if (w, h) != (width, height):
                    actual += f"（请求 {width}×{height} 未被设备采用）"
                with self.lock:
                    self.frame = frame
                    self.sequence += 1
                    self.captured_at = now
                    self.status = actual
        except Exception as error:
            with self.lock:
                self.frame = None
                self.status = f"错误：{error} 摄像头重新连接后请退出并重新运行。"
        finally:
            if cap is not None:
                cap.release()

    def snapshot(self):
        with self.lock:
            return self.frame, self.sequence, self.status

    def timed_snapshot(self):
        with self.lock:
            return self.frame, self.sequence, self.status, self.captured_at


class App:
    def __init__(self, root, devices, settings):
        self.root = root
        self.closed = False
        self.after_id = None
        self.cameras = []
        self.photos = [None, None]
        root.title("双 USB 摄像头：完整画面预览")
        root.geometry("1200x650")
        root.minsize(640, 360)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda event: self.close())
        header = ttk.Frame(root, padding=8)
        header.pack(fill="x")
        ttk.Label(header, text="完整画面，不裁剪；绿色十字为图像中心（不代表已标定的光学中心）。").pack(side="left")
        ttk.Button(header, text="退出", command=self.close).pack(side="right")
        body = ttk.Frame(root)
        body.pack(fill="both", expand=True)
        self.labels, self.statuses = [], []
        for i, (device, setting) in enumerate(zip(devices, settings)):
            panel = ttk.Frame(body, padding=5)
            panel.grid(row=0, column=i, sticky="nsew")
            body.columnconfigure(i, weight=1, uniform="camera")
            ttk.Label(panel, text=f"摄像头 {i+1} — {device}", anchor="center").pack(fill="x")
            status = tk.StringVar(value="等待画面")
            ttk.Label(panel, textvariable=status, wraplength=550, anchor="center").pack(fill="x")
            label = tk.Label(panel, background="#151515", foreground="white", text="等待画面…")
            label.pack(fill="both", expand=True)
            self.labels.append(label)
            self.statuses.append(status)
            self.cameras.append(Camera(device, setting))
        body.rowconfigure(0, weight=1)
        self.refresh()

    def refresh(self):
        if self.closed:
            return
        for i, camera in enumerate(self.cameras):
            frame, sequence, status = camera.snapshot()
            self.statuses[i].set(status)
            if frame is None:
                self.photos[i] = None
                self.labels[i].configure(image="", text=status)
                continue
            label = self.labels[i]
            width, height = label.winfo_width(), label.winfo_height()
            if width < 10 or height < 10:
                continue
            h, w = frame.shape[:2]
            new_w, new_h = fit_size(w, h, width, height)
            display = cv2.resize(frame, (new_w, new_h))
            cx, cy = round((new_w-1)/2), round((new_h-1)/2)
            for color, thickness in [((0, 0, 0), 4), ((0, 255, 0), 2)]:
                cv2.line(display, (max(0, cx-18), cy), (min(new_w-1, cx+18), cy), color, thickness)
                cv2.line(display, (cx, max(0, cy-18)), (cx, min(new_h-1, cy+18)), color, thickness)
            self.statuses[i].set(status + f"；中心 ({(w-1)/2:g}, {(h-1)/2:g})")
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            ppm = f"P6\n{new_w} {new_h}\n255\n".encode() + rgb.tobytes()
            self.photos[i] = tk.PhotoImage(data=ppm, format="PPM")
            label.configure(image=self.photos[i], text="")
        self.after_id = self.root.after(70, self.refresh)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        for camera in self.cameras:
            camera.stop_event.set()
        self.root.destroy()
        for camera in self.cameras:
            camera.thread.join(timeout=1)


def main():
    root = tk.Tk()
    root.withdraw()
    app = None
    try:
        devices = CAMERA_DEVICES or usb_devices(command("v4l2-ctl", "--list-devices"))
        if len(devices) < 2:
            raise RuntimeError("没有检测到两台 USB 摄像头，请确认都插好。")
        devices = devices[:2]
        if len(set(devices)) != 2:
            raise RuntimeError("两台摄像头不能使用相同的视频节点。")
        settings = []
        for device in devices:
            setting = highest_mode(command("v4l2-ctl", "-d", device, "--list-formats-ext"))
            settings.append(setting)
            print(f"{device}: 请求 {setting[1]}×{setting[2]}, {setting[0]}, {setting[3]} FPS")
        root.deiconify()
        app = App(root, devices, settings)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("启动失败：", error)
        messagebox.showerror("双摄像头", str(error), parent=root)
    finally:
        if app is not None:
            app.close()
        elif root.winfo_exists():
            root.destroy()


if __name__ == "__main__":
    main()
