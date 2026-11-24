"""RAW field calibration. Camera only: never opens UART or moves any motor."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import numpy as np

COLORS = {'1': '红', '2': '黄', '3': '蓝', '4': '绿', '5': '黑', '6': '浅蓝'}


def measured_ranges(samples, margin):
    if not 0 <= margin <= 15:
        raise ValueError('颜色余量须为0～15。')
    ranges = {}
    for key, name in COLORS.items():
        values = np.asarray(samples.get(key, []), dtype=float)
        if values.ndim != 2 or values.shape[1] != 3 or len(values) < 20 or not np.isfinite(values).all():
            raise ValueError(f'请先采集{key}号{name}色的20帧样本。')
        low = np.clip(np.floor(np.percentile(values, 2, axis=0)) - margin, 0, 254)
        high = np.clip(np.ceil(np.percentile(values, 98, axis=0)) + margin, 1, 255)
        high = np.maximum(high, low + 1)
        ranges[key] = [low.astype(int).tolist(), high.astype(int).tolist()]
    # The receiver accepts only an unambiguous color. Never hide overlapping fits.
    for key in COLORS:
        values = np.asarray(samples[key])
        passed = 0
        for point in values:
            matches = [k for k, (low, high) in ranges.items()
                       if np.all(point >= low) and np.all(point <= high)]
            passed += matches == [key]
        if passed / len(values) < .90:
            raise ValueError(f'{key}号{COLORS[key]}色样本不能稳定单独归类；重新选顶面内部区域采集，或减小颜色余量。')
    return ranges


def make_config(pickup, samples, margin, window, dwell, action, unlimited=False, static_manual=False):
    if static_manual:
        if not unlimited:
            raise ValueError('人工静态摆料模式须同时启用无限停留。')
        return dict(static_manual=True, unlimited_stop=True)
    if pickup is None:
        raise ValueError('请先记录场内侧抓取停位的顶面中心。')
    try:
        window = float(window)
        if not unlimited: dwell, action = float(dwell), float(action)
    except (TypeError, ValueError):
        raise ValueError('窗口、最短停留和取料清离耗时须填写实测数值。')
    if not np.isfinite(window) or not 0 < window < 29 or (not unlimited and (
            not all(np.isfinite(v) for v in (dwell, action)) or not (
            0 < action < dwell <= 120000 and dwell > action + 1350))):
        raise ValueError('窗口须在0～29 mm；耗时为正，最短停留≤120000 ms，且比清离耗时多超过1350 ms。')
    if static_manual and not unlimited:
        raise ValueError('人工静态摆料模式须同时启用无限停留。')
    return dict(pickup=list(pickup), window_mm=window, unlimited_stop=bool(unlimited), static_manual=bool(static_manual),
                minimum_stop_ms=None if unlimited else dwell,
                pickup_clear_ms=None if unlimited else action, lab_ranges=measured_ranges(samples, margin))


def write_json(path, data):
    path = Path(path)
    if path.exists():
        backup = path.with_name(path.name + '.' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.bak')
        shutil.copy2(path, backup)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


class Setup:
    def __init__(self, root, args):
        from ring_metric import RingMetric
        self.root, self.args = root, args
        self.metric = RingMetric.load(args.calibration)
        self.capture = self.frame = self.photo = self.roi = self.pickup = self.anchor = None
        self.samples = {}
        self.collecting = None
        self.pending_samples = []
        self.choose_pickup = False
        root.title('RAW现场配置 · 不连接串口、不驱动小车')
        self.status = tk.StringVar(value='小车停稳后，先退出原双摄程序，再打开相机。')
        left, right = ttk.Frame(root, padding=8), ttk.Frame(root, padding=8)
        left.grid(row=0, column=0, sticky='n'); right.grid(row=0, column=1, sticky='n')
        ttk.Label(left, text='相机2原图；拖框选择物料顶面内部，不要包括边缘、孔洞或背景。').pack()
        self.canvas = tk.Canvas(left, width=640, height=480, bg='black')
        self.canvas.pack()
        self.canvas.bind('<ButtonPress-1>', self.press)
        self.canvas.bind('<B1-Motion>', self.drag)
        self.canvas.bind('<ButtonRelease-1>', self.release)
        ttk.Label(left, textvariable=self.status, wraplength=630).pack(anchor='w', pady=8)
        ttk.Label(right, text='1. 固定观察姿态与抓取窗口').pack(anchor='w')
        ttk.Button(right, text='打开相机2', command=self.open_camera).pack(fill='x')
        ttk.Button(right, text='记录抓取点：点击实际停位的顶面中心', command=self.arm_pickup).pack(fill='x', pady=4)
        self.point_text = tk.StringVar(value='抓取点：未记录（不是默认画面中心）')
        ttk.Label(right, textvariable=self.point_text, wraplength=330).pack(anchor='w')
        ttk.Separator(right).pack(fill='x', pady=9)
        ttk.Label(right, text='2. 分别摆入六色物料，框选后采集20帧').pack(anchor='w')
        self.labels = {}
        for key, name in COLORS.items():
            row = ttk.Frame(right); row.pack(fill='x')
            ttk.Button(row, text=f'{key}号 · {name}', command=lambda k=key: self.collect(k)).pack(side='left')
            label = ttk.Label(row, text='未采集'); label.pack(side='left', padx=8)
            self.labels[key] = label
        ttk.Separator(right).pack(fill='x', pady=9)
        self.fields = {}
        self.time_entries = []
        for key, title, default in [('margin', 'LAB颜色余量', '4'), ('window', '窗口半径 / mm', '8'),
                                    ('dwell', '最短可靠停留 / ms', ''), ('action', '取料至清离最坏耗时 / ms', '')]:
            row = ttk.Frame(right); row.pack(fill='x', pady=2)
            ttk.Label(row, text=title, width=26).pack(side='left')
            variable = tk.StringVar(value=default)
            entry=ttk.Entry(row, textvariable=variable, width=9); entry.pack(side='left')
            if key in ('dwell','action'): self.time_entries.append(entry)
            self.fields[key] = variable
        self.unlimited = tk.BooleanVar(value=True)
        ttk.Checkbutton(right, text='本次物料无限停留（不检查时间预算）', variable=self.unlimited,
                        command=self.update_time_fields).pack(anchor='w', pady=4)
        self.update_time_fields()
        self.static_manual = tk.BooleanVar(value=True)
        ttk.Checkbutton(right, text='本轮人工静态摆料（与单件调试相同）', variable=self.static_manual).pack(anchor='w')
        self.pose = tk.BooleanVar(value=False)
        ttk.Checkbutton(right, text='当前相机观察姿态与标定文件一致', variable=self.pose).pack(anchor='w', pady=6)
        ttk.Button(right, text='保存采样草稿（未测完可下次继续）', command=self.save_draft).pack(fill='x')
        ttk.Button(right, text='校验并生成 raw_config.json', command=self.export).pack(fill='x', pady=5)
        ttk.Label(right, text='静态模式使用顶面HSV颜色和三帧坐标；无需填写LAB或窗口。\n转盘模式：取消静态模式，再按实测标定。', wraplength=340).pack(anchor='w')
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.load_draft()

    def raw_point(self, event):
        w, h = self.metric.image_size
        return [int(np.clip(round(event.x * w / 640), 0, w-1)),
                int(np.clip(round(event.y * h / 480), 0, h-1))]

    def arm_pickup(self):
        self.choose_pickup = True
        self.status.set('将物料放到实际可抓取停位，点击其顶面中心。')

    def press(self, event):
        if self.frame is None or self.collecting:
            return
        if self.choose_pickup:
            self.pickup = self.raw_point(event); self.choose_pickup = False
            self.point_text.set(f'抓取点（原图像素）：{self.pickup}')
            return
        self.anchor = self.raw_point(event)

    def drag(self, event):
        if self.anchor and not self.collecting:
            point = self.raw_point(event)
            self.roi = [min(self.anchor[0], point[0]), min(self.anchor[1], point[1]),
                        max(self.anchor[0], point[0]), max(self.anchor[1], point[1])]

    def release(self, event):
        self.drag(event); self.anchor = None

    def open_camera(self):
        if self.capture:
            return
        # Share the existing service's camera order; do not guess camera1.
        from dual_camera_recognition import CAMERA_DEVICES
        device = self.args.camera or CAMERA_DEVICES[1]
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            messagebox.showerror('相机未打开', '请先退出占用相机的双摄/预览程序，并核对相机2设备路径。')
            return
        w, h = self.metric.image_size
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, w); capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.capture = capture
        self.status.set(f'相机2：{device}；标定要求原图 {w}×{h}。')
        self.refresh()

    def refresh(self):
        if self.capture is None:
            return
        ok, frame = self.capture.read()
        if not ok or tuple(frame.shape[:2][::-1]) != self.metric.image_size:
            self.capture.release(); self.capture = None
            messagebox.showerror('采集失败', '相机无画面或原图分辨率与标定不一致；未生成配置。')
            return
        self.frame = frame
        if self.collecting:
            x0, y0, x1, y1 = self.roi
            lab = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2LAB)
            self.pending_samples.append(np.median(lab, axis=(0, 1)).tolist())
            self.labels[self.collecting].configure(text=f'{len(self.pending_samples)} / 20')
            if len(self.pending_samples) >= 20:
                self.samples[self.collecting] = self.pending_samples
                self.labels[self.collecting].configure(text='已采集20帧')
                self.status.set('本色采集完成；可换下一色或重新框选并重采。')
                self.collecting = None
        display = frame.copy()
        if self.roi:
            cv2.rectangle(display, tuple(self.roi[:2]), tuple(self.roi[2:]), (0, 255, 255), 2)
        if self.pickup:
            cv2.drawMarker(display, tuple(self.pickup), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        rgb = cv2.cvtColor(cv2.resize(display, (640, 480)), cv2.COLOR_BGR2RGB)
        self.photo = tk.PhotoImage(data=b'P6\n640 480\n255\n' + rgb.tobytes(), format='PPM')
        self.canvas.delete('frame'); self.canvas.create_image(0, 0, anchor='nw', image=self.photo, tags='frame')
        self.root.after(100, self.refresh)

    def collect(self, key):
        if self.frame is None or not self.roi or self.roi[2]-self.roi[0]<6 or self.roi[3]-self.roi[1]<6:
            messagebox.showinfo('先框选', '请打开相机，并拖框选取物料顶面内部至少6×6原图像素的区域。')
            return
        if self.collecting:
            return
        self.collecting = key; self.pending_samples = []
        self.status.set(f'采集{key}号{COLORS[key]}色：请让物料静止，手退出画面。')

    def save_draft(self):
        write_json(self.args.draft, dict(pickup=self.pickup, samples=self.samples, unlimited_stop=self.unlimited.get(),
                                       static_manual=self.static_manual.get(),
                                       fields={k:v.get() for k,v in self.fields.items()}))
        self.status.set(f'草稿已保存：{self.args.draft.name}；尚不可用于RAW。')

    def load_draft(self):
        if not self.args.draft.exists():
            return
        try:
            draft = json.loads(self.args.draft.read_text(encoding='utf-8-sig'))
            self.pickup = draft.get('pickup'); self.samples = draft.get('samples', {})
            self.unlimited.set(bool(draft.get('unlimited_stop',True))); self.update_time_fields()
            self.static_manual.set(bool(draft.get('static_manual',True)))
            for key, value in draft.get('fields', {}).items():
                if key in self.fields: self.fields[key].set(value)
            for key in self.samples:
                if key in self.labels: self.labels[key].configure(text='已加载草稿样本')
            self.point_text.set(f'草稿抓取点：{self.pickup}（请复核姿态）')
        except (OSError, ValueError, TypeError) as exc:
            messagebox.showwarning('草稿未加载', str(exc))

    def export(self):
        try:
            if self.collecting: raise ValueError('请等当前颜色采集完成。')
            if not self.pose.get(): raise ValueError('请先复核当前相机观察姿态与标定文件一致。')
            config = make_config(self.pickup, self.samples, float(self.fields['margin'].get()),
                                 self.fields['window'].get(), self.fields['dwell'].get(), self.fields['action'].get(), self.unlimited.get(), self.static_manual.get())
            # Use the real currently deployed RAW loader to check pixel geometry too.
            from route2_top_center import EdgeRawTarget
            EdgeRawTarget(self.metric, config)
            self.save_draft()
            write_json(self.args.output, config)
            self.status.set(f'校验通过，已生成：{self.args.output}')
            messagebox.showinfo('配置已生成', '已生成有效格式的RAW配置。关闭本工具，再用 --raw-config raw_config.json 启动统一程序。仍需实物联调验证。')
        except (ValueError, TypeError, OSError, KeyError) as exc:
            messagebox.showerror('未生成配置', str(exc))

    def close(self):
        if self.capture is not None: self.capture.release()
        self.root.destroy()

    def update_time_fields(self):
        for entry in self.time_entries:
            entry.configure(state='disabled' if self.unlimited.get() else 'normal')


def main():
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=base/'ring_metric_Z184_image_center.json')
    parser.add_argument('--camera', help='相机2设备；默认沿用统一程序的第二路')
    parser.add_argument('--output', type=Path, default=base/'raw_config.json')
    parser.add_argument('--draft', type=Path, default=base/'raw_config.draft.json')
    args = parser.parse_args()
    root = tk.Tk()
    try:
        Setup(root, args)
        root.mainloop()
    except (OSError, ValueError, ImportError) as exc:
        messagebox.showerror('工具未启动', str(exc)); root.destroy()


if __name__ == '__main__':
    main()
