import sys
import serial
import cv2 as cv
import numpy as np
import time
import configparser
from threading import Thread
from maix import camera, display, image,pinmap,touchscreen,app,http
from detect_color import Color_detect
from find_circle import Find_circle
#   -------		串口通讯的规则为：	-------      
#   波特率115200		                        
#   下位机：帧头(1字节)0x1B	 数据(1字节)0x30     
#   上位机：数据(字符)       帧尾(1字节)0x0D     
#   ---------------------------------------	   
#   当接收到下位机数据后会发送'0x06'，表示命令已接收
# 0x30(停止)  0x32(物料颜色识别)  0x33(圆心坐标定位) 0x36(圆环颜色顺序)

html = """<!DOCTYPE html>
<html>
<head>
    <title>JPG Stream</title>
</head>
<body>
    <h1>MaixPy JPG Stream</h1>
    <img src="/stream" alt="Stream">
</body>
</html>"""

# 创建配置解析器
config = configparser.ConfigParser()
config.read('config.ini')
exposure = int(config.get('Settings', 'exposure'))
gain = int(config.get('Settings', 'gain'))

camera_width  = 240
camera_height = 240
cam = camera.Camera(camera_width,camera_height,fps=30)
cam.exposure(exposure)  #曝光时间
cam.gain(gain)       #增益
pinmap.set_pin_function("A18", "UART1_RX")
pinmap.set_pin_function("A19", "UART1_TX")
dis = display.Display()
img_back = image.load("/maixapp/share/icon/ret.png")
back_rect = [0, 0, 32, 32]
back_rect_disp = image.resize_map_pos(cam.width(), cam.height(), dis.width(), dis.height(), image.Fit.FIT_CONTAIN, back_rect[0], back_rect[1], back_rect[2], back_rect[3])
ts = touchscreen.TouchScreen()

device = "/dev/ttyS1"
ser = serial.Serial(device, 115200)
# print('serial test start ...')
response = None
Flag = True

def is_in_button(x, y, btn_pos):
    return x > btn_pos[0] and x < btn_pos[0] + btn_pos[2] and y > btn_pos[1] and y < btn_pos[1] + btn_pos[3]

def adjust(x,y):
    global config,exposure,gain
    if 0<exposure<40000:
        if 110<x<210 and 305<y<360:
            exposure = exposure - 1000
            gain = gain - 100
            cam.exposure(exposure)  #曝光时间
            cam.gain(gain)       #增益
        if 385<x<470 and 305<y<360:
            exposure = exposure + 1000
            gain = gain + 100
            cam.exposure(exposure)  #曝光时间
            cam.gain(gain)       #增益
        config.set('Settings', 'exposure', str(exposure))
        config.set('Settings', 'gain', str(gain))
    if 0<exposure<40000:
        with open('config.ini', 'w') as config_file:
            config.write(config_file)

def rece():
    global response, Flag
    #res = None

    while Flag:
        res = None
        while Flag:
            time.sleep(0.1)
            if ser.inWaiting():
                read_bytes = ser.read(1)
                # print(read_bytes)
                if res is None:
                    if read_bytes == b'\x1B':
                        res = read_bytes
                else:
                    res += read_bytes
                    if (len(res) == 2):
                        ser.write(b'\x06')
                        break

        if res != None:
            if res == b'\x1B\x30':
                response = None
            else:
                response = res


def remove(string):
    return string.replace(" ", "")

def main():
    global Flag
    Cdet = Color_detect()
    Fdet = Find_circle()
    t = Thread(target=rece)
    t.start()

    run_text = ''
    res_text = ''
    frame_tail = b'\x0D'
    QR_start = b'\x1B\x31'
    color_start = b'\x1B\x32'
    cir_start = b'\x1B\x33'
    Sequence_start = b'\x1B\x36'

    oi_x = 40
    oi_y = 40
    oi_w = 160
    oi_h = 160

    stream = http.JpegStreamer()
    stream.set_html(html)
    stream.start()
    #response = QR_start##################################################################################################
    lst = 0
    last_x,last_y = 0,0
    while not app.need_exit():
        try:
            img = cam.read()
            if response == None:
                jpg = img.to_jpeg()
                stream.write(jpg)
            if not img:
                time.sleep(0.001)
                continue
            st = time.time()
            run_t = int(1//round(st - lst, 4))
            lst = st
            frame = image.image2cv(img,ensure_bgr=True, copy=False)
            if response == QR_start:
                qrcodes = img.find_qrcodes()
                run_text = "QR"
                for qr in qrcodes:
                    corners = qr.corners()
                    for i in range(4):
                        img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1], image.COLOR_RED)
                    img.draw_string(qr.x(), qr.y() - 15, qr.payload(), image.COLOR_RED)
                    res_text = qr.payload()
                    ser.write(res_text.encode()+frame_tail)
                else:
                    res_text = ""
            if response == color_start:
                # roi = img[y:y+h, x:x+w]
                C_text = Cdet.run(frame[oi_y:oi_y+oi_h,oi_x:oi_x+oi_w])
                run_text = "COLOR"
                if C_text:
                    res_text = C_text
                    # response = None
                    ser.write(res_text.encode()+frame_tail)
                else:
                    res_text = ""
                cv.rectangle(frame, (oi_x, oi_y),
                            (oi_x + oi_w, oi_y + oi_h), (0, 255, 0), 2)
            elif response == cir_start:

                # 需要检测的圆环颜色列表
                colors_to_detect = ["green"]# 最后一个值为通用，按需求使用"red", "blue", "green", "general"
                # 遍历颜色列表
                for color in colors_to_detect:
                    F_text = Fdet.find_circle_Pos(frame, color, debug=True)
                    if F_text:
                        # 如果找到，退出循环
                        break
                run_text = "CIRCLE"
                if F_text:
                    res_text = remove(str(F_text))
                    img.draw_circle(F_text[0], F_text[1], 2,image.Color.from_rgb(255, 0, 0),2)
                    ser.write(res_text.encode()+frame_tail)
                else:
                    res_text = ""
            elif response == Sequence_start: 
                F_text = Fdet.find_circle_Seq(frame, True)
                run_text = "Sequence"
                if F_text:
                    res_text = remove(str(F_text))
                    ser.write(res_text.encode()+frame_tail)
                else:
                    res_text = ""
            elif response == b'\x1B\x30':
                run_text = ""
                res_text = ""
            elif response is None:
                run_text = ""
                res_text = ""
            img.draw_string(125, 5,"RUN: {0}".format(run_text), color = image.COLOR_RED,scale=1)
            img.draw_string(125, 18,"RES:{0}".format(res_text), color = image.COLOR_RED,scale=1)
            img.draw_string(125, 31,"FPS: {0}".format(run_t), color = image.COLOR_RED,scale=1)
            img.draw_image(0, 0, img_back)
            img.draw_rect(0,210, 50, 30, color = image.COLOR_RED,thickness=3)
            img.draw_rect(190, 210,50,30, color = image.COLOR_BLUE,thickness=3)
            img.draw_string(9,212, '-', color = image.COLOR_GREEN,scale=2.5,thickness=2)
            img.draw_string(199, 212, '+', color = image.COLOR_GREEN,scale=2.5,thickness=2)
            dis.show(img)
            x, y, preesed = ts.read()
            if is_in_button(x, y, back_rect_disp):
                # 停止线程
                Flag = False
                t.join()  # 等待线程结束
                app.set_exit_flag(True)
            if last_x != x and last_y != y:
                adjust(x,y)
                time.sleep(0.01)
            last_x,last_y = x,y
        except:
            time.sleep(0.001)

if __name__ == '__main__':
    main()