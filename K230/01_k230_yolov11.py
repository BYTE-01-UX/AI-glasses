'''
实验名称：物体检测（基于yolov11n）
实验平台：01Studio CanMV K230
教程：wiki.01studio.cc
说明：可以通过display="xxx"参数选择"hdmi"、"lcd3_5"(3.5寸mipi屏)或"lcd2_4"(2.4寸mipi屏)显示方式（本工程默认使用2.4寸MIPI屏）
'''
# ============================
# 可配置项总览（按需修改）
# 1) 模型文件路径：kmodel_path（默认 /sdcard/best_cpu.kmodel）
#    - 若更换文件名或目录，修改此变量即可
# 2) 模型输入尺寸：默认 320x320
#    - 若你的 kmodel 为 640x640，需同步修改两处为 [640,640]：
#      a) model_input_size（创建 ObjectDetectionApp 时）
#      b) rgb888p_size（PipeLine -> Ai2d 的输入帧大小）
# 3) 输入数据类型：当前按 uint8→uint8（Ai2d 输出给模型为 uint8）
#    - 若你的 kmodel 需要 float32：
#      a) 将 set_ai2d_dtype 的“输出 dtype”改为 np.float32
#      b) 如需归一化/减均值/标准化，请在 config_preprocess 中增加相应 Ai2d 操作
# 4) 显示输出/屏幕类型：
#    - display 可选 'hdmi' | 'lcd3_5' | 'lcd2_4'（默认 2.4 寸）
#    - display_size 随屏幕类型自动设置；如需自定义，可直接修改
# 5) 阈值：
# 串口与引脚映射（可改）：
# - 这里把 UART2 的 TX/RX 映射到管脚 11/12（默认与 STM32 的 PC6/PC7 连接）
# - 若你的连线不同，请把下面两行改为你实际使用的 FPIOA 管脚编号
#   fpioa.set_function(11, FPIOA.UART2_TXD)
#   fpioa.set_function(12, FPIOA.UART2_RXD)

#    - confidence_threshold（默认 0.5）、nms_threshold（默认 0.2）、max_boxes_num（默认 50）
# 6) 串口输出：
#    - FPIOA 引脚映射：UART2 TX=11, RX=12（可按硬件改）
#    - 波特率：115200
# 7) 标签文件：
#    - /sdcard/classes.txt 优先；若不存在会回退到本地 classes.txt；最少兜底为 ["目标"]
# 8) 性能提示：
#    - 若 640x480 显示 + 640x640 推理帧率较低，可改回 320x320 模型
# ============================

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
from machine import Timer
import ujson
from media.media import *
from media.sensor import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import gc
from machine import UART
from machine import FPIOA
#初始化串口对应引脚
fpioa = FPIOA()
fpioa.set_function(3,FPIOA.UART1_TXD)
fpioa.set_function(4,FPIOA.UART1_RXD)
fpioa.set_function(11,FPIOA.UART2_TXD)
fpioa.set_function(12,FPIOA.UART2_RXD)
#设置串口号1和波特率  uart1=UART(UART.UART1,9600)
uart2=UART(UART.UART2,115200) #设置串口号2和波特率，我平常用的是串口2跟单片机通信，看你自己
DISPLAY_WIDTH = ALIGN_UP(1920, 16)
DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(1080, 16)
OUT_RGB888P_HEIGH = 720

# 自定义YOLOv8检测类
class ObjectDetectionApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,max_boxes_num,confidence_threshold=0.5,nms_threshold=0.2,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.labels=labels
        # 模型输入分辨率
        self.model_input_size=model_input_size
        # 阈值设置
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.max_boxes_num=max_boxes_num
        # sensor给到AI的图像分辨率
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 显示分辨率
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        # 检测框预置颜色值
        self.color_four=[(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
                         (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
                         (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
                         (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
                         (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157)]
        # 宽高缩放比例
        self.x_factor = float(self.rgb888p_size[0])/self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1])/self.model_input_size[1]
        # Ai2d实例，用于实现模型预处理
        self.ai2d=Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

        # Ai2d 输入/输出 dtype 说明：
        # - 本行配置为 输入: uint8(NCHW), 输出: uint8(NCHW)
        # - 若你的 kmodel 输入是 float32，请改为：np.uint8, np.float32，并在下方预处理中做归一化

    # 配置预处理操作，这里使用了resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/libs/AI2D.py查看
    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理
    def postprocess(self,results):
        global Pass
        with ScopedTiming("postprocess",self.debug_mode > 0):
            result=results[0]
            result = result.reshape((result.shape[0] * result.shape[1], result.shape[2]))
            output_data = result.transpose()
            boxes_ori = output_data[:,0:4]
            scores_ori = output_data[:,4:]
            confs_ori = np.max(scores_ori,axis=-1)
            inds_ori = np.argmax(scores_ori,axis=-1)
            boxes,scores,inds = [],[],[]
            for i in range(len(boxes_ori)):
                if confs_ori[i] > self.confidence_threshold:
                    scores.append(confs_ori[i])
                    inds.append(inds_ori[i])
                    x = boxes_ori[i,0]
                    y = boxes_ori[i,1]
                    w = boxes_ori[i,2]
                    h = boxes_ori[i,3]
                    left = int((x - 0.5 * w) * self.x_factor)
                    top = int((y - 0.5 * h) * self.y_factor)
                    right = int((x + 0.5 * w) * self.x_factor)
                    bottom = int((y + 0.5 * h) * self.y_factor)
                    boxes.append([left,top,right,bottom])
                    print(confs_ori[i])
            if len(boxes)==0:
                return []
            boxes = np.array(boxes)
            scores = np.array(scores)
            inds = np.array(inds)
            # NMS过程
            keep = self.nms(boxes,scores,nms_threshold)
            dets = np.concatenate((boxes, scores.reshape((len(boxes),1)), inds.reshape((len(boxes),1))), axis=1)
            dets_out = []
            for keep_i in keep:
                dets_out.append(dets[keep_i])
            dets_out = np.array(dets_out)
            dets_out = dets_out[:self.max_boxes_num, :]
            return dets_out

    # 绘制结果
    def draw_result(self,pl,dets):
        global count
        global num
        global a
        global flag
        global mode
        global init
        global Pass
        global jisu_flag
        with ScopedTiming("display_draw",self.debug_mode >0):
            if dets:
                pl.osd_img.clear()
                for det in dets:
                    x1, y1, x2, y2 = map(lambda x: int(round(x, 0)), det[:4])
                    x= x1*self.display_size[0] // self.rgb888p_size[0]
                    y= y1*self.display_size[1] // self.rgb888p_size[1]
                    w = (x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = (y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    pl.osd_img.draw_rectangle(x,y, w, h, color=self.get_color(int(det[5])),thickness=4)
                    pl.osd_img.draw_string_advanced( x , y-50,32," " + self.labels[int(det[5])] + " " + str(round(det[4],2)) , color=self.get_color(int(det[5])))
                    print(self.labels[int(det[5])])

            else:
                pl.osd_img.clear()


    # 多目标检测 非最大值抑制方法实现
    def nms(self,boxes,scores,thresh):
        """Pure Python NMS baseline."""
        x1,y1,x2,y2 = boxes[:, 0],boxes[:, 1],boxes[:, 2],boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = np.argsort(scores,axis = 0)[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            new_x1,new_y1,new_x2,new_y2,new_areas = [],[],[],[],[]
            for order_i in order:
                new_x1.append(x1[order_i])
                new_x2.append(x2[order_i])
                new_y1.append(y1[order_i])
                new_y2.append(y2[order_i])
                new_areas.append(areas[order_i])
            new_x1 = np.array(new_x1)
            new_x2 = np.array(new_x2)
            new_y1 = np.array(new_y1)
            new_y2 = np.array(new_y2)
            xx1 = np.maximum(x1[i], new_x1)
            yy1 = np.maximum(y1[i], new_y1)
            xx2 = np.minimum(x2[i], new_x2)
            yy2 = np.minimum(y2[i], new_y2)
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            new_areas = np.array(new_areas)
            ovr = inter / (areas[i] + new_areas - inter)
            new_order = []
            for ovr_i,ind in enumerate(ovr):
                if ind < thresh:
                    new_order.append(order[ovr_i])
            order = np.array(new_order,dtype=np.uint8)
        return keep

    # 根据当前类别索引获取框的颜色
    def get_color(self, x):
        idx=x%len(self.color_four)
        return self.color_four[idx]


if __name__=="__main__":

    # 显示模式，可以选择"hdmi"、"lcd3_5"(3.5寸mipi屏)和"lcd2_4"(2.4寸mipi屏)

    # 默认使用2.4寸 MIPI 屏
    display="lcd2_4"

    if display=="hdmi":
        display_mode='hdmi'
        display_size=[1920,1080]

    elif display=="lcd3_5":
        display_mode= 'st7701'
        display_size=[800,480]

    elif display=="lcd2_4":
        display_mode= 'st7701'
        display_size=[640,480]

    rgb888p_size=[320,320] #特殊尺寸定义

    # 模型路径固定为 /sdcard/best_cpu.kmodel（与设备“此电脑/CanMV/sdcard/best_cpu.kmodel”一致）
    kmodel_path = "/sdcard/best_cpu.kmodel"

    # 加载标签：优先 /sdcard/classes.txt；其次当前/上级目录
    def load_labels():
        candidates = [
            "/sdcard/classes.txt",
            "./classes.txt",
            "../classes.txt",
            "../../classes.txt",
        ]
        for p in candidates:
            try:
                with open(p, 'r') as f:
                    lines = [line.strip() for line in f.readlines() if line.strip()]
                    if len(lines) > 0:
                        return lines
            except Exception as e:
                pass
        # 兜底：若未找到则返回空列表
        return []

    labels = load_labels()
    if not labels:
        # 未找到标签文件时的安全兜底（最少给一个占位标签，避免索引越界）
        labels = ["目标"]

    # 其它参数设置
    confidence_threshold = 0.5
    nms_threshold = 0.2
    max_boxes_num = 50

    # 初始化PipeLine（保持与官方 YOLOv8 示例一致，避免与PipeLine内部预处理冲突）
    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    if display =="lcd2_4":
        pl.create(Sensor(width=1280, height=960))  # 4:3 画面
    else:
        pl.create(Sensor(width=1920, height=1080))  # 16:9 画面

    # 与导出脚本保持一致：模型输入固定为 320x320，dtype=uint8
    ob_det = ObjectDetectionApp(
        kmodel_path,
        labels=labels,
        model_input_size=[320, 320],
        max_boxes_num=max_boxes_num,
        confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold,
        rgb888p_size=[320, 320],
        display_size=display_size,
        debug_mode=0
    )
    ob_det.config_preprocess()

    # 进入主循环
    clock = time.clock()
    last_send = utime.ticks_ms()
    while True:

        clock.tick()

        img=pl.get_frame() # 获取当前帧数据
        res=ob_det.run(img) # 推理当前帧
        ob_det.draw_result(pl,res) # 绘制结果到PipeLine的osd图像
        pl.show_image() # 显示当前的绘制结果
        gc.collect()

        # 仅在识别到目标时，通过 UART2 发送简短中文描述："<方向>有<名称>"
        # 方向取值：上/下/左/右/左上/左下/右上/右下（按检测框中心相对画面中心判断）
        now_ms = utime.ticks_ms()
        if utime.ticks_diff(now_ms, last_send) >= 1000:
            count = len(res) if res is not None else 0
            if count > 0:
                # 选取置信度最高的一个目标
                best = None
                best_conf = -1.0
                for det in res:
                    c = float(det[4])
                    if c > best_conf:
                        best = det
                        best_conf = c
                if best is not None:
                    cls_idx = int(best[5])
                    label = labels[cls_idx] if 0 <= cls_idx < len(labels) else "目标"
                    # 将 AI 输入坐标缩放到显示坐标
                    x1d = int(best[0] * display_size[0] // rgb888p_size[0])
                    y1d = int(best[1] * display_size[1] // rgb888p_size[1])
                    x2d = int(best[2] * display_size[0] // rgb888p_size[0])
                    y2d = int(best[3] * display_size[1] // rgb888p_size[1])
                    cx = (x1d + x2d) // 2
                    cy = (y1d + y2d) // 2
                    cx_n = (cx - display_size[0]//2) / float(display_size[0]//2)
                    cy_n = (cy - display_size[1]//2) / float(display_size[1]//2)
                    # 阈值：中心±10%判为中间带，否则给出方向
                    th = 0.10
                    is_left  = (cx_n < -th)
                    is_right = (cx_n >  th)
                    is_up    = (cy_n < -th)
                    is_down  = (cy_n >  th)
                    dir_str = ""
                    if (is_up or is_down) and (is_left or is_right):
                        if is_left and is_up: dir_str = "左上"
                        elif is_right and is_up: dir_str = "右上"
                        elif is_left and is_down: dir_str = "左下"
                        elif is_right and is_down: dir_str = "右下"
                    elif (is_up or is_down):
                        dir_str = "上" if is_up else "下"
                    elif (is_left or is_right):
                        dir_str = "左" if is_left else "右"
                    else:
                        # 落在中心小区域：按偏移较大轴给出主方向
                        if abs(cx_n) >= abs(cy_n):
                            dir_str = "左" if cx_n < 0 else "右"
                        else:
                            dir_str = "上" if cy_n < 0 else "下"
                    uart2.write("%s有%s\r\n" % (dir_str, label))
            # 无目标时不发串口
            last_send = now_ms

        #print(clock.fps()) #打印帧率
