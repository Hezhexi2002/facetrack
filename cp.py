 # -*- coding:utf-8 -*-
'''
人脸识别并控制舵机进行人脸追踪

NOTE
1. 为保证人脸识别的帧率分辨率控制在320x240
BUG
1. 刚开始的时候缓存区视频流的问题， 导致舵机云台会乱转 向大佬低头  数值越界
ESP32出解析的数据：Bottom: 6553600 Top: 6556160
2. Reset 舵机云台，串口数据发送之后，ESP32有时候不执行
TODO
1. 获得舵机旋转角度的反馈数据
2. 创建两个Trackbar， 设置两个Kp取值
3. 绘制上下臂舵机的波形图

参考Kp取值
1. Kp = 10  比例系数较大，来回抖动非常明显
2. Kp = 20  幅度过大，旋转后目标直接丢失
3. Kp = 5   幅度适中，有小幅抖动
4. Kp = 2   相应速度比较慢
'''
import cv2
import RPi.GPIO as GPIO
import time
from PCA9685 import PCA9685
import multiprocessing


# 关闭警告
GPIO.setwarnings(False)
# BOARD编号方式，基于插座引脚编号
GPIO.setmode(GPIO.BOARD)
# IO口设置为输出模式
GPIO.setup(13, GPIO.OUT) # GPIO13为LED
GPIO.setup(15, GPIO.OUT) # GPIO15为蜂鸣器
# 输出低电平
GPIO.output(13, GPIO.LOW)
GPIO.output(15, GPIO.LOW)

# 全局变量，报警检测的时间，单位：秒
TIMER_THRESHOLD = 3.0

pwm = PCA9685()
pwm.setPWMFreq(50)
last_btm_degree = 75 # 最近一次底部舵机的角度值记录
last_top_degree = 75 # 最近一次顶部舵机的角度值记录

btm_kp = 5 # 底部舵机的Kp系数
top_kp = 5 # 顶部舵机的Kp系数

offset_dead_block = 0.1 # 设置偏移量的死区

# 舵机角度初始化
#set_cloud_platform_degree(last_btm_degree, last_top_degree)
pwm.setRotationAngle(1, last_btm_degree )
pwm.setRotationAngle(0, last_top_degree ) 
# 载入人脸检测的Cascade模型
FaceCascade = cv2.CascadeClassifier('/home/pi/facetrack/haar/haarcascade_frontalface_default.xml')
eyeL_cascade = cv2.CascadeClassifier('/home/pi/facetrack/haar/haarcascade_lefteye_2splits.xml')
eyeR_cascade = cv2.CascadeClassifier('/home/pi/facetrack/haar/haarcascade_righteye_2splits.xml')

# 全局变量，用于保存识别出的人脸、左右眼的坐标
face_region = [0, 0, 0, 0]
eyeL_region = [0, 0, 0, 0]
eyeR_region = [0, 0, 0, 0]

# 全局变量，用于保存检测结果
DetectResult = 0
# 检测结果定义
FIND_NONE = 0x00 # 无结果
FIND_FACE = 0x01 # 脸
FIND_EYEL = 0x02 # 左眼
FIND_EYER = 0x04 # 右眼

# 创建一个窗口 名字叫做Face
cv2.namedWindow('FaceDetect',flags=cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

def update_btm_kp(value):
# 更新底部舵机的比例系数
    global btm_kp
    btm_kp = value

def update_top_kp(value):
# 更新顶部的比例系数
    global top_kp
    top_kp = value

# 创建底部舵机Kp的滑动条
cv2.createTrackbar('BtmServoKp','FaceDetect',0, 20,update_btm_kp)
# 设置btm_kp的默认值
cv2.setTrackbarPos('BtmServoKp', 'FaceDetect', btm_kp)
# 创建顶部舵机Kp的滑动条
cv2.createTrackbar('TopServoKp','FaceDetect',0, 20,update_top_kp)
# 设置top_kp的默认值
cv2.setTrackbarPos('TopServoKp', 'FaceDetect', top_kp)



cap = cv2.VideoCapture(0)
cap.set(3,320) # set Width
cap.set(4,240) # set Height

# 自己写的定时器类
class timer():
    # 构造函数
    def __init__(self):
        self.status = 0
        self.StartTime = 0
        self.StopTime = 0

    # 开始计时
    def start(self):
        if (self.status == 0):
            self.StartTime = time.time()
            self.status = 1

    # 结束计时
    def stop(self):
        if(self.status == 1):
            self.StopTime = time.time()
            self.status = 0
        return self.StopTime - self.StartTime
    
    # 定时器启动时获取已经计时的时间，定时器停止时获取从开始到停止的时间
    def getTimer(self):
        if (self.status == 0):
            return self.StopTime - self.StartTime
        else:
            return time.time() - self.StartTime
    
    # 重置定时器
    def reset(self):
        self.status = 0
        self.StartTime = 0
        self.StopTime = 0
    



# 报警
# event: 信号量，保证GPIO不会被并发调用
def alarm(event):
    if (event.is_set()):
       return
    else:
        event.set()
        for i in range(0, 4):
            # 让蜂鸣器和LED按一定规律报警
            GPIO.output(13, GPIO.HIGH)
            GPIO.output(15, GPIO.LOW)
            time.sleep(0.2)
            GPIO.output(13, GPIO.LOW)
            GPIO.output(15, GPIO.HIGH)
            time.sleep(0.2)

        # 关闭蜂鸣器和LED
        GPIO.output(13, GPIO.LOW)
        GPIO.output(15, GPIO.LOW)
       
        event.clear() # 释放信号量






def btm_servo_control(offset_x):
#底部舵机的比例控制
#这里舵机使用开环控制
    global offset_dead_block # 偏移量死区大小
    global btm_kp # 控制舵机旋转的比例系数
    global last_btm_degree # 上一次底部舵机的角度

# 设置最小阈值
    if abs(offset_x) < offset_dead_block:
      offset_x = 0

# offset范围在-50到50左右
    delta_degree = offset_x * btm_kp
# 计算得到新的底部舵机角度
    next_btm_degree = last_btm_degree + delta_degree
# 添加边界检测
    if next_btm_degree < 0:
      next_btm_degree = 0
    elif next_btm_degree > 180:
        next_btm_degree = 180

    return int(next_btm_degree)


def top_servo_control(offset_y):
#顶部舵机的比例控制
#这里舵机使用开环控制
    global offset_dead_block
    global top_kp # 控制舵机旋转的比例系数
    global last_top_degree # 上一次顶部舵机的角度

# 如果偏移量小于阈值就不相应
    if abs(offset_y) < offset_dead_block:
      offset_y = 0

# offset_y *= -1
# offset范围在-50到50左右
    delta_degree = offset_y * top_kp
# 新的顶部舵机角度
    next_top_degree = last_top_degree + delta_degree
# 添加边界检测
    if next_top_degree < 0:
      next_top_degree = 0
    elif next_top_degree > 180:
        next_top_degree = 180

    return int(next_top_degree)

# 人脸识别
# img: 待识别的图片
def GetFace(img):
    # 引用全局变量    
    global face_region, DetectResult

    # 参数image:要检测的图片，一般为灰度图
    # 参数scaleFactor:缩放因子，对图片进行缩放，默认为1.1
    # 参数minNeighbors:最小邻居数，默认为3
    # 参数flags:兼容老版本的一个参数，在3.0以后的版本中没用，默认为0
    # 参数minSize:检测的最小尺寸
    # 参数maxSize:检测的最大尺寸
    faces1 = FaceCascade.detectMultiScale(
        image = img,
        scaleFactor = 1.15,
        minNeighbors = 3,
        flags = 0,
        minSize = (20,20),
        maxSize = (200,200)
    )

    if(len(faces1) != 1):
        # 没有识别到脸或识别到多张脸则清除标志位，并将脸部坐标清零
        DetectResult &= ~FIND_FACE
        face_region = [0, 0, 0, 0]
    else:
        # 否则设置相关标志位，保存脸部坐标
        DetectResult |= FIND_FACE
        face_region = faces1[0]


# 眼睛识别
# img: 待识别的图片
def GetEyes(img):
    # 引用全局变量    
    global eyeL_region, eyeR_region, DetectResult

    eyeL = eyeL_cascade.detectMultiScale(
        image = img,
        scaleFactor = 1.15,
        minNeighbors = 3,
        flags = 0,
        minSize = (20,20),
        maxSize = (200,200)
    )

    eyeR = eyeR_cascade.detectMultiScale(
        image = img,
        scaleFactor = 1.15,
        minNeighbors = 3,
        flags = 0,
        minSize = (20,20),
        maxSize = (200,200)
    )

    # 处理左眼识别结果
    if(len(eyeL) != 1):
        # 未识别出左眼或识别出多个左眼则清除标志位，并将左眼坐标清零
        DetectResult &= ~FIND_EYEL
        eyeL_region = [0, 0, 0, 0]
    else:
        # 设置标志位，保存眼部坐标，计算眼睛相对于脸部的坐标
        DetectResult |= FIND_EYEL
        eyeL_region = eyeL[0]
        eyeL_region[0] = eyeL_region[0] + face_region[0]
        eyeL_region[1] = eyeL_region[1] + face_region[1] + int(face_region[3] * 0.2)

    # 处理右眼识别结果
    if(len(eyeR) != 1):
        # 未识别出右眼或识别出多个右眼则清除标志位，并将右眼坐标清零
        DetectResult &= ~FIND_EYER
        eyeR_region = [0, 0, 0, 0]
    else:
        # 设置标志位，保存眼部坐标，计算眼睛相对于脸部的坐标
        DetectResult |= FIND_EYER
        eyeR_region = eyeR[0]
        eyeR_region[0] = eyeR_region[0] + face_region[0]
        eyeR_region[1] = eyeR_region[1] + face_region[1] + int(face_region[3] * 0.2)

# 提取脸部图像
# img: 待提取的图片
# face_region_img: 提取出的脸部图片
def SelectFace(img):
    global face_region
    face_region_img = img[face_region[1]:face_region[1]+face_region[3], face_region[0]:face_region[0]+face_region[2]]
    return face_region_img

# 从识别出的脸上选取眼睛的区域图像
# img1: 待提取的图片
# eyes_region_img: 提取出的眼睛图片
def SelectEyes(img1):
    # 设人脸图像的长宽均为1个单位长度，x向右为正方向，y向下为正方向
    # 眼睛的区域一般位于人脸的y=0.2至y=0.6的范围内

    # 获取图像的长宽
    y, x = img1.shape

    # 计算要选取的范围
    y1 = int(y * 0.2)
    y2 = int(y * 0.6)

    eyes_region_img = img1[y1:y2, 0:x] # 提取区域img[y1:y2,x1:x2]

    return eyes_region_img    

# 绘制人脸，眼睛的边框
def DrawBoundary(img, face, eye1, eye2):
    cv2.rectangle(img, (face[0], face[1]), (face[0] + face[2], face[1] + face[3]), (0, 255, 0), 3)
    cv2.rectangle(img, (eye1[0], eye1[1]), (eye1[0] + eye1[2], eye1[1] + eye1[3]), (255, 0, 0), 3)
    cv2.rectangle(img, (eye2[0], eye2[1]), (eye2[0] + eye2[2], eye2[1] + eye2[3]), (255, 0, 0), 3)
    return img


# 目前找的是画面中面积最大的人脸
    max_face =  max(faces, key=lambda face: face[2]*face[3])
    (x, y, w, h) = max_face
    if w < 10 or h < 10:
        return None
    return max_face

def calculate_offset(img_width, img_height, face):
#计算人脸在画面中的偏移量
#偏移量的取值范围： [-1, 1]
    (x, y, w, h) = face
    face_x = float(x + w/2.0)
    face_y = float(y + h/2.0)
# 人脸在画面中心X轴上的偏移量
    offset_x = float(face_x / img_width - 0.5) * 2
# 人脸在画面中心Y轴上的偏移量
    offset_y = float(face_y / img_height - 0.5) * 2

    return (offset_x, offset_y)

# 初始化计时器
face_timer = timer()
eyes_timer = timer()

# 信号量
AlarmEvent = multiprocessing.Event()
    
   


while (True):
    

    ret, img = cap.read()
    # 手机画面水平翻转
    img = cv2.flip(img, 1)
    # 将彩色图片转换为灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    
    # 识别脸部图像
    GetFace(gray)
  
    if(DetectResult & FIND_FACE == FIND_FACE):
        face_img = SelectFace(gray)
        eye_img = SelectEyes(face_img)
        GetEyes(eye_img)
 
        img_height, img_width,_ = img.shape
        print("img h:{} w:{}".format(img_height, img_width))
        # 计算x轴与y轴的偏移量
        (offset_x, offset_y) = calculate_offset(img_width, img_height, face_region)
        # 计算下一步舵机要转的角度
        next_btm_degree = btm_servo_control(offset_x)
        next_top_degree = top_servo_control(offset_y)
        # 舵机转动
        pwm.setRotationAngle(1, next_btm_degree)
        pwm.setRotationAngle(0, next_top_degree)
        # 更新角度值
        last_btm_degree = next_btm_degree
        last_top_degree = next_top_degree
        print("X轴偏移量：{} Y轴偏移量：{}".format(offset_x, offset_y))
        print('底部角度： {} 顶部角度：{}'.format(next_btm_degree, next_top_degree))
    else:
        # 否则清除眼睛的检测结果
        # 如果这里不清除，会导致上一次眼睛检测的结果叠加在没有识别出人脸的图像上
        DetectResult &= ~FIND_EYEL
        DetectResult &= ~FIND_EYER
        eyeL_region = [0, 0, 0, 0]
        eyeR_region = [0, 0, 0, 0]


  
    key = cv2.waitKey(1)
    if key == ord('q'):
        # 退出程序
        break
    elif key == ord('r'):
        print('舵机重置')
        # 重置舵机
        # 最近一次底部舵机的角度值记录
        last_btm_degree = 80
        # 最近一次顶部舵机的角度值记录
        last_top_degree = 75
        # 舵机角度初始化
        pwm.setRotationAngle(1, last_btm_degree)
        pwm.setRotationAngle(0, last_top_degree)





    # 计时与报警的相关代码
    if(DetectResult & FIND_FACE == FIND_NONE):
        # 未检测到人脸则启动计时器
        face_timer.start()
    else:
        # 否则重置计时器
        face_timer.reset()

    if(DetectResult & (FIND_EYEL | FIND_EYER) == FIND_NONE):
        # 未检测到左右眼则启动计时器            
        eyes_timer.start()
    else:
        # 否则重置计时器
        eyes_timer.reset()


    if(face_timer.getTimer() > TIMER_THRESHOLD):
        # 如果5秒之内没检测到人脸则报警
        # 保证人脸检测能继续运行，这里启动一个新的进程执行报警程序
        p = multiprocessing.Process(target=alarm, args=(AlarmEvent,))
        p.start()
        p.join(0)
    elif(eyes_timer.getTimer() > TIMER_THRESHOLD):
        # 如果5秒之内没检测到眼睛则报警
        # 保证人脸检测能继续运行，这里启动一个新的进程执行报警程序
        p = multiprocessing.Process(target=alarm, args=(AlarmEvent,))
        p.start()
        p.join(0)
    else:
        pass

    # 画出脸、眼睛的范围并显示  
    marked_face = DrawBoundary(img, face_region, eyeL_region, eyeR_region)
    cv2.imshow("FaceDetect", marked_face) # 显示标记出的脸部、眼睛图像


  
        
        
# 释放VideoCapture
cap.release()
# 关闭所有的窗口
cv2.destroyAllWindows()
#关闭舵机
pwm.exit_PCA9685()
# 关闭蜂鸣器、LED，GPIO清零
GPIO.output(13, GPIO.LOW)
GPIO.output(15, GPIO.LOW)
GPIO.cleanup()
   
