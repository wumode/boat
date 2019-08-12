import sensor, image, time
import time
from pyb import UART
from pyb import Servo
from pyb import Pin

laser_disable=0
laser_enable=1
uart = UART(3, 921600, timeout=50) #串口初始化定义 p4 p5
# red_threshold  =(0, 100, 17, 127, -128, 127)  # 以前
red_threshold  = (20, 68, 41, 127, -13, 54)
yellow_threshold  = red_threshold
#白天 室外 (0, 100, 24, 127, -128, 127) 曝光值 0.6
#室内 灯光 (0, 100, 17, 127, -128, 127) 曝光值 1.5
up_servo=Servo(1)  #p7 p8
down_servo=Servo(2)
up_servo_zero_position = 1500
down_servo_zero_position = 1440
#up_servo_zero_position = 1800       #向下正
#down_servo_zero_position = 1100     #向左正
up_servo.pulse_width(up_servo_zero_position)
down_servo.pulse_width(down_servo_zero_position)

kp_up = -2.6
ki_up = -0.42
kd_up = -0.43

kp_down = 3.19
ki_down = 0.46
kd_down = 0.43

#kp_down = 2.5
#ki_down = 0
#kd_down = 0

bias_down = 0.0
last_bias_down = 0.0
last_last_bias_down = 0.0

bias_up = 0.0
last_bias_up = 0.0
last_last_bias_up = 0.0


up_pwm_limit_up = 500
up_pwm_limit_down = 300
domn_pwm_limit = 700

up_bias_correct = 10.0
down_bias_correct = 0.0
up_bias_range = 10
down_bias_range = 10

#pwm_up_last = 0
#pwm_down_last = 0
#up_error_last = 0
#down_error_last = 0
trace_error_last = 0

mode_choose = -1
mode_choose_last = -1
mode_flag = 0
stop_flag = 0
EXPOSURE_TIME_SCALE = 0.50 #曝光值

# up_servo.pulse_width(1600)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
# sensor.set_auto_whitebal(False) #白平衡关掉 白平衡白天时候不用开启
# sensor.set_auto_gain(False) #颜色识别时候关掉 自动增益
clock = time.clock()
sensor.skip_frames(time=2000)
#更改曝光值
current_exposure_time_in_microseconds = sensor.get_exposure_us()
sensor.set_auto_exposure(False,exposure_us = int(current_exposure_time_in_microseconds * EXPOSURE_TIME_SCALE))

pwm_up = up_servo_zero_position
pwm_down = down_servo_zero_position

# up_servo.pulse_width(1500)

#找到最大的色块 返回色块的外框的宽度w（int）blob[2] 返回色块的外框的宽度h（int）blob[3]


def Incremental_PI_Up (pwm, now, target):
    global bias_up, last_bias_up,last_last_bias_up
    global kp_up, ki_up, kd_up
    bias_up = target - now
    pwm += kp_up*(bias_up - last_bias_up)+ki_up*bias_up + kd_up*(bias_up - 2*last_bias_up + last_last_bias_up)
    last_last_bias_up = last_bias_up
    last_bias_up = bias_up
    return pwm

def Incremental_PI_Down (pwm, now, target):
    global bias_down, last_bias_down,last_last_bias_down
    global kp_down, ki_down, kd_down
    bias_down = target - now
    # print("down bias: %d, last_bias_down: %d" % (bias_down, last_bias_down))
    pwm += kp_down*(bias_down - last_bias_down)+ki_down*bias_down + kd_down*(bias_down - 2*last_bias_down + last_last_bias_down)
    last_last_bias_down = last_bias_down
    last_bias_down = bias_down
    return pwm

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size :
            max_blob = blob
            max_size = blob[2]*blob[3]
    return max_blob


while(True):
    clock.tick()
    while UART.any(UART(3)):
        mode_choose = UART.readchar(UART(3))
    # mode_choose = 2
    # continue
    if mode_choose != 1 and mode_choose != 2 and mode_choose != 3:
        print("while: %d" % mode_choose)
        mode_choose = mode_choose_last
        print("real: %d" % mode_choose)
    # print("NOW: %d" % mode_choose)
    mode_choose_last = mode_choose
    if mode_choose == 2:#打气球程序段
        # clock.tick()
        img = sensor.snapshot()
        blobs = img.find_blobs([red_threshold])
        if blobs:
            max_blob = find_max(blobs)
            down_error_ori = max_blob.cx()-img.width()/2
            up_error_ori = max_blob.cy()-img.height()/2
            img.draw_rectangle(max_blob.rect())
            # img.draw_cross(max_blob.cx(), max_blob.cy())

            #up_error = 0.8*up_error_ori+0*(up_error_ori - up_error_last)
            #down_error = 1.5*down_error_ori+0*(up_error_ori - down_error_last)

            pwm_up = Incremental_PI_Up(pwm_up, up_error_ori, up_bias_correct)
            pwm_down = Incremental_PI_Down(pwm_down, down_error_ori, down_bias_correct)
            pwm_up   = int(pwm_up)
            pwm_down = int(pwm_down)

            if pwm_up > up_pwm_limit_down + up_servo_zero_position:
                pwm_up = up_pwm_limit_down + up_servo_zero_position
            elif pwm_up < up_servo_zero_position - up_pwm_limit_up:
                pwm_up = up_servo_zero_position - up_pwm_limit_up

            if pwm_down > domn_pwm_limit + down_servo_zero_position:
                pwm_down = domn_pwm_limit + down_servo_zero_position
            elif pwm_down < down_servo_zero_position - domn_pwm_limit:
                pwm_down = down_servo_zero_position - domn_pwm_limit
            # print("up: %d, down: %d" % (pwm_up, pwm_down))
            up_servo.pulse_width(pwm_up)
            down_servo.pulse_width(pwm_down)
            # print("up_err: %d, down_err: %d" %(up_error_ori, down_error_ori))
            if(abs(down_error_ori)<15 and up_error_ori<up_bias_range+up_bias_correct and up_error_ori>up_bias_correct-up_bias_range):
                data=bytearray([laser_enable,0x0a])
                uart.write(data)
            else:
                data=bytearray([laser_disable,0x0a])
                uart.write(data)
    elif mode_choose == 3:
        # clock.tick()
        up_servo.pulse_width(up_servo_zero_position)
        down_servo.pulse_width(down_servo_zero_position)
        img = sensor.snapshot()
        blobs_yellow = img.find_blobs([yellow_threshold])
        if blobs_yellow:
            max_blob = find_max(blobs_yellow)
            # print("area: %d" % max_blob.area())
            if max_blob.area() > 400:
                stop_flag=1
            else:
                stop_flag=0
            data_trace = bytearray([max_blob.cx(),stop_flag,0x0b])
            # print("cx: %d"%max_blob.cx())
            # print(data_trace)
            uart.write(data_trace)
            img.draw_rectangle(max_blob.rect())
            # img.draw_cross(max_blob.cx(), max_blob.cy())
        else:
            data_trace = bytearray([125,1,0x0b])
            uart.write(data_trace)
    elif mode_choose == 1:
        up_servo.pulse_width(up_servo_zero_position)
        down_servo.pulse_width(down_servo_zero_position)
    # print(mode_choose)
    # print(clock.fps())
