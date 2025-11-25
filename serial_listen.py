import serial
import subprocess
import imu
import gps
import math
import serial
import threading
import time

ser = serial.Serial(port='/dev/ttyST', baudrate=115200, timeout=None)
if not ser.is_open :
    raise ValueError("faided to open serial")
ser.reset_output_buffer()
ser.write(b'1500,1500\x00')


imu_=imu.IMU()
imu_.init_imu()

gps_=gps.GPS(target_E=121.5231,target_N=38.86986)

pwm_l,pwm_r=1500,1500

KP,KI,KD=1,0.1,0.1

angle_to_n=0

def read_NE_():
    while 1:
        gps_.read_NE()

def execute():
    global pwm_l,pwm_r
    global angle_to_n

    angle_to_n=math.atan2((imu_.CY+49.5),imu_.CX+1e-6)*180/math.pi
    pwm_diff=gps_.decision_angle(angle_to_N=angle_to_n,imu_yaw=imu_.angleZ)

    pwm_max=gps_.decision_distance()

    if pwm_max !=1500:
        if pwm_diff>=0:
            pwm_l=pwm_max
            pwm_r=3000-pwm_max+pwm_diff
        else:
            pwm_l=pwm_max+pwm_diff
            pwm_r=3000-pwm_max
    else:
        pwm_r=1500
        pwm_l=1500
        
    str_temp1=str(pwm_l).encode('ascii')
    str_temp2=str(pwm_r).encode('ascii')
    
    if pwm_l<1000:
        str_temp1=b'0'+str_temp1
    if pwm_r<1000:
        str_temp2=b'0'+str_temp2
	
    str_temp=str_temp1+b','+str_temp2+b'\x00'

    prev_pwm_l=pwm_l
    prev_pwm_r=pwm_r
    if abs(prev_pwm_l-pwm_l)>700:
        print('error left')
        return
    if abs(prev_pwm_r-pwm_r)>700:
        print('error right')
        return
    ret=ser.write(str_temp)

def imu_read():
    while 1:
        imu_.read_data_onebatch()

def get_pid():
    global KP,KI,KD
    global gps_

    ser_pid = serial.Serial(port='/dev/ttyS6', baudrate=115200, timeout=None)
    if not ser_pid.is_open :
        raise ValueError("faided to open serial")
    ser_pid.reset_output_buffer()

    while 1:
        pid_data=ser_pid.read_until(b'end').decode(errors='ignore')
        ser_pid.write(b'distance:'+str(gps_.dist).encode('ascii')+b'\n')
        ser_pid.write(b'CY'+str(imu_.CY+49.5).encode('ascii'))

        pid_data=pid_data.split(sep=',')
        KP=float(pid_data[0])
        KI=float(pid_data[1])
        KD=float(pid_data[2])
        gps_.set_pid_(KP,KI,KD)

th1=threading.Thread(target=imu_read)
th1.daemon=True
th1.start()

th2=threading.Thread(target=read_NE_)
th2.daemon=True
th2.start()

th3=threading.Thread(target=get_pid)
th3.daemon=True
th3.start()

data = 'stop'

while 1:
    data=ser.read_until(b'service').decode(errors='ignore')
    data=data.split(sep='_')
    if data[0] == 'start':
        execute()
    if data[0] == 'stop':
        ret=ser.write(b'1500,1500\x00')