import pid
import math
import serial_base

'''
    经纬度以小数形式表示
    （理想）地球赤道周长=40030173 m
    38.8度周长=40030173 m * cos（38.8度）=31197033.56 m
    所以38.8纬度处，1经度=86658.42656 m
    1纬度=111194.92664 m
'''

class GPS(serial_base.serial_):
    def __init__(self, ser_port="/dev/ttyGPS",target_N=38.88888,target_E=121.53535):
        super().__init__(ser_port=ser_port)
        self.target_N= target_N
        self.target_E= target_E
        self.box=[]
        self.angle=0.0
    
        self.N=38.88888
        self.E=121.53535
        self.T=0

        self.Kp=3
        self.Ki=0
        self.Kd=0

        self.dist=200
        self.aim_angle=0

        self.angle_pid=pid.P_PID(200,800,Kp=self.Kp, Ki=self.Ki, Kd=self.Kd)#输入目标角度和当前角度，输出转速差

    def read_NE(self):
        gngll_str=self.ser.read_until()
        if gngll_str=='':
            return
        
        temp=gngll_str.decode()

        data_part = temp.split('*')[0].strip('$').strip('\r\n')# 移除字符串中的特殊字符和校验和部分
        fields = data_part.split(',')# 按逗号分割各字段

        if fields[0] != 'GNGLL':
            print("不是有效的GNGLL格式字符串")
            return
        if '' in (fields[1],fields[3],fields[5]):
            return
        self.N = float(fields[1][0:2])+float(fields[1][2:-1])/60   # 纬度数值（4位整数5位小数）
        self.E = float(fields[3][0:3])+float(fields[3][3:-1])/60    # 经度数值（5位整数5位小数）
        self.T = float(fields[5])     # 时间（6位 三位小数）

    def decision_distance(self):        
        distance=math.sqrt(pow(abs(self.N -self.target_N)*111194.92664,2)+pow(abs(self.E-self.target_E)*86658.42656,2))
        self.dist=distance

        if distance >=8:
            return 2100
            #推进器2100工作，满载
        elif distance >2.5:
            #推进器1700工作，半载
            return 1700
        else :
            #推进器停止
            return 1500

    # 返回差速
    # angle_to_N是根据磁场计算出的x轴指向北极的角度，逆时针为正
    # target_angle是目标偏航角度（度）
    def decision_angle(self,angle_to_N):
        vector=[(self.target_E-self.E)*86658.42656,(self.target_N-self.N)*111194.92664] #位置向量，由自身指向目标
        angle1=math.atan2(vector[1],vector[0])       #弧度-pi---+pi 
        angle1=angle1*180.0/math.pi   #角度-180---+180 
        target_angle=angle_to_N+angle1-90
        self.aim_angle=target_angle
        
        if target_angle>180.0 :
            target_angle=target_angle-360
        if target_angle<-180.0 :
            target_angle=target_angle+360
        
        pwm_differential=self.angle_pid.PID_Position_Calc(0,target_angle)
        pwm_differential=int(pwm_differential)
        #若target_angle > 当前0，PID会输出（pwm_differential）为正值，同时也对应着船逆时针转，即左转速小于右转速，所以恒有：：pwm_differential=右转速-左转速
        #注意，差速为0--800
        return pwm_differential

    def set_pid_(self,KP,KI,KD):
        self.angle_pid.set_pid(KP,KI,KD)

    def get_distance():
        return


if __name__=="__main__":
    gps_=GPS()
    while 1:
        gps_.read_NE()
        print(f'current_N:{gps_.current_N}')
        print(f'current_E:{gps_.current_E}')

