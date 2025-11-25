class P_PID:
    def __init__(self, Integralmax, outputmax,Kp=1, Ki=0.3, Kd=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.PrevError = 0.0
        self.LastError = 0.0
        self.Error = 0.0
        self.DError = 0.0
        self.SumError = 0.0
        self.output = 0.0
        self.Integralmax = Integralmax
        self.outputmax = outputmax

    def set_pid(self,KP,KI,KD):
        self.Kp=KP
        self.Ki=KI
        self.Kd=KD
        # pass

    def abs_limit(self, value, ABS_MAX):
        if value > ABS_MAX:
            value = ABS_MAX
        if value < -ABS_MAX:
            value = -ABS_MAX
        return value

    def PID_Position_Calc(self, Target_val, Actual_val):
        self.Error = Target_val - Actual_val #目标-当前
        self.SumError += self.Error
        self.DError = self.Error - self.LastError
        self.output = self.Kp * self.Error + self.abs_limit(self.Ki * self.SumError, self.Integralmax) + self.Kd * self.DError
        self.LastError = self.Error

        print(f'KP:{self.Kp},KI:{self.Ki},KD:{self.Kd}')
        return self.abs_limit(self.output, self.outputmax)