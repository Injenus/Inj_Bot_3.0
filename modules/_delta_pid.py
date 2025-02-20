"""
Возвращает дельту для регулируемой величины.
"""

class PID():
    def __init__(self, min_val, max_val, p_k, i_k, d_k, dt=0.1):
        self.min_val = min_val
        self.max_val = max_val
        self.p_k = p_k
        self.i_k = i_k
        self.d_k = d_k
        self.dt = dt
        self.prev_err = 0
        self.error = None
        self.integral = 0

    def calculate(self, error, dt=-1):
        if dt != -1:
            self.dt = dt
        self.error = error

        prop = self.error-self.prev_err
        self.integral += self.error*self.dt
        if self.integral > self.max_val:
            self.integral = self.max_val
        if self.integral < -self.max_val:
            self.integral = -self.max_val
        diff = (self.error-self.prev_err)/self.dt
        self.prev_err = self.error

        self.value = prop*self.p_k+self.integral*self.i_k+diff*self.d_k
        #self.value = max(self.min_val, min(abs(self.value), self.max_val))*(self.value/abs(self.value))
        # if abs(self.value) < self.min_val:
        #     if self.value < 0:
        #         self.value = -self.min_val
        #     else:
        #         self.value = self.min_val
        # elif abs(self.value) > self.max_val:
        #     if self.value < 0:
        #         self.value = -self.max_val
        #     else:
        #         self.value = self.max_val
        return self.value

    def reset(self):
        self.integral = 0
        self.error = None
        self.prev_err = 0
    
    def auto_tune(self, target, current):
        error = target - current
        error_change = abs(error - self.prev_err)
        
        if error_change > 0.1 * abs(target):  # Если изменение ошибки заметное
            self.p_k += 0.0001
        else:
            self.p_k -= 0.0001
        
        if abs(self.integral) > 0.5 * abs(target):  # Если интеграл слишком велик
            self.i_k -= 0.00001
        else:
            self.i_k += 0.00001

        if abs(error_change) > 0.1 * abs(target):  # Если ошибка меняется слишком быстро
            self.d_k += 0.00001
        else:
            self.d_k -= 0.00001

        self.p_k = max(0, self.p_k)
        self.i_k = max(0, self.i_k)
        self.d_k = max(0, self.d_k)