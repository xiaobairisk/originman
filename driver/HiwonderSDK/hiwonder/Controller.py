from . import ros_robot_controller_sdk as rrc
import time

board = rrc.Board()


class Controller:
    def __init__(self, board, time_out=50):
        self.board = board
        self.time_out = time_out

    """
        function name：  获取总线舵机温度限制(get the bus servo temperature limit)

        param：
                servo_id: 要获取温度限制的舵机id(to obtain the servo ID with temperature limit)

        return：   返回舵机温度限制(return the servo temperature limit)
    """ 
    def get_bus_servo_temp_limit(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_temp_limit(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)

    """
        function name：  获取总线舵机角度限制(get bus servo angle limit)

        param：
                servo_id: 要获取角度限制的舵机ID(to obtain the servo ID with angle limit)

        return：   返回舵机角度限制(return servo angle limit)
    """ 
    def get_bus_servo_angle_limit(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_angle_limit(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机电压限制(get bus servo voltage limit)

        param：
                servo_id: 要获取电压限制的舵机id(to obtain the servo ID with angle limit)

        return：   返回舵机电压限制(return the servo voltage limit)
    """ 
    def get_bus_servo_vin_limit(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_vin_limit(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机ID(get bus servo ID)

        param：

        return：   返回舵机ID(return servo ID)
    """      
    def get_bus_servo_id(self):
        count = 0
        while True:
            data = self.board.bus_servo_read_id()
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机位置(get bus servo position)

        param：
                servo_id: 要获取位置的舵机id(to obtain the servo ID with angle limit)

        return：   返回舵机位置(return servo position)
    """      
    def get_bus_servo_pulse(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_position(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
   

    """
        function name：  获取总线舵机电压(get bus servo voltage)

        param：
                servo_id: 要获取电压的舵机id(to obtain the servo ID with angle limit)

        return：   返回舵机电压(return servo voltage)
    """  
    def get_bus_servo_vin(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_vin(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机温度(get bus servo temperature)

        param：
                servo_id: 要获取温度的舵机id(to obtain the servo ID with angle limit)

        return：   返回舵机温度(return servo temperature)
    """  
    def get_bus_servo_temp(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_temp(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机偏差(get bus servo deviation)

        param：
                servo_id: 要获取偏差的舵机id(to obtain the servo ID with deviation)

        return：   返回舵机偏差(return servo deviation)
    """    
    def get_bus_servo_deviation(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_offset(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  设置总线舵机位置(set bus servo position)

        param：
                servo_id: 要驱动的舵机id(the servo ID to be driven)
                pulse:    舵机目标位置(servo target position)
                use_time: 转动需要的时间(the time required for rotation)
    """
    def set_bus_servo_pulse(self, servo_id, pulse, use_time):
        self.board.bus_servo_set_position(use_time/1000, [[servo_id, pulse]])

    """
        function name：  设置pwm舵机位置(set pwm servo position)

        param：
                servo_id: 要驱动的舵机id(the servo id to be driven)
                pulse:    舵机目标位置(servo target position)
                use_time: 转动需要的时间(the time required for rotation)  
    """
    def set_pwm_servo_pulse(self, servo_id, pulse, use_time):
        self.board.pwm_servo_set_position(use_time/1000, [[servo_id, pulse]])

    """
        function name：  设置总线舵机ID(set bus servo ID)

        param：
                servo_id_now: 当前的舵机ID(current servo ID)
                servo_id_new: 新的舵机ID(new servo ID)
    """
    def set_bus_servo_id(self, servo_id_now, servo_id_new):
        self.board.bus_servo_set_id(servo_id_now, servo_id_new)

    """
        function name：  设置总线舵机偏差(get bus servo deviation)

        param：
                servo_id:  要设置的舵机ID(the servo ID to be set)
                deviation: 设置的舵机偏差(the deviation of the servo to be set)
    """
    def set_bus_servo_deviation(self, servo_id, deviation):
        self.board.bus_servo_set_offset(servo_id, deviation)

    """
        function name：  设置总线舵机温度限制(setting the temperature limit for the bus servo)

        param：
                servo_id:  要设置的舵机ID(the servo ID to be set)
                temp_limit: 设置的温度限制(the temperature limit to be set)
    """
    def set_bus_servo_temp_limit(self, servo_id, temp_limit):
        self.board.bus_servo_set_temp_limit(servo_id, temp_limit)

    """
        function name：  设置总线舵机角度限制(setting the angle limit for the bus servo)

        param：
                servo_id:  要设置的舵机ID(the servo ID to be set)
                angle_limit: 设置的角度限制(the angle limit to be set)
    """
    def set_bus_servo_angle_limit(self, servo_id, angle_limit):
        self.board.bus_servo_set_angle_limit(servo_id, angle_limit)

    """
        function name：  设置总线舵机电压限制(set the voltage limit for the bus servo)

        param：
                servo_id:  要设置的舵机ID(the servo ID to be set)
                vin_limit: 设置的电压限制(set the voltage limit)
    """
    def set_bus_servo_vin_limit(self, servo_id, vin_limit):
        self.board.bus_servo_set_vin_limit(servo_id, vin_limit)

    """
        function name：  下载舵机偏差(download servo deviation)

        param：
                servo_id:  要下载偏差的舵机ID(to download the ID of the servo with deviation)
    """    
    def save_bus_servo_deviation(self, servo_id):
        self.board.bus_servo_save_offset(servo_id)

    """
        function name：  舵机掉电(servo power loss)

        param：
                servo_id:  要掉电的舵机ID(the ID of the servo to be powered off)
    """ 
    def unload_bus_servo(self, servo_id):
        self.board.bus_servo_enable_torque(servo_id, 1)

    """
        function name：  驱动蜂鸣器(drive buzzer)

        param：
                freq:       声音频率(sound frequency)
                on_time：   开启时间(turn on time)
                off_time:   关闭时间(turn off time)
                repeat：    重复次数(repetition times)
    """ 
    def set_buzzer(self, freq, on_time, off_time, repeat=1):
        self.board.set_buzzer(freq, on_time, off_time, repeat=1)

    """
        function name：  获取imu数据(get imu data)

        param：
        
        return：返回IMU数据，ax, ay, az, gx, gy, gz(return IMU data, ax, ay, az, gx, gy, gz)
    """ 
    def get_imu(self):
        count = 0
        while True:
            data = self.board.get_imu()
            count += 1
            if data is not None:
                return data
            if count > self.time_out:
                return None
            time.sleep(0.01)

    """
        function name：  数据接收使能(data reception enable)

        param：
        
        return：
    """ 
    def enable_recv(self):
        self.board.enable_reception(False)
        time.sleep(1)
        self.board.enable_reception(True)
        time.sleep(1)
        
    