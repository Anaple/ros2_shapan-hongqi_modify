#舵机角度
CARTUEN_DEAFULT = 190

#速度
CARSPEED_DEAFULT = 99

#电机消息
#电机1 电机2 舵机1 舵机2 舵机3 舵机4
def CARTUEN_MSG(tuen1,dir1,speet1)->str:

    return f"<100,100,{tuen1},{dir1}{speet1},100,100>"


#TTY串口路径
TTY_MILLIMETER = "/dev/mt_millimeter"
TTY_IMU = "/dev/mt_imu"
TTY_THS1 ="/dev/ttyTHS0"
TTY_RFID = "/dev/ttyTHS1"
TTY_UWB = "/dev/mt_uwb"

