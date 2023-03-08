#舵机角度
CARTUEN_DEAFULT = 190
#速度
CARSPEED_DEAFULT = 10

CARSPEED_STOP = 90


#电机方向 红旗车 2 前行 1 后退
CARGEAR_N = 2
CARGEAR_R = 1

#舵机偏转方向
# 默认1  若方向相反 特殊处理 -1
CARTUEN_ANGLE = -1

#舵机偏转值
#转弯角度
CARTUEN_ANGLE_CENTEROFFSET = 2.8125




#电机消息
#speed 190不动 190<speed <260 R   120<speed<190 N 
def CARTUEN_MSG(tuen1,dir1,speet1)->str:


    return f"<100,100,{tuen1},{dir1}{speet1},100,100>"


#TTY串口路径
TTY_MILLIMETER = "/dev/mt_millimeter"
TTY_IMU = "/dev/mt_imu"
TTY_THS1 ="/dev/ttyTHS0"
TTY_RFID = "/dev/ttyTHS1"
TTY_UWB = "/dev/mt_uwb"


#Socket配置
SOCKET_IP = '192.168.0.31'
SOCKET_PORT = 9000




#DB路径
LOCAL_DB = "/home/nvidia/ro2_honqi/src/test1.db"





