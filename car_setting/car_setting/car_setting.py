#舵机角度
CARTUEN_DEAFULT = 190
#速度
CARSPEED_DEAFULT = 99


#电机消息

def CARTUEN_MSG(tuen1,dir1,speet1)->str:

    return f"<100,100,{tuen1},{dir1}{speet1},100,100>"


#TTY串口路径
TTY_MILLIMETER = "/dev/mt_millimeter"
TTY_IMU = "/dev/mt_imu"
TTY_THS1 ="/dev/ttyTCU0"
TTY_RFID = "/dev/ttyTHS1"
TTY_UWB = "/dev/mt_uwb"


#Socket配置
SOCKET_IP = '192.168.0.31'
SOCKET_PORT = 9000

#DB路径
LOCAL_DB = "/home/nvidia/ro2_honqi/src/test1.db"





