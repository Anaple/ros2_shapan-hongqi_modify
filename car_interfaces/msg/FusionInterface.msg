float32  timestamp  #时间戳
uint8  id   # 车辆ID
float32  car_length         # 车长m
float32  car_width         # 车宽m
float32  car_height         # 车高m
float32  yaw         # 偏航角（0-359.99）
float32  pitch        # 俯仰角（-90-90）
float32  roll         # 横滚角（-180-180）
float32  wx         # 角速度x
float32  wy         # 角速度y
float32   wz         # 角速度z
float32  ax         # 加速度x
float32  ay         # 加速度y
float32  az         # 加速度z
float32   longitude        # 经度
float32   latitude         # 纬度
float32  height         # 高度
float32   eastvelocity         # 东向速度
float32   northvelocity        # 北向速度
float32   skyvelocity         # 天向速度
float32   car_speed    # 车辆速度（0-20km/h）
float32   steer_angle    # 车辆转角（-28-28度）
uint8  gear_pos    # 车辆档位信号（01：驻车，02：空挡，03：前进，04：后退，05：无效）
float32  brake_tq    # 制动力矩（-50-50nm）
uint8  parking_state    # 制动状态（00：驻车状态，01：驻车释放状态）
uint8  soc    # 电池当前SOC（0-100）
uint8  battery_vol    # 电池电压（0-55V）
uint8  battery_discharge_cur    # 电池放电电流（0-100A）
float32   camcenteroffset     # 目标识别车道中心线偏移量
float32   magcenteroffset     # 磁导航磁线中心线偏移量
uint16  obstaclenumber  #障碍物数量
float32[]  obstacledata  #障碍物数据
int8   lightnumber  #在线红绿灯数量
float32[]  lightdata   # 红绿灯状态数据
float32  process_time     # 进程处理时间
