float32  timestamp 
uint8  id  # 毫米波雷达ID
uint16  number  #障碍物数量
float32[]  obstacledata  #障碍物数据
float32  process_time    # 进程处理时间
int32  category     # 障碍物类别， 0点目标；1小汽车；2卡车/客车；3行人；4摩托车/电动车；5自行车；6宽大目标（如墙面）；7reserved保留； 8traffic light； 9stop sign
float32  length       # 障碍物长度m，精度为0.01
float32  width       # 障碍物宽度m，精度为0.01
float32  height       # 障碍物高度m，精度为0.01
float32  x     # 障碍物的x坐标[米]，精度为0.01
float32  y   # 障碍物的y坐标[米]，精度为0.01
float32  z    # 障碍物的z坐标[米]，精度为0.01
float32  v    # 障碍物径向速度m/s，精度为0.01
float32  latv  # 障碍物的横向速度m/s，精度为0.01
