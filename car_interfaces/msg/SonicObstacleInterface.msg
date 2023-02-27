float32  timestamp  #时间戳
uint8  id  # 超声波雷达ID
uint16  number  #障碍物数量
float32[]  obstacledata  #障碍物数据
float32  process_time    # 进程处理时间 其中obstacledata格式float32[]为一维数组，储存按照[angle, ranges,angle, ranges,……]，2个长度数据为一组
float32  angle   # 测量角度°，精度0.01
float32  ranges     # 测量的距离数据[米]，精度0.01
