### 导航功能包

# model.py文件
    Node_data类和Edge_data类继承peewee类的Model类，做对象关系映射，
# Map类 networkx 库
    在car_7_path_planner功能包下的AMGraph.py文件中的Map类是有向图类
    create_graph方法是创建有向图，同时为节点添加三个属性，分别为，速度，角度，节点位置信息，
    add_edge方法是向有向图添加边信息，同时添加权重，速度，角度
    add_node方法是向有向图添加节点信息
    ShortestPath方法输入起点和终点节点，返回最短路径列表
    默认使用dijkstra，计算最短路径
#show_map
    show_map.py文件提供有向图可视化
    show_map函数，首先收集节点的物理位置列表，然后通过位置信息可视化有向图
# test_map
    测试map类在amgraph_test.py文件
    test_map函数主要测试加载sqlite文件，然后读取数据，通过数据创建有向图，可视化有向图

# global_path_planning.py
    全局路径规划，加载有向图，订阅网络节点，接收到云发送目标位置，开始执行全局路径规划，有向图获取最短路径

#local_path_planning.py
    局部路径规划，订阅全局路径规划、以及UWB位置信息、障碍物信息，发布小车运动指令