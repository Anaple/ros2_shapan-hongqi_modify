# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
from AMGraph import Map
from model import get_node_datas, get_edge_datas
from show_map import show_map, show_map1


def map_test():
    map = Map()
    map.create_graph()
    node_datas = get_node_datas()
    for i in node_datas:
        map.add_node(i.node,i.pose_x,i.pose_y)

    for i in get_edge_datas():
        map.add_edge(i.node1.node,i.node2.node,i.weight,i.speed,i.angle,i.magnetic_state)


    # show_map(map.map)
    node_edge = map.ShortestPath(1,14)
    for i in node_edge:
        print(i)
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    map_test()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
