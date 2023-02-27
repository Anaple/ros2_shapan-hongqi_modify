import networkx as nx


class Map():
    def create_graph(self):
        self.map = nx.DiGraph()
        nx.set_node_attributes(self.map, 50, "pose")


    def add_edge(self,node1,node2,weight,speed,angle,magnetic_state=0):
        """

        :param node1:
        :param node2:
        :param weight: 权重
        :param speed: 速度
        :param angle: 角度
        :param magnetic_state:磁条状态 0为空 1为左转 2为寻中线 3为右转
        :return:
        """
        self.map.add_edges_from([
            (node1,node2,{'weight':weight,'speed':speed,'angle':angle,'magnetic_state':magnetic_state})
        ])

    def add_node(self,node,pose_x,pose_y):
        self.map.add_node(node,pose=(pose_x,pose_y))

    def ShortestPath(self,srart_node,end_node):
        nodes = nx.shortest_path(self.map,srart_node,end_node)
        node_edge = []
        for i in range(len(nodes)-1):
            # print("nodes[i]  {} nodes[i + 1] {}".format(nodes[i], nodes[i + 1]))
            edge = self.map.get_edge_data(nodes[i], nodes[i + 1])
            # print("edge {}".format(edge))
            edge.update({"node":nodes[i+1]})
            edge.update({"ma_st_node": nodes[i]})
            node_edge.append(edge)
        return node_edge
