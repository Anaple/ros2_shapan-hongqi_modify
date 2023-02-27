import networkx as nx
import matplotlib.pyplot as plt


def show_map(G):
    # Custom the edges:
    # font_size标签字体大小，font_color标签字体颜色,font_weight字体形式
    pos = [(0,0)]
    num = 0
    for i in G.nodes.data():
        print(i)
        pose = i[-1]['pose']
        num+=1
        pos.append(pose)
    print(num)
    nx.draw(G, with_labels=True , font_color="black", font_weight="bold",
            pos=pos,arrowsize=10,arrows=True, arrowstyle='fancy')
    plt.show()

def show_map1(G):
    # Custom the edges:
    # font_size标签字体大小，font_color标签字体颜色,font_weight字体形式
    nx.draw(G, with_labels=True, node_size=300, font_size=10, font_color="black", font_weight="bold",)
    plt.show()