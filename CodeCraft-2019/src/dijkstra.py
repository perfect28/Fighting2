import time
import heapq


class Node:

    def __init__(self, name):
        self.name = name
        self.vis = False
        self.adjacenciesList = []
        self.pre = None
        self.dis = 0x3f3f3f3f

    def __lt__(self, other):
        return self.dis < other.dis


class Edge:

    def __init__(self, weight, startvertex, endvertex):
        self.weight = weight
        self.startvertex = startvertex
        self.endvertex = endvertex


def calculateshortestpath(vertexlist, startvertex):
    q = []
    startvertex.dis = 0
    heapq.heappush(q, startvertex)

    while q:
        tmp = heapq.heappop(q)
        for edge in tmp.adjacenciesList:
            tempdist = edge.startvertex.dis + edge.weight
            if tempdist < edge.endvertex.dis:
                edge.endvertex.dis = tempdist
                edge.endvertex.pre = edge.startvertex
                heapq.heappush(q, edge.endvertex)


def getshortestpath(t):
    print("The value of it's minimum distance is: ", t.dis),
    node = t
    while node:
        print(node.name),
        node = node.pre
    print(" ")


# 主函数部分
node = []
edge = []
n = int(input())
m = int(input())
for i in range(n):
    node.append(Node(i + 1))  # 生成节点的名字
for i in range(m):
    a, b, c = map(int, input().split())  # a到b的距离为c
    edge.append(Edge(c, node[a - 1], node[b - 1]))
    node[a - 1].adjacenciesList.append(edge[i])

begin = time.perf_counter()
calculateshortestpath(node, node[0])
for i in range(n):
    getshortestpath(node[i])
end = time.perf_counter()

print("read :%f s" % (end - begin))

# 3
# 3
# 1 2 2
# 1 3 3
# 2 3 4
