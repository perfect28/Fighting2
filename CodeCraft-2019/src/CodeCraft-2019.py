# coding=utf-8
import heapq
import logging
import math
import sys
import numpy as np

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')


class Node:

    def __init__(self, node_id):
        self.id = node_id
        self.edgeList = []  # 邻接道路列表（边信息，用边，因为边含节点）
        self.vis = False
        self.dis = 0x3f3f3f3f

    def __lt__(self, other):
        return self.dis < other.dis


class Edge:
    '''
        u: start point
        v: end point
        len: length of road
        limit_rate: max speed
        num_lane: numbers of lane
        status: (num_lane,time_step) road status
    '''

    def __init__(self, road_id, u, v, road_len, limit_rate, num_lane, is_bidirection):
        self.road_id = road_id
        self.u = u
        self.v = v
        self.road_len = road_len
        self.limit_rate = limit_rate
        self.num_lane = num_lane
        self.is_bidirection = is_bidirection
        self.status = np.zeros((road_len, num_lane))
        self.car_num = 0  # 此时停在这条道路上的车的数量
        # x,y表示调度到哪辆车了
        self.x = 0  # 对应道路长度坐标
        self.y = 0  # 对应车道坐标
        # print("road(u,v,len,channel): " + str(road_id) + ' ' + str(u) + ' ' + str(v) + ' ' + str(road_len) + ' ' + str(num_lane))


class Car:

    def __init__(self, car_id, start_point, end_point, speed, start_time):
        self.id = car_id
        self.speed = speed
        self.start_point = start_point
        self.end_point = end_point
        self.start_time = start_time
        self.end_time = 100
        self.status = 0  # 车的状态，0表示等待状态，1表示终止状态
        self.path = []  # 车的导航路径，即最终结果
        self.path_id = 0  # 车当前即将开往的道路对应的path id
        self.dir = 'D'  # 下一个路口的转弯方向, 'D'|'L'|'R'
        # print("car(start->end): " + str(car_id) + ' ' + str(start_point) + ' ' + str(end_point))


class CarScheduler:
    def __init__(self):
        self.carList = {}
        self.roadList = {}
        self.crossList = {}

        self.current_time = 0  # 当前时间片
        self.max_time = 100  # 最大时间片设置为100

        self.sum_carnum = 0
        self.scheduled_carnum = 0

    def initFile(self, car_path, road_path, cross_path):
        with open(road_path, 'r') as file:
            lines = file.read().split('\n')  # store the lines in a list
            lines = [x for x in lines if len(x) > 0]  # get read of the empty lines
            lines = [x for x in lines if x[0] != '#']  # get rid of comments
            lines = [x.lstrip('(').rstrip(')') for x in lines]  # get rid of fringe whitespaces
            for line in lines:
                data = list(map(int, line.split(', ')))
                road_id, road_len, limit_rate, num_lane, u, v, is_bidirection = data
                self.roadList[road_id] = Edge(road_id, u, v, road_len, limit_rate, num_lane, is_bidirection)
                if is_bidirection == 1:  # 如果是双向道
                    self.roadList[road_id + 100000] = Edge(road_id + 100000, v, u, road_len, limit_rate, num_lane,
                                                           is_bidirection)

        with open(cross_path, 'r') as file:
            lines = file.read().split('\n')  # store the lines in a list
            lines = [x for x in lines if len(x) > 0]  # get read of the empty lines
            lines = [x for x in lines if x[0] != '#']  # get rid of comments
            lines = [x.lstrip('(').rstrip(')') for x in lines]  # get rid of fringe whitespaces
            for line in lines:
                data = list(map(int, line.split(', ')))
                cross_id = data[0]
                self.crossList[cross_id] = Node(cross_id)
                # 路口的道路信息：同一条道路，若为双向，则添加(入，出）
                # 例如,路口为u，道路为uv,则添加(vu方向id，uv方向id)
                # 如果单向，则添加(uv,-1)或者(-1,uv)
                edgeList = self.crossList[cross_id].edgeList
                for id in data[1:]:
                    if id == -1:
                        edgeList.append((id, id))
                        continue

                    road = self.roadList[id]
                    if road.is_bidirection:
                        id2 = id + 100000
                    if road.v == cross_id:  # 此路口为该边出口
                        edgeList.append((id, id2))
                    elif road.u == cross_id:  # 此路口为该边入口
                        edgeList.append((id2, id))

                # print("cross(出口边，入口边): " + str(cross_id) + ' ' + str(self.crossList[cross_id].edgeList))

        # 最后读取car的信息
        with open(car_path, 'r') as file:
            lines = file.read().split('\n')  # store the lines in a list
            lines = [x for x in lines if len(x) > 0]  # get read of the empty lines
            lines = [x for x in lines if x[0] != '#']  # get rid of comments
            lines = [x.lstrip('(').rstrip(')') for x in lines]  # get rid of fringe whitespaces
            for line in lines:
                data = list(map(int, line.split(',')))
                car_id, start_point, end_point, speed, start_time = data
                self.carList[car_id] = Car(car_id, start_point, end_point, speed, start_time)

    def calculateshortestpath(self, startNode, endNode):
        q = []
        for node in self.crossList.values():
            node.dis = 0x3f3f3f3f
            node.vis = False

        self.crossList[startNode.id].dis = 0
        self.crossList[startNode.id].vis = True
        heapq.heappush(q, startNode)

        while q:
            tmp = heapq.heappop(q)  # 获得此时的cross,即节点信息
            for _, edge_id in tmp.edgeList:  # 遍历邻接道路,后者表示从此路口出去的边id
                if edge_id == -1: continue
                edge = self.roadList[edge_id]
                u = edge.u  # ｕ应该和上边的tmp的id一致
                v = edge.v
                if self.crossList[v].vis: continue
                edge_weight = math.ceil(edge.road_len * 1.0 / edge.limit_rate)  # 边的权重为长度除以限速，粗略估计
                status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数
                tempdist = self.crossList[u].dis + edge_weight + status_weight
                if tempdist < self.crossList[v].dis:
                    if endNode.id == v:  # 到达终点
                        return tempdist
                    self.crossList[v].dis = tempdist
                    self.crossList[v].vis = True
                    heapq.heappush(q, self.crossList[v])

    '''
    当前时间片的车出库
    '''

    def driveCarInGarage(self):
        for car in self.carList.values():
            if car.start_time == self.current_time:
                # 获取该车即将开往的道路id
                # 这里需要改变一下思路，不再是一开始就知道整个PATH然后按图索骥
                # 而是每一个时间片都预估下一个road_id

                min_len = 0x3f3f3f3f

                # 遍历该出发点的每一个出口，粗略估计每条路的行驶代价
                for _, id in self.crossList[car.start_point].edgeList:
                    if id == -1: continue
                    startNode = self.crossList[self.roadList[id].v]
                    endNode = self.crossList[car.end_point]
                    dis = self.calculateshortestpath(startNode, endNode)

                    edge = self.roadList[id]
                    edge_weight = math.ceil(edge.road_len * 1.0 / edge.limit_rate)  # 边的权重为长度除以限速，粗略估计
                    status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数
                    dis += edge_weight + status_weight

                    if dis < min_len:
                        min_len = dis
                        road_id = id

                cur_road = self.roadList[road_id]
                num_lane = cur_road.num_lane
                road_len = cur_road.road_len
                status = cur_road.status
                for i in range(num_lane):  # 遍历车道
                    if status[road_len - 1, i] == 0:
                        s = min(car.speed, cur_road.limit_rate)  # 可以行驶的距离
                        for j in range(road_len - 1, road_len - s - 1, -1):  # 尝试着开更远
                            if status[j, i] != 0:  # 前方有车辆
                                j += 1
                                break
                        status[j, i] = car.id  # 调度车辆
                        cur_road.car_num += 1  # 该道路车辆＋1
                        self.sum_carnum += 1  # 调度总车辆+1
                        break

    '''
    调度某一车道车辆到终止位置
    '''

    def driveAllCarJustOnRoadToEndState(self, road_id, channel_id):
        cur_road = self.roadList[road_id]
        status = cur_road.status
        tmp_pos = 0  # 临时最前的车位置
        for i in range(cur_road.road_len):  # 从前往后枚举这条车道的车辆
            if status[i, channel_id] != 0:  # 当前位置有车
                car = self.carList[status[i, channel_id]]
                if car.status == 0:  # 状态为等待行驶
                    s = min(car.speed, cur_road.limit_rate)
                    if s > i - tmp_pos:  # 可行驶距离超过剩余到路口的距离
                        break  # 剩下的车都不改变状态
                    else:
                        status[i - s, channel_id] = status[i, channel_id]
                        status[i, channel_id] = 0
                        tmp_pos = i - s
                else:  # 状态为终止状态
                    continue

    def work(self, car_path, road_path, cross_path):
        self.initFile(car_path, road_path, cross_path)

        self.current_time = 0

        while self.current_time < self.max_time:
            self.current_time += 1  # 系统时间+1
            print("current time : " + str(self.current_time))
            scheduled_carnum = 0  # 已调度车辆数置零

            for car in self.carList.values():
                car.status = 0  # 车辆状态置为等待行驶

            while scheduled_carnum < self.sum_carnum:
                for road in self.roadList.values():
                    '''
                    / * 调整所有道路上在道路上的车辆，让道路上车辆前进，只要不出路口且可以到达终止状态的车辆
                    * 分别标记出来等待的车辆（要出路口的车辆，或者因为要出路口的车辆阻挡而不能前进的车辆）
                    * 和终止状态的车辆（在该车道内可以经过这一次调度可以行驶其最大可行驶距离的车辆） * /
                    '''
                    for channle_id in range(road.num_lane):
                        self.driveAllCarJustOnRoadToEndState(road.road_id, channle_id)

            while scheduled_carnum < self.sum_carnum:
                # driveAllWaitCar()
                for cross in self.crossList.values():
                    for roads_id in cross.edgeList:
                        print(roads_id)
                        # Direction dir = getDirection();
                        # Car car = getCarFromRoad(road, dir);
                        # if conflict: break
                        #
                        # channle = car.getChannel()
                        # car.moveToNextRoad()
                        # driveAllCarJustOnRoadToEndState(channel)

            # 车库中的车辆上路行驶
            self.driveCarInGarage()


def main():
    if len(sys.argv) != 5:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]

    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("answer_path is %s" % (answer_path))

    carScheduler = CarScheduler()

    carScheduler.work(car_path, road_path, cross_path)


# to read input file
# process
# to write output file
# python CodeCraft-2019.py ../config/car.txt ../config/road.txt ../config/cross.txt ../config/answer.txt

if __name__ == "__main__":
    main()
