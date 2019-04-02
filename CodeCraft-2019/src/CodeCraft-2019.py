# coding=utf-8
import functools
import heapq
import logging
import math
import random
import sys
import time
import copy
from queue import Queue

import numpy as np

logging.basicConfig(level=logging.DEBUG,
                    filename='./logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')


class Node:

    def __init__(self, node_id):
        self.id = node_id
        self.edgeList1 = []  # 邻接道路列表（边信息，用边，因为边含节点）
        self.edgeList2 = []  # 邻接道路列表（边信息，用边，因为边含节点）
        self.vis = False
        self.dis = 0x3f3f3f3f
        self.pre = -1

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
        self.status = np.zeros((road_len, num_lane), dtype=int)
        self.car_num = 0  # 此时停在这条道路上的车的数量
        self.mean_speed = limit_rate  # 道路的平均速度
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
        self.end_time = 10000
        self.status = 0  # 车的状态，0表示等待状态，1表示终止状态
        self.path = []  # 车的导航路径，即最终结果
        self.cur_point = start_point  # 下一个路口（即将到达)的id号
        self.last_edge = -1  # 上一轮所在道路id
        self.cur_edge = -1  # 当前所在道路id
        self.next_edge = -1  # 下一条道路id
        self.cur_lane = -1  # 当前所在车道id
        self.cur_pos = -1  # 当前所在的排数
        self.cur_dir = ''  # 下一个路口的转弯方向, 'D'|'L'|'R'
        self.sure_dir = ''
        # print("car(start->end): " + str(car_id) + ' ' + str(start_point) + ' ' + str(end_point))


class CarScheduler:
    def __init__(self):
        self.carList = {}
        self.roadList = {}
        self.crossList = {}

        # self.sorted_carList = []
        self.sorted_crossList = []
        self.finished_speedList = [0] * 20
        self.depatureList = [[] for i in range(10000)]  # key值为时间
        # self.shortestpathList = [[[] for j in range(2000)] for i in range(2000)]

        self.current_time = 0  # 当前时间片
        self.max_time = 10000  # 最大时间片设置为10000
        self.deadlock_time = 0  # 死锁次数

        # 可调参数
        self.delay_time = 1  # 车辆延迟时间，可调参数
        self.weight = 5  # 车况权值，可调参数
        self.max_schedule_carnum = 6000  # 最大同时调度车辆数，可调参数

        self.sum_carnum = 0  # 当前需要调度的车辆数
        self.finished_carnum = 0  # 已经到达终点的车辆数
        self.schedule_carList = []  # 车辆调度完毕则从该列表中移除

        self.scheduled_carnum = 0  # 当前已调度的车辆数
        self.cur_schedule_carList = []  # 当前已调度的车辆列表，为了debug哪些车辆出了问题
        self.last_scheduled_carnum = -1  # 上一轮已调度的车辆数，用来检查死锁
        self.cross_scheduled_carnum = 0

        # 解决死锁，需要回滚
        self.back_car_dir = {}  # 上一轮车的方向信息
        self.back_carList = {}  # 上一轮车辆信息
        self.back_roadList = {}  # 上一轮道路信息
        self.back_speedList = []
        self.back_sum_carnum = 0
        self.back_finished_carnum = 0
        self.back_schedule_carList = []

    def initFile(self, car_path, road_path, cross_path):
        with open(road_path, 'r', encoding='UTF-8') as file:
            lines = file.read().split('\n')  # store the lines in a list
            lines = [x for x in lines if len(x) > 0]  # get read of the empty lines
            lines = [x for x in lines if x[0] != '#']  # get rid of comments
            lines = [x.lstrip('(').rstrip(')') for x in lines]  # get rid of fringe whitespaces
            for line in lines:
                data = list(map(int, line.split(', ')))
                road_id, road_len, limit_rate, num_lane, u, v, is_bidirection = data
                self.roadList[road_id] = Edge(road_id, u, v, road_len, limit_rate, num_lane, is_bidirection)
                if is_bidirection == 1:  # 如果是双向道
                    self.roadList[road_id + 100000] = \
                        Edge(road_id + 100000, v, u, road_len, limit_rate, num_lane, is_bidirection)

        with open(cross_path, 'r', encoding='UTF-8') as file:
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
                edgeList1 = self.crossList[cross_id].edgeList1  # 入口边
                edgeList2 = self.crossList[cross_id].edgeList2  # 出口边
                for id in data[1:]:
                    if id == -1:
                        edgeList1.append(id)
                        edgeList2.append(id)
                        continue

                    road = self.roadList[id]
                    if road.is_bidirection:
                        id2 = id + 100000
                    else:
                        id2 = -1

                    if road.v == cross_id:  # 此路口为该边出口
                        edgeList1.append(id)
                        edgeList2.append(id2)
                    elif road.u == cross_id:  # 此路口为该边入口
                        edgeList1.append(id2)
                        edgeList2.append(id)

                # print("cross(出口边，入口边): " + str(cross_id) + ' ' + str(self.crossList[cross_id].edgeList))

        # 最后读取car的信息
        with open(car_path, 'r', encoding='UTF-8') as file:
            lines = file.read().split('\n')  # store the lines in a list
            lines = [x for x in lines if len(x) > 0]  # get read of the empty lines
            lines = [x for x in lines if x[0] != '#']  # get rid of comments
            lines = [x.lstrip('(').rstrip(')') for x in lines]  # get rid of fringe whitespaces
            for line in lines:
                data = list(map(int, line.split(',')))
                car_id, start_point, end_point, speed, start_time = data

                if speed / 2 <= 2:
                    start_time += 750
                elif speed / 2 == 3:
                    start_time += 550
                elif speed / 2 == 4:
                    start_time += 400
                elif speed / 2 == 5:
                    start_time += 250
                elif speed / 2 == 6:
                    start_time += 200
                elif speed / 2 == 7:
                    start_time += 100

                self.carList[car_id] = Car(car_id, start_point, end_point, speed, start_time)
                self.depatureList[start_time].append(car_id)
                # if start_time in self.depatureList.keys():
                #     self.depatureList[start_time].append(car_id)
                # else:
                #     self.depatureList[start_time] = [car_id]

            speedList = [0 for i in range(20)]  # key值为时间
            for car in self.carList.values():
                speedList[car.speed] += 1

            # print("speed 4: " + str(speedList[4]))
            # print("speed 6: " + str(speedList[6]))
            # print("speed 8: " + str(speedList[8]))
            # print("speed 10: " + str(speedList[10]))
            # print("speed 12: " + str(speedList[12]))
            # print("speed 14: " + str(speedList[14]))
            # print("speed 16: " + str(speedList[16]))

        self.sorted_crossList = sorted(self.crossList.values(), key=lambda d: d.id)
        # self.sorted_carList = sorted(self.carList.values(), key=lambda d: d.id)
        # self.sorted_carList = sorted(self.carList.values(), key=lambda d: (d.speed, -d.id), reverse=True)

    def calculateshortestpath(self, cur_road, car_speed, startNode, endNode):
        if startNode.id == endNode.id:  # 直接返回！！
            return 0

        q = []
        for node in self.crossList.values():
            node.dis = 0x3f3f3f3f
            node.vis = False
            node.pre = -1

        self.crossList[startNode.id].dis = 0
        self.crossList[startNode.id].vis = True
        heapq.heappush(q, startNode)

        while q:
            nowNode = heapq.heappop(q)  # 获得此时的cross,即节点信息
            for road_id in nowNode.edgeList2:  # 遍历邻接道路,后者表示从此路口出去的边id
                if road_id == -1: continue
                if cur_road != -1 and math.fabs(road_id - cur_road) == 100000: continue  # 去掉来时的方向
                edge = self.roadList[road_id]
                u = edge.u  # ｕ应该和上边的nowNode的id一致
                v = edge.v
                if self.crossList[v].vis: continue
                edge_weight = math.ceil(edge.road_len * 1.0 / min(car_speed, edge.limit_rate))  # 边的权重为长度除以限速，粗略估计
                status_weight = math.floor(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数

                cross_weight = 0
                # 考虑ｖ节点的路况
                # for r_id in self.crossList[v].edgeList1:  # 考虑入边
                #     if r_id == -1: continue
                #     cross_weight += self.roadList[r_id].car_num

                speed_weight = 0
                # speed_weight = abs(car_speed - edge.mean_speed)
                # speed_weight *= speed_weight

                tempdist = self.crossList[u].dis + edge_weight + self.weight * status_weight \
                           + cross_weight + speed_weight

                if tempdist < self.crossList[v].dis:
                    self.crossList[v].dis = tempdist
                    self.crossList[v].vis = True
                    # self.crossList[v].pre = u
                    self.crossList[v].pre = edge
                    if endNode.id == v:  # 到达终点
                        return tempdist
                    heapq.heappush(q, self.crossList[v])

    def spfa(self, cur_road, car_speed, startNode, endNode):
        if startNode.id == endNode.id:  # 直接返回！！
            return 0

        for node in self.crossList.values():
            node.dis = 0x3f3f3f3f
            node.vis = False
            node.pre = -1

        self.crossList[startNode.id].dis = 0
        self.crossList[startNode.id].vis = True
        q = Queue()
        q.put(startNode)

        while not q.empty():
            nowNode = q.get()
            nowNode.vis = False
            for road_id in nowNode.edgeList2:  # 遍历邻接道路,后者表示从此路口出去的边id
                if road_id == -1: continue
                if cur_road != -1 and math.fabs(road_id - cur_road) == 100000: continue  # 去掉来时的方向
                edge = self.roadList[road_id]
                u = edge.u  # ｕ应该和上边的tmp的id一致
                v = edge.v

                edge_weight = math.ceil(edge.road_len * 1.0 / min(car_speed, edge.limit_rate))  # 边的权重为长度除以限速，粗略估计
                status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重
                tempdist = self.crossList[u].dis + edge_weight + self.weight * status_weight

                if tempdist < self.crossList[v].dis:
                    self.crossList[v].dis = tempdist
                    self.crossList[v].pre = u
                    if not self.crossList[v].vis:
                        self.crossList[v].vis = True
                        # if endNode.id == v:  # 到达终点
                        #     return tempdist
                        q.put(self.crossList[v])

    def preShortestPath(self):
        for startNode in self.crossList.values():
            start_id = startNode.id
            for endNode in self.crossList.values():
                end_id = endNode.id
                dis = self.calculateshortestpath(-1, 20, startNode, endNode)  # 此时只能忽略车速
                path = []
                nowNode = endNode
                while nowNode.pre != -1:
                    path.append(nowNode.id)
                    nowNode = self.crossList[nowNode.pre]

                # 二维列表
                self.shortestpathList[start_id][end_id] = path
                # 二维字典创建
                # if start_id in self.shortestpathList.keys():
                #     self.shortestpathList[start_id].update({end_id: path})
                # else:
                #     self.shortestpathList.update({start_id: {end_id: path}})

    def driveCarInGarage(self):
        # 这里需要慎重考虑一下，并不是到时间就发车，应该根据路况来定，以免发生死堵
        # 1. 对于同一时间同一地点，速度快的先发车 ！！此项违反交通规则！
        # 2. 当前路况态度，延缓发车，避免死锁

        # if self.current_time not in self.depatureList.keys():
        #     return # 没有可以出发的车辆
        delay_time = self.delay_time

        for car_id in sorted(self.depatureList[self.current_time]):  # 按id号顺序发车!!
            car = self.carList[car_id]  # 重新获取该车的引用
            # if car.id == 10035:
            #     print('debug')
            # delay_time = math.ceil(car.speed)
            if len(self.schedule_carList) >= self.max_schedule_carnum:
                car.start_time += delay_time
                self.depatureList[self.current_time + delay_time].append(car.id)
                continue

            startNode = self.crossList[car.start_point]
            endNode = self.crossList[car.end_point]

            dis = self.calculateshortestpath(-1, car.speed, startNode, endNode)
            # dis = self.spfa(-1, car.speed, startNode, endNode)

            # 记录path为交点的做法
            # path = []
            # nowNode = endNode
            # while nowNode.pre != -1:
            #     path.append(nowNode.id)
            #     nowNode = self.crossList[nowNode.pre]
            # cross_id = path[-1]
            #
            # # path = self.shortestpathList[startNode.id][endNode.id]
            # # cross_id = path[-1]
            #
            # for road_id in self.crossList[car.cur_point].edgeList2:
            #     if road_id == -1: continue
            #     road = self.roadList[road_id]
            #     if road.v == cross_id:
            #         break

            # 记录path为边的做法
            road_id = -1
            nowNode = endNode
            while nowNode.pre != -1:
                car.next_edge = road_id
                road_id = nowNode.pre.road_id
                nowNode = self.crossList[nowNode.pre.u]

            if road_id == -1: continue
            cur_road = self.roadList[road_id]
            num_lane = cur_road.num_lane
            road_len = cur_road.road_len

            if cur_road.car_num > num_lane * road_len * 4 / 5:
                car.start_time += delay_time
                self.depatureList[self.current_time + delay_time].append(car.id)
                continue

            is_depature = False
            status = cur_road.status
            for i in range(num_lane):  # 遍历车道
                if status[road_len - 1, i] == 0:
                    s = min(car.speed, cur_road.limit_rate)  # 可以行驶的距离
                    for j in range(road_len - 1, road_len - s - 1, -1):  # 尝试着开更远
                        if status[j, i] != 0:  # 前方有车辆
                            j += 1
                            break

                    # 统计量
                    self.sum_carnum += 1  # 调度总车辆+1
                    self.schedule_carList.append(car.id)
                    # 道路
                    status[j, i] = car.id  # 调度车辆
                    cur_road.car_num += 1  # 该道路车辆＋1
                    cur_road.mean_speed = (cur_road.mean_speed * (
                            cur_road.car_num - 1) + car.speed) * 1.0 / cur_road.car_num
                    # 车辆
                    car.status = 1
                    car.path.append(cur_road.road_id)
                    car.cur_lane = i
                    car.cur_pos = j
                    car.last_edge = car.cur_edge
                    car.cur_edge = cur_road.road_id
                    car.cur_point = cur_road.v
                    # print(str(car.id) + " depart!!")
                    is_depature = True
                    break

            if not is_depature:  # 表示此次没有出库，出发时间后移1
                car.start_time += delay_time
                self.depatureList[self.current_time + delay_time].append(car.id)

    def driveAllCarJustOnRoadToEndState(self, road_id, channel_id, st):
        road = self.roadList[road_id]
        status = road.status
        tmp_pos = -1  # 临时最前的车位置!!初始值为-1
        flag = False  # flag为False，表示还未确定后面的车是否可以直接完成调度

        for i in range(st + 1):
            if status[i, channel_id] != 0:  # 当前位置有车
                car = self.carList[status[i, channel_id]]
                if car.status == 1:
                    flag = True
                    tmp_pos = i

        for i in range(st + 1, road.road_len):  # 从前往后枚举这条车道的车辆
            if status[i, channel_id] != 0:  # 当前位置有车
                car = self.carList[status[i, channel_id]]
                if car.id == 79161:
                    print('debug')
                if car.status == 0:  # 状态为等待行驶
                    s = min(car.speed, road.limit_rate)
                    if flag:  # 已经有车终止，后面的车都可以终止
                        if s >= i - tmp_pos:  # 这里一定要加等于！！，否则之前的车就被覆盖了
                            status[tmp_pos + 1, channel_id] = status[i, channel_id]  # 紧贴上一辆车后面！！
                            car.cur_pos = tmp_pos + 1
                            if tmp_pos + 1 != i:  # !！避免车没有动的情况
                                status[i, channel_id] = 0
                            tmp_pos = tmp_pos + 1
                        else:
                            status[i - s, channel_id] = status[i, channel_id]
                            car.cur_pos = i - s
                            tmp_pos = i - s
                            status[i, channel_id] = 0
                        # 统计量
                        self.scheduled_carnum += 1
                        self.cross_scheduled_carnum += 1
                        self.cur_schedule_carList.remove(car.id)
                        # 车辆
                        car.status = 1  # 车状态变为终止，其他不变
                        car.last_edge = car.cur_edge
                    else:
                        if s >= i - tmp_pos:
                            tmp_pos = i
                            continue
                        else:
                            status[i - s, channel_id] = status[i, channel_id]
                            car.cur_pos = i - s
                            tmp_pos = i - s
                            status[i, channel_id] = 0
                            # 统计量
                            self.scheduled_carnum += 1
                            self.cross_scheduled_carnum += 1
                            self.cur_schedule_carList.remove(car.id)
                            # 车辆
                            car.status = 1  # 车状态变为终止，其他不变
                            car.last_edge = car.cur_edge
                            flag = True  # 标记前方有标记为终态的车
                else:  # 状态为终止状态
                    flag = True  # 标记前方有标记为终态的车
                    tmp_pos = i  # !!一定要更新这里!!

    # 不需要求三个方向的最短路，可以优化！
    def get_direction(self, car):
        if car.id == 79161:
            print('debug')

        if car.cur_edge == -1:  # 初始方向都定为直行
            car.cur_dir = 'D'
            return

        if car.cur_dir != '' and car.last_edge == car.cur_edge:  # 不用改变方向
            return

        if car.next_edge != -1:
            road_id = car.next_edge
            car.next_edge = -1
            # print(car.id)
            # 根据下条路的road_id和此时的cur_edge求得转弯方向
            idx1 = self.crossList[car.cur_point].edgeList1.index(car.cur_edge)  # cur_edge是入边方向
            idx2 = self.crossList[car.cur_point].edgeList2.index(road_id)  # road_id是出边方向
            tmp = (idx2 + 4 - idx1) % 4
            if tmp == 1:
                car.cur_dir = 'L'
            elif tmp == 2:
                car.cur_dir = 'D'
            elif tmp == 3:
                car.cur_dir = 'R'
            return

        # cnt = 0
        # for ids in self.crossList[car.cur_point].edgeList2:
        #     if car.cur_edge != -1 and math.fabs(ids - car.cur_edge) == 100000:
        #         cnt += 1

        road = self.roadList[car.cur_edge]
        startNode = self.crossList[road.v]
        endNode = self.crossList[car.end_point]

        dis = self.calculateshortestpath(car.cur_edge, car.speed, startNode, endNode)
        # dis = self.spfa(car.cur_edge, car.speed, startNode, endNode)

        # 记录path为交点的做法
        # path = []
        # nowNode = endNode
        # while nowNode.pre != -1:
        #     path.append(nowNode.id)
        #     nowNode = self.crossList[nowNode.pre]
        # cross_id = path[-1]

        # path2 = self.shortestpathList[startNode.id][endNode.id]
        # cross_id = path2[-1]
        # if cross_id == road.u:  # 如果预处理的结果是反向，得重新寻找最短路
        #     dis = self.calculateshortestpath(car.cur_edge, car.speed, startNode, endNode)
        #
        #     path = []
        #     nowNode = endNode
        #     while nowNode.pre != -1:
        #         path.append(nowNode.id)
        #         nowNode = self.crossList[nowNode.pre]
        #     cross_id = path[-1]

        # for road_id in self.crossList[car.cur_point].edgeList2:
        #     if road_id == -1: continue
        #     road = self.roadList[road_id]
        #     if road.v == cross_id:
        #         break

        # print(str(startNode.id) + "->" + str(endNode.id))
        if startNode.id == endNode.id:
            car.cur_dir = 'D'
            car.next_edge = -1
        else:
            # 记录path为边的做法
            road_id = -1
            nowNode = endNode
            while nowNode.pre != -1:
                car.next_edge = road_id
                road_id = nowNode.pre.road_id
                # print(road_id)
                nowNode = self.crossList[nowNode.pre.u]

            # 根据下条路的road_id和此时的cur_edge求得转弯方向
            idx1 = self.crossList[car.cur_point].edgeList1.index(car.cur_edge)  # cur_edge是入边方向
            idx2 = self.crossList[car.cur_point].edgeList2.index(road_id)  # road_id是出边方向
            tmp = (idx2 + 4 - idx1) % 4
            if tmp == 1:
                car.cur_dir = 'L'
            elif tmp == 2:
                car.cur_dir = 'D'
            elif tmp == 3:
                car.cur_dir = 'R'

    def dealwith_deadlock(self):
        # random.shuffle(self.cur_schedule_carList)  # 打乱顺序
        # 更新方向
        for i in self.cur_schedule_carList:
            tmp_car = self.carList[i]
            # print(tmp_car.cur_dir + " " + str(tmp_car.cur_edge) + " " + str(tmp_car.cur_point))
            tmp_road = self.roadList[tmp_car.cur_edge]
            if tmp_road.status[tmp_road.x][tmp_road.y] == tmp_car.id:  # 此车是路口第一优先级
                ran_num = random.randint(0, 1)
                print(str(tmp_car.id) + " " + tmp_car.cur_dir + " "
                      + str(tmp_car.cur_edge) + " " + str(tmp_car.cur_point))
                if tmp_car.cur_dir == 'R':
                    idx1 = self.crossList[tmp_car.cur_point].edgeList1.index(
                        tmp_car.cur_edge)
                    idx2 = (idx1 + 2) % 4
                    left_road_id = self.crossList[tmp_car.cur_point].edgeList2[idx2]
                    if left_road_id == -1:
                        continue
                    else:
                        tmp_car.cur_dir = 'D'
                        tmp_car.sure_dir = ''
                        break
                elif tmp_car.cur_dir == 'L':
                    idx1 = self.crossList[tmp_car.cur_point].edgeList1.index(
                        tmp_car.cur_edge)
                    idx2 = (idx1 + 2) % 4
                    right_road_id = self.crossList[tmp_car.cur_point].edgeList2[idx2]
                    if right_road_id == -1:
                        continue
                    else:
                        tmp_car.cur_dir = 'D'
                        tmp_car.sure_dir = ''
                        break

        # 由于回滚时，本轮还没有车出库，所有改变状态的都是上一轮的车
        self.schedule_carList = self.back_schedule_carList[:]
        for car_id in self.schedule_carList:  # 只管调度中的车辆
            car = self.carList[car_id]
            if car.cur_dir == '':
                self.back_car_dir[car_id] = car.sure_dir  # 本轮已经调度过的车
            else:
                self.back_car_dir[car_id] = car.cur_dir

        for car in self.back_carList.values():
            back_car = copy.copy(car)
            back_car.path = car.path[:]
            self.carList[car.id] = back_car
            # print(str(back_car.id) + " " + str(back_car.start_time))

        # self.sorted_carList = sorted(self.carList.values(), key=lambda d: d.id)

        for road in self.back_roadList.values():
            back_road = copy.copy(road)
            back_road.status = road.status.copy()
            self.roadList[road.road_id] = back_road

        self.sum_carnum = self.back_sum_carnum
        self.finished_carnum = self.back_finished_carnum
        self.current_time -= 1
        self.finished_speedList = self.back_speedList[:]

    def update_xy(self, road):
        # 更新x,y坐标信息
        car_id = road.status[road.x][road.y]
        while car_id == 0 or self.carList[car_id].status == 1:
            # 无车或者已经被调度
            road.y += 1
            if road.y >= road.num_lane:
                road.x += 1
                road.y = 0
            if road.x >= road.road_len: break
            car_id = road.status[road.x][road.y]

    def work(self, car_path, road_path, cross_path):
        self.initFile(car_path, road_path, cross_path)

        self.current_time = 0

        deadlock_flag = False
        sum_start = time.clock()

        while self.current_time < self.max_time and self.finished_carnum < len(self.carList):
            start = time.clock()
            self.current_time += 1  # 系统时间+1
            print("--------------current time---------------- : " + str(self.current_time))
            print("-------------current finish--------------- : " + str(self.finished_carnum))
            print("-------------current schedule--------------- : " + str(self.sum_carnum))
            # print("speed 2: " + str(self.finished_speedList[2]))
            # print("speed 4: " + str(self.finished_speedList[4]))
            # print("speed 6: " + str(self.finished_speedList[6]))
            # print("speed 8: " + str(self.finished_speedList[8]))
            # print("speed 10: " + str(self.finished_speedList[10]))
            # print("speed 12: " + str(self.finished_speedList[12]))
            # print("speed 14: " + str(self.finished_speedList[14]))
            # print("speed 16: " + str(self.finished_speedList[16]))
            if self.current_time == 59:
                print("debug")

            tmp_start = time.clock()
            self.scheduled_carnum = 0  # 已调度车辆数置零
            self.cur_schedule_carList = []
            # self.preShortestPath()  # 预处理最短路径(测试很慢^)
            # for car in self.carList.values():
            for car_id in self.schedule_carList:  # 只管调度中的车辆
                car = self.carList[car_id]
                self.cur_schedule_carList.append(car_id)
                car.status = 0  # 车辆状态置为等待行驶
                if deadlock_flag:  # 若上一轮发生死锁，不用重新找方向
                    # if car_id == 71380:
                    #     print("debug")
                    if car_id in self.back_car_dir.keys():
                        # if self.back_car_dir[car_id] == '':  # 说明上次执行过这个方向了
                        #     car.cur_dir = car.sure_dir
                        #     # self.get_direction(car)  # 获取车辆方向
                        # else:
                        car.cur_dir = self.back_car_dir[car_id]
                        car.next_edge = -1  # 有的车被强制改变方向！！
                    else:
                        print("wrong!!!!!!")
                else:
                    # 这里需要优化，每次求方向代价太大，且不必要
                    # 只要能到路口的车求方向，其他车方向不变
                    if car.cur_pos < min(car.speed, self.roadList[car.cur_edge].limit_rate):
                        if car.cur_point != car.end_point:
                            self.get_direction(car)  # 获取车辆方向
                        else:
                            car.cur_dir = 'D'
                car.sure_dir = ''  # 每轮重置
                # print("car" + str(car.id) + ": " + car.cur_dir + "\t" + str(car.path))

            tmp_end = time.clock()
            print('Preprocess Car time: %s Seconds' % (tmp_end - tmp_start))
            tmp_start = time.clock()

            for road in self.roadList.values():
                '''
                调整所有道路上在道路上的车辆，让道路上车辆前进，
                只要不出路口且可以到达终止状态的车辆
                '''
                for channle_id in range(road.num_lane):
                    self.driveAllCarJustOnRoadToEndState(road.road_id, channle_id, -1)

                if road.car_num == 0:
                    road.x = road.road_len
                    continue
                road.x = 0
                road.y = 0
                # 更新x,y坐标信息
                self.update_xy(road)

            tmp_end = time.clock()
            print('Preprocess Road time: %s Seconds' % (tmp_end - tmp_start))

            tmp_start = time.clock()
            self.last_scheduled_carnum = -1
            has_finined_car = False
            deadlock_flag = False  # 还原死锁标志
            while self.scheduled_carnum < self.sum_carnum:
                if self.last_scheduled_carnum != -1:
                    if not has_finined_car and self.last_scheduled_carnum == self.scheduled_carnum:
                        print("deadlock!!!!")
                        self.deadlock_time += 1
                        deadlock_flag = True
                        self.dealwith_deadlock()
                        break
                        # 直接改变方向的想法不可行！因为很可能会改变本轮的调度优先级次序

                has_finined_car = False
                self.last_scheduled_carnum = self.scheduled_carnum
                print("current scheduled_carnum: " + str(self.scheduled_carnum))
                print("current sum_carnum: " + str(self.sum_carnum))
                # driveAllWaitCar()
                for cross in self.sorted_crossList:

                    self.cross_scheduled_carnum = 0
                    last_cross_scheduled_carnum = -1

                    while True:
                        if last_cross_scheduled_carnum == self.cross_scheduled_carnum:
                            break
                        last_cross_scheduled_carnum = self.cross_scheduled_carnum

                        def cmp(u, v):
                            if u > 100000: u -= 100000
                            if v > 100000: v -= 100000
                            if u < v:
                                return -1
                            else:
                                return 1

                        for road_id in sorted(cross.edgeList1,
                                              key=functools.cmp_to_key(cmp)):  # 按id顺序遍历该路口的邻接边(以此路口为出口的)
                            # print(str(cross.id) + ': ' + str(road_id))
                            if road_id == -1: continue
                            road = self.roadList[road_id]

                            # 这里是调度的核心部分，即路口处的调度
                            # 调度步骤如下：
                            # 1. 从x,y开始，得到当前应该调度的一辆车
                            # 2. 按情况分析，对此车进行调度：
                            #   (1)L: 左转，需要考虑直行的车，即edgeList1[(i+3)%4]道路上优先级最高的车是否直行
                            #   (2)D: 直行，直接判断是否能过路口
                            #   (3)R: 右转，需要考虑直行和左转的车，即edgeList1[(i+1)%4]上直行的车，
                            # 和edgeList1[(i+2)%4]左转的车
                            # 3. 此车若能调度，则将该车道的车都调度一次,然后继续这条道路下一辆车；
                            # 否则，循环到下一条道路

                            while True:  # 循环执行，若能调度，尽可能多的调度
                                # 1. 获取调度车辆
                                if road.x >= road.road_len: break  # 表示这条道路已调度完成
                                car_id = road.status[road.x][road.y]
                                # print(str(road.road_id) + "\t" + str(car_id))
                                if car_id == 79161:
                                    print("debug")
                                car = self.carList[car_id]

                                # 终止条件： 车的cur_point即end_point，此时不用再考虑冲突
                                if car.cur_point == car.end_point:
                                    has_finined_car = True
                                    # 统计量
                                    self.cur_schedule_carList.remove(car.id)
                                    self.sum_carnum -= 1  # 调度总车辆-1
                                    self.schedule_carList.remove(car.id)
                                    self.finished_carnum += 1
                                    self.cross_scheduled_carnum += 1
                                    self.finished_speedList[car.speed] += 1
                                    # 道路
                                    road = self.roadList[car.cur_edge]
                                    road.status[road.x][road.y] = 0  # 清理车辆
                                    road.car_num -= 1  # 该道路车辆-1

                                    if road.car_num == 0:
                                        road.mean_speed = road.limit_rate
                                    else:
                                        road.mean_speed = (road.mean_speed * (
                                                road.car_num + 1) - car.speed) * 1.0 / road.car_num

                                    self.driveAllCarJustOnRoadToEndState(car.cur_edge, car.cur_lane, road.x)
                                    # 更新x,y坐标信息
                                    self.update_xy(road)
                                    continue  # 继续这条车道！！

                                idx = cross.edgeList1.index(road_id)  # 表示当前的road_id
                                # 2. 判断是否有冲突
                                conflict = False
                                if car.cur_dir == 'L':
                                    # 考虑右边直行的车
                                    road_onright_id = cross.edgeList1[(idx + 3) % 4]
                                    if road_onright_id != -1:
                                        road_onright = self.roadList[road_onright_id]
                                        if road_onright.x < road_onright.road_len:
                                            car2_id = road_onright.status[road_onright.x][road_onright.y]
                                            car2 = self.carList[car2_id]
                                            if car2.cur_dir == 'D':
                                                conflict = True
                                elif car.cur_dir == 'R':
                                    # 考虑左边直行的车
                                    road_onleft_id = cross.edgeList1[(idx + 1) % 4]
                                    if road_onleft_id != -1:
                                        road_onleft = self.roadList[road_onleft_id]
                                        if road_onleft.x < road_onleft.road_len:
                                            car3_id = road_onleft.status[road_onleft.x][road_onleft.y]
                                            car3 = self.carList[car3_id]
                                            if car3.cur_dir == 'D':
                                                conflict = True
                                    # 考虑对面左转的车
                                    road_towards_id = cross.edgeList1[(idx + 2) % 4]
                                    if road_towards_id != -1:
                                        road_towards = self.roadList[road_towards_id]
                                        if road_towards.x < road_towards.road_len:
                                            car4_id = road_towards.status[road_towards.x][road_towards.y]
                                            car4 = self.carList[car4_id]
                                            if car4.cur_dir == 'L':
                                                conflict = True

                                # 3. 若可以调度，对当前车道进行调度
                                if conflict: break  # 若冲突，此道路直接跳过！！
                                if car.cur_dir == 'L':  # 左转
                                    cur_road_id = cross.edgeList2[(idx + 1) % 4]
                                elif car.cur_dir == 'D':  # 直行
                                    cur_road_id = cross.edgeList2[(idx + 2) % 4]
                                else:  # car.cur_dir == 'R':  # 右转
                                    cur_road_id = cross.edgeList2[(idx + 3) % 4]
                                # print(str(car.id) + " " + car.cur_dir + " " + str(cur_road_id))
                                cur_road = self.roadList[cur_road_id]
                                num_lane = cur_road.num_lane
                                road_len = cur_road.road_len

                                s = min(car.speed, cur_road.limit_rate)  # 可以行驶的距离
                                # 按规则减去之前那条路已经行驶的距离
                                if s > road.x:
                                    s -= road.x
                                else:
                                    s = 0

                                # s = 0时需要特殊处理
                                if s == 0:
                                    # 统计量
                                    self.scheduled_carnum += 1  # 调度车辆+1
                                    self.cross_scheduled_carnum += 1
                                    self.cur_schedule_carList.remove(car.id)
                                    # 道路 road.car_num 不变
                                    road.status[road.x][road.y] = 0
                                    road.status[0][road.y] = car.id
                                    # 车辆
                                    car.cur_pos = 0
                                    car.status = 1
                                    car.last_edge = car.cur_edge
                                    self.driveAllCarJustOnRoadToEndState(car.cur_edge, car.cur_lane, road.x)
                                    # 更新x,y坐标信息
                                    self.update_xy(road)
                                    continue
                                else:
                                    # 理一下思路：
                                    # 按id遍历车道，若车道最末尾有车且状态为1，那么寻找下一车道；
                                    # 若所有车道均不可,则按堵死情况处理;
                                    # is_traffic_jam记录是否所有车道均有车且已经状态为1,所有初值为True
                                    # !不需要用到is_traffic_jam,如果有车需要等待,则已经跳出循环
                                    # 若车道最末尾有车且状态为0,那么此时应该等待,跳转到下一个道路

                                    wait_flag = False  # 表示前方是否有等待车辆
                                    for i in range(num_lane):  # 遍历车道
                                        if cur_road.status[road_len - 1, i] == 0:
                                            for j in range(road_len - 1, road_len - s - 1, -1):  # 尝试着开更远
                                                if cur_road.status[j, i] != 0:  # 前方有车辆
                                                    front_car = self.carList[cur_road.status[j, i]]
                                                    if front_car.status == 0:  # 前车还在等待，此车不能调度
                                                        wait_flag = True
                                                        break
                                                    j += 1
                                                    break

                                            if wait_flag: break

                                            # 统计量
                                            self.scheduled_carnum += 1  # 调度车辆+1
                                            self.cross_scheduled_carnum += 1
                                            self.cur_schedule_carList.remove(car.id)
                                            # 道路
                                            cur_road.status[j, i] = car.id  # 调度车辆
                                            cur_road.car_num += 1  # 该道路车辆＋1
                                            cur_road.mean_speed = (cur_road.mean_speed * (
                                                    cur_road.car_num - 1) + car.speed) * 1.0 / cur_road.car_num

                                            road.status[road.x][road.y] = 0
                                            road.car_num -= 1

                                            if road.car_num == 0:
                                                road.mean_speed = road.limit_rate
                                            else:
                                                road.mean_speed = (road.mean_speed * (
                                                        road.car_num + 1) - car.speed) * 1.0 / road.car_num

                                            # 车辆
                                            last_lane = car.cur_lane

                                            car.path.append(cur_road.road_id)
                                            car.status = 1
                                            car.cur_point = cur_road.v
                                            car.last_edge = car.cur_edge
                                            car.cur_edge = cur_road.road_id
                                            car.cur_lane = i
                                            car.cur_pos = j
                                            car.sure_dir = car.cur_dir
                                            car.cur_dir = ''
                                            if car.cur_edge == car.next_edge:
                                                car.next_edge = -1

                                            # 信息全部更新之后才可以调度该条车道
                                            self.driveAllCarJustOnRoadToEndState(road.road_id, last_lane, road.x)
                                            # 更新x,y坐标信息
                                            self.update_xy(road)
                                            break  # 这里跳出，是已经找到可以行驶的车道了，不用遍历后面的车道
                                        else:
                                            front_car = self.carList[cur_road.status[road_len - 1, i]]
                                            if front_car.status == 0:  # 未调度
                                                wait_flag = True
                                                break
                                            continue  # 寻找下一个可用车道

                                    if wait_flag: break  # 表明第一优先级的车需要等待,跳出该道路

                                # ！！一定要处理路口堵死的情况
                                if car.status == 0:  # 表示这里没有冲突，但是前进的道路上堵着了
                                    # 路口堵死，调度到路口最前处, 处理和s=0一样
                                    # 统计量
                                    self.scheduled_carnum += 1  # 调度车辆+1
                                    self.cross_scheduled_carnum += 1
                                    self.cur_schedule_carList.remove(car.id)
                                    # 道路 road.car_num 不变
                                    road.status[road.x][road.y] = 0
                                    road.status[0][road.y] = car.id

                                    # 车辆
                                    car.cur_pos = 0
                                    car.status = 1
                                    car.last_edge = car.cur_edge
                                    self.driveAllCarJustOnRoadToEndState(car.cur_edge, car.cur_lane, road.x)
                                    # 更新x,y坐标信息
                                    self.update_xy(road)
                                # break  # 这里跳出，是跳出当前的道路循环!!wrong!
                        # break

            tmp_end = time.clock()
            print('Scheduling time: %s Seconds' % (tmp_end - tmp_start))

            if not deadlock_flag:  # 若未发生死锁，存储上一轮的所有状态信息
                tmp_start = time.clock()

                # 车库中的车辆上路行驶
                self.driveCarInGarage()

                tmp_end = time.clock()
                print('Depart time: %s Seconds' % (tmp_end - tmp_start))

                self.back_sum_carnum = self.sum_carnum
                self.back_finished_carnum = self.finished_carnum
                self.back_schedule_carList = self.schedule_carList[:]
                self.back_speedList = self.finished_speedList[:]

                tmp_start = time.clock()

                # 无法对所有车进行备份，也没有必要，代价太大
                # for car in self.carList.values():
                for car_id in self.schedule_carList:
                    car = self.carList[car_id]
                    back_car = copy.copy(car)
                    back_car.path = car.path[:]
                    self.back_carList[car.id] = back_car
                    # print(str(back_car.id) + " " + str(back_car.start_time))

                tmp_end = time.clock()
                print('Backup Car time: %s Seconds' % (tmp_end - tmp_start))
                tmp_start = time.clock()

                for road in self.roadList.values():
                    back_road = copy.copy(road)
                    back_road.status = road.status.copy()
                    self.back_roadList[road.road_id] = back_road

                tmp_end = time.clock()
                print('Backup Road time: %s Seconds' % (tmp_end - tmp_start))

            end = time.clock()
            print('Running time: %s Seconds' % (end - start))

            sum_end = time.clock()
            print('Sum time: %s Seconds' % (sum_end - sum_start))


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

    # 正常执行
    start = time.clock()
    carScheduler = CarScheduler()
    carScheduler.work(car_path, road_path, cross_path)

    with open(answer_path, 'w') as f:
        for car in carScheduler.carList.values():
            for id, path_id in enumerate(car.path):
                if path_id > 100000:
                    car.path[id] = path_id - 100000
            f.write("(" + str(car.id) + ", " + str(car.start_time) + ", "
                    + ", ".join([str(a) for a in car.path]) + ")\n")
    end = time.clock()
    print('Final time: %s Seconds' % (end - start))

    # # 测试最优参数
    # for delay_time in range(1, 3):
    #     for weight in range(1, 3):
    #         for carnum in range(1, 5):
    #             start = time.clock()
    #             carScheduler = CarScheduler()
    #             carScheduler.delay_time = delay_time  # 车辆延迟时间，可调参数
    #             carScheduler.weight = weight * 1.0 / 2  # 车况权值，可调参数
    #             carScheduler.max_schedule_carnum = 1000 + carnum * 500  # 最大同时调度车辆数，可调参数
    #             carScheduler.work(car_path, road_path, cross_path)
    #             end = time.clock()
    #
    #             print(
    #                 'delay_time: %2s, weight: %2s, carnum: %5s, Final res: %4s, Final time: %5s Seconds(deadlock time: %3s)'
    #                 % (delay_time, weight * 1.0 / 2, 1000 + carnum * 500, carScheduler.current_time, end - start,
    #                    carScheduler.deadlock_time))


# to read input file
# process
# to write output file
# python CodeCraft-2019.py ../config/car.txt ../config/road.txt ../config/cross.txt ../config/answer.txt

if __name__ == "__main__":
    main()
