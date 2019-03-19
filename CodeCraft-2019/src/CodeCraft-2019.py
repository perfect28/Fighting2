# coding=utf-8
import heapq
import logging
import math
import sys
import time

import numpy as np

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
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
        self.cur_point = start_point  # 下一个路口（即将到达)的id号
        self.cur_edge = -1  # 当前所在道路id
        self.cur_lane = -1  # 当前所在车道id
        self.cur_dir = ''  # 下一个路口的转弯方向, 'D'|'L'|'R'
        # print("car(start->end): " + str(car_id) + ' ' + str(start_point) + ' ' + str(end_point))


class CarScheduler:
    def __init__(self):
        self.carList = {}
        self.roadList = {}
        self.crossList = {}

        self.current_time = 0  # 当前时间片
        self.max_time = 10000  # 最大时间片设置为10000

        self.sum_carnum = 0  # 当前需要调度的车辆数
        self.schedule_carList = []  # 车辆调度完毕则从该列表中移除

        self.scheduled_carnum = 0  # 当前已调度的车辆数
        self.cur_schedule_carList = []  # 当前已调度的车辆列表，为了debug哪些车辆出了问题
        self.last_scheduled_carnum = -1  # 上一轮已调度的车辆数，用来检查死锁

        self.finished_carnum = 0  # 已经到达终点的车辆数

        self.sorted_carList = []
        self.sorted_crossList = []

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
                self.carList[car_id] = Car(car_id, start_point, end_point, speed, start_time)

        self.sorted_crossList = sorted(self.crossList.values(), key=lambda d: d.id)
        self.sorted_carList = sorted(self.carList.values(), key=lambda d: d.speed, reverse=True)

    def calculateshortestpath(self, startNode, endNode):
        if startNode.id == endNode.id:  # 直接返回！！
            return 0
        q = []
        for node in self.crossList.values():
            node.dis = 0x3f3f3f3f
            node.vis = False

        self.crossList[startNode.id].dis = 0
        self.crossList[startNode.id].vis = True
        heapq.heappush(q, startNode)

        while q:
            tmp = heapq.heappop(q)  # 获得此时的cross,即节点信息
            for road_id in tmp.edgeList2:  # 遍历邻接道路,后者表示从此路口出去的边id
                if road_id == -1: continue
                edge = self.roadList[road_id]
                u = edge.u  # ｕ应该和上边的tmp的id一致
                v = edge.v
                if self.crossList[v].vis: continue
                edge_weight = math.ceil(edge.road_len * 1.0 / edge.limit_rate)  # 边的权重为长度除以限速，粗略估计
                status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数
                tempdist = self.crossList[u].dis + edge_weight + 3 * status_weight
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
        # 这里需要慎重考虑一下，并不是到时间就发车，应该根据路况来定，以免发生死堵
        # 1. 对于同一时间同一地点，速度快的先发车
        # 2. 当前路况态度，延缓发车，避免死锁
        for car in self.sorted_carList:
            if car.start_time == self.current_time:
                # 获取该车即将开往的道路id
                # 这里需要改变一下思路，不再是一开始就知道整个PATH然后按图索骥
                # 而是每一个时间片都预估下一个road_id

                # if car.id == 10013:
                #     print("car10013 departure")

                min_len = 0x3f3f3f3f
                # 遍历该出发点的每一个出口，粗略估计每条路的行驶代价
                for edge_id in self.crossList[car.start_point].edgeList2:
                    if edge_id == -1: continue
                    edge = self.roadList[edge_id]

                    startNode = self.crossList[edge.v]
                    endNode = self.crossList[car.end_point]
                    dis = self.calculateshortestpath(startNode, endNode)

                    edge_weight = math.ceil(edge.road_len * 1.0 / edge.limit_rate)  # 边的权重为长度除以限速，粗略估计
                    status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数
                    dis += edge_weight + 3 * status_weight

                    if dis < min_len:
                        min_len = dis
                        road_id = edge_id

                cur_road = self.roadList[road_id]
                num_lane = cur_road.num_lane
                road_len = cur_road.road_len

                if cur_road.car_num > num_lane * road_len / 7:
                    car.start_time += 1
                    continue

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
                        # 车辆
                        car.status = 1
                        car.path.append(cur_road.road_id)
                        car.cur_lane = i
                        car.cur_edge = cur_road.road_id
                        car.cur_point = cur_road.v
                        break

                if car.cur_edge == -1:  # 表示此次没有出库，出发时间后移1
                    car.start_time += 1

    '''
    调度某一车道车辆到终止位置
    '''

    def driveAllCarJustOnRoadToEndState(self, road_id, channel_id, st):
        # st若为-1,则需判断第一辆车是否过路口
        # 若不为-1，则后面的车都能完成调度
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

                # if car.id == 10163:
                #     print("debug car 10163")

                if car.status == 0:  # 状态为等待行驶
                    s = min(car.speed, road.limit_rate)
                    if not flag and s > i:  # 可行驶距离超过剩余到路口的距离!!(一定是路口，其他则是能走多少多少）
                        break  # 剩下的车都不改变状态

                    if s >= i - tmp_pos:  # 这里一定要加等于！！，否则之前的车就被覆盖了
                        status[tmp_pos + 1, channel_id] = status[i, channel_id]  # 紧贴上一辆车后面！！
                        if tmp_pos + 1 != i:  # !！避免车没有动的情况
                            # 道路
                            status[i, channel_id] = 0
                        tmp_pos = tmp_pos + 1
                    else:
                        status[i - s, channel_id] = status[i, channel_id]
                        tmp_pos = i - s
                        # 道路
                        status[i, channel_id] = 0

                    # 统计量
                    self.scheduled_carnum += 1
                    self.cur_schedule_carList.remove(car.id)

                    # 车辆
                    car.status = 1  # 车状态变为终止，其他不变
                    flag = True
                else:  # 状态为终止状态
                    flag = True
                    tmp_pos = i  # !!一定要更新这里!!
                    continue

    def get_direction(self, car):
        if car.cur_edge == -1:
            car.cur_dir = 'D'
            return

        min_len = 0x3f3f3f3f

        # 遍历该出发点的每一个出口，粗略估计每条路的行驶代价
        for edge_id in self.crossList[car.cur_point].edgeList2:
            if edge_id == -1: continue
            if math.fabs(edge_id - car.cur_edge) == 100000: continue  # 去掉来时的方向
            edge = self.roadList[edge_id]

            startNode = self.crossList[edge.v]
            endNode = self.crossList[car.end_point]
            dis = self.calculateshortestpath(startNode, endNode)

            edge_weight = math.ceil(edge.road_len * 1.0 / edge.limit_rate)  # 边的权重为长度除以限速，粗略估计
            status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数
            dis += edge_weight + 3 * status_weight

            if dis < min_len:
                min_len = dis
                road_id = edge_id

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

    def work(self, car_path, road_path, cross_path):
        self.initFile(car_path, road_path, cross_path)

        self.current_time = 0

        while self.current_time < self.max_time and self.finished_carnum < len(self.carList):
            self.current_time += 1  # 系统时间+1
            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!current time : " + str(self.current_time))
            # if self.current_time == 11:
            #     print("start debug")

            self.scheduled_carnum = 0  # 已调度车辆数置零

            self.cur_schedule_carList = []
            # for car in self.carList.values():
            for car_id in self.schedule_carList:
                car = self.carList[car_id]
                self.cur_schedule_carList.append(car_id)
                car.status = 0  # 车辆状态置为等待行驶
                self.get_direction(car)  # 获取车辆方向
                # print("car" + str(car.id) + ": " + car.cur_dir + "\t" + str(car.path))

            for road in self.roadList.values():
                '''
                调整所有道路上在道路上的车辆，让道路上车辆前进，
                只要不出路口且可以到达终止状态的车辆
                '''
                for channle_id in range(road.num_lane):
                    self.driveAllCarJustOnRoadToEndState(road.road_id, channle_id, -1)

            for road in self.roadList.values():  # 预处理每条道路的x,y初始坐标
                if road.car_num == 0:
                    road.x = road.road_len
                    continue
                road.x = 0
                road.y = 0
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

            self.last_scheduled_carnum = -1
            while self.scheduled_carnum < self.sum_carnum:
                # has_finined_car = False
                # if self.last_scheduled_carnum != -1:
                #     if not has_finined_car and self.last_scheduled_carnum - self.scheduled_carnum == 0:
                #         print("deadlock!!!!")
                #         print(self.cur_schedule_carList)
                # self.last_scheduled_carnum = self.scheduled_carnum
                # print("current scheduled_carnum: " + str(self.scheduled_carnum))
                # print("current sum_carnum: " + str(self.sum_carnum))
                # driveAllWaitCar()
                for cross in self.sorted_crossList:
                    for road_id in sorted(cross.edgeList1):  # 按id顺序遍历该路口的邻接边(以此路口为出口的)
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
                            car = self.carList[car_id]

                            # if car.id == 10163:
                            #     print("debug car 10163")

                            # 终止条件： 车的cur_point即end_point，此时不用再考虑冲突
                            if car.cur_point == car.end_point:
                                has_finined_car = True
                                # 统计量
                                self.cur_schedule_carList.remove(car.id)
                                self.sum_carnum -= 1  # 调度总车辆-1
                                self.schedule_carList.remove(car.id)
                                self.finished_carnum += 1
                                # 道路
                                road = self.roadList[car.cur_edge]
                                road.status[road.x][road.y] = 0  # 清理车辆
                                road.car_num -= 1  # 该道路车辆-1

                                self.driveAllCarJustOnRoadToEndState(car.cur_edge, car.cur_lane, road.x)
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
                            elif car.cur_dir == 'R':  # 右转
                                cur_road_id = cross.edgeList2[(idx + 3) % 4]

                            cur_road = self.roadList[cur_road_id]
                            num_lane = cur_road.num_lane
                            road_len = cur_road.road_len
                            is_traffic_jam = True
                            for i in range(num_lane):  # 遍历车道
                                if cur_road.status[road_len - 1, i] == 0:
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
                                        self.cur_schedule_carList.remove(car.id)
                                        # 道路 road.car_num 不变
                                        road.status[road.x][road.y] = 0
                                        road.status[0][road.y] = car.id
                                        # 车辆
                                        car.status = 1
                                        self.driveAllCarJustOnRoadToEndState(car.cur_edge, car.cur_lane, road.x)
                                    else:
                                        for j in range(road_len - 1, road_len - s - 1, -1):  # 尝试着开更远
                                            if cur_road.status[j, i] != 0:  # 前方有车辆
                                                j += 1
                                                break
                                        # 统计量
                                        self.scheduled_carnum += 1  # 调度车辆+1
                                        self.cur_schedule_carList.remove(car.id)
                                        # 道路
                                        cur_road.status[j, i] = car.id  # 调度车辆
                                        cur_road.car_num += 1  # 该道路车辆＋1
                                        road.status[road.x][road.y] = 0
                                        road.car_num -= 1

                                        # 车辆
                                        last_lane = car.cur_lane

                                        car.path.append(cur_road.road_id)
                                        car.status = 1
                                        car.cur_point = cur_road.v
                                        car.cur_edge = cur_road.road_id
                                        car.cur_lane = i

                                        # 信息全部更新之后才可以调度该条车道
                                        self.driveAllCarJustOnRoadToEndState(road.road_id, last_lane, road.x)

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
                                    break  # 这里跳出，是已经找到可以行驶的车道了，不用遍历后面的车道
                                else:
                                    front_car = self.carList[cur_road.status[road_len - 1, i]]
                                    if front_car.status == 0:  # 未调度
                                        is_traffic_jam = False

                            # ！！一定要处理路口堵死的情况
                            if car.status == 0 and is_traffic_jam:  # 表示这里没有冲突，但是前进的道路上堵着了
                                # 路口堵死，调度到路口最前处, 处理和s=0一样

                                # 统计量
                                self.scheduled_carnum += 1  # 调度车辆+1
                                self.cur_schedule_carList.remove(car.id)
                                # 道路 road.car_num 不变
                                road.status[road.x][road.y] = 0
                                road.status[0][road.y] = car.id

                                # 车辆
                                car.status = 1
                                self.driveAllCarJustOnRoadToEndState(car.cur_edge, car.cur_lane, road.x)
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

                            break  # 这里跳出，是跳出当前的道路循环

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

    start = time.clock()

    carScheduler = CarScheduler()

    carScheduler.work(car_path, road_path, cross_path)

    end = time.clock()
    print('Running time: %s Seconds' % (end - start))

    with open(answer_path, 'w') as f:
        for car in carScheduler.carList.values():
            for id, path_id in enumerate(car.path):
                if path_id > 100000:
                    car.path[id] = path_id - 100000
            f.write("(" + str(car.id) + ", " + str(car.start_time) + ", "
                    + ", ".join([str(a) for a in car.path]) + ")\n")


# to read input file
# process
# to write output file
# python CodeCraft-2019.py ../config/car.txt ../config/road.txt ../config/cross.txt ../config/answer.txt

if __name__ == "__main__":
    main()
