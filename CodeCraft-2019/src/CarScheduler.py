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
from Entity import *


class CarScheduler:
    def __init__(self):
        self.carList = {}
        self.roadList = {}
        self.crossList = {}

        self.sorted_crossList = []
        self.finished_speedList = [0] * 30
        self.depatureList = [[] for i in range(10000)]  # key值为时间
        self.priorityList = {}  # 优先车辆发车队列，不按时间划分了
        self.modifyList = []

        self.current_time = 0  # 当前时间片
        self.max_time = 10000  # 最大时间片设置为10000
        self.deadlock_time = 0  # 死锁次数

        # 可调参数
        self.delay_time = 1  # 车辆延迟时间，可调参数
        self.weight = 3  # 车况权值，可调参数
        self.max_schedule_carnum = 3000  # 最大同时调度车辆数，可调参数

        self.sum_carnum = 0  # 当前需要调度的车辆数
        self.schedule_carList = []  # sum_carnum对应的列表
        self.finished_carnum = 0  # 已经到达终点的车辆数
        self.finished_pricar = 0  # 已经到达终点的优先车辆数

        self.scheduled_carnum = 0  # 当前已调度的车辆数
        self.cur_schedule_carList = []  # 当前未调度的车辆列表，为了debug哪些车辆出了问题
        self.last_scheduled_carnum = -1  # 上一轮已调度的车辆数，用来检查死锁
        self.deadlock_flag = False

        # 解决死锁，需要回滚
        self.back_car_dir = {}  # 上一轮车的方向信息
        self.back_carList = {}  # 上一轮车辆信息
        self.back_roadList = {}  # 上一轮道路信息
        self.back_priorityList = {}  # 上一轮的优先车辆发车列表
        self.back_speedList = []
        self.back_sum_carnum = 0
        self.back_finished_carnum = 0
        self.back_schedule_carList = []

    def initFile(self, car_path, road_path, cross_path, preset_answer_path):
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

        with open(car_path, 'r', encoding='UTF-8') as file:
            lines = file.read().split('\n')  # store the lines in a list
            lines = [x for x in lines if len(x) > 0]  # get read of the empty lines
            lines = [x for x in lines if x[0] != '#']  # get rid of comments
            lines = [x.lstrip('(').rstrip(')') for x in lines]  # get rid of fringe whitespaces
            for line in lines:
                data = list(map(int, line.split(',')))
                car_id, start_point, end_point, speed, start_time, is_priority, is_preset = data
                self.carList[car_id] = Car(car_id, start_point, end_point, speed, start_time, is_priority, is_preset)

                if not is_preset and not is_priority and speed >= 8:
                    start_time += 400

                if not is_preset and is_priority and speed >= 8:
                    start_time += 200

                self.carList[car_id].start_time = start_time

                if is_priority:
                    if start_point in self.priorityList.keys():
                        self.priorityList[start_point].append(car_id)
                    else:
                        self.priorityList[start_point] = [car_id]
                else:
                    self.depatureList[start_time].append(car_id)

            speedList = [0 for i in range(30)]  # key值为时间

            preset_carnum = 0
            priority_carnum = 0
            for car in self.carList.values():
                speedList[car.speed] += 1
                if car.is_preset == 1:
                    preset_carnum += 1
                if car.is_priority:
                    priority_carnum += 1

            print("preset carnum : %d, preset carnum : %d" % (preset_carnum, priority_carnum))
            for speed, num in enumerate(speedList):
                if num > 0:
                    print("speed " + str(speed) + ": " + str(num))

        with open(preset_answer_path, 'r', encoding='UTF-8') as file:
            lines = file.read().split('\n')  # store the lines in a list
            lines = [x for x in lines if len(x) > 0]  # get read of the empty lines
            lines = [x for x in lines if x[0] != '#']  # get rid of comments
            lines = [x.lstrip('(').rstrip(')') for x in lines]  # get rid of fringe whitespaces
            for line in lines:
                data = list(map(int, line.split(',')))
                car_id, start_time = data[:2]
                path = []
                for road_id in data[2:]:
                    path.append(road_id)
                car = self.carList[car_id]
                car.start_time = start_time
                car.path = path
                car.path_pos = 0
        self.sorted_crossList = sorted(self.crossList.values(), key=lambda d: d.id)

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
                status_weight = math.floor(edge.car_num * 1.0 / edge.num_lane / edge.road_len)  # 路况权重
                # speed_weight = 0
                # speed_weight = abs(car_speed - edge.mean_speed)
                # speed_weight *= speed_weight
                tempdist = self.crossList[u].dis + edge_weight + self.weight * status_weight
                if tempdist < self.crossList[v].dis:
                    self.crossList[v].dis = tempdist
                    self.crossList[v].vis = True
                    self.crossList[v].pre = edge
                    if endNode.id == v:  # 到达终点
                        return tempdist
                    heapq.heappush(q, self.crossList[v])

    def updateDepartCar(self, car, road, j, i):  # 车辆，道路，行，列
        # 统计量
        self.sum_carnum += 1  # 调度总车辆+1
        self.schedule_carList.append(car.id)
        # 道路
        road.status[j, i] = car.id  # 调度车辆
        road.car_num += 1  # 该道路车辆＋1
        road.mean_speed = (road.mean_speed * (road.car_num - 1) + car.speed) * 1.0 / road.car_num
        # 车辆
        car.status = 1
        if not car.is_preset:
            car.path.append(road.road_id)
        else:
            car.path_pos += 1
        car.cur_lane = i
        car.cur_pos = j
        car.cur_point = road.v
        car.last_edge = car.cur_edge
        car.cur_edge = road.road_id

    def RunToRoad(self, road, s):  # 发车
        road_len = road.road_len
        for i in range(road.num_lane):  # 遍历车道
            j = road_len
            for j in range(road_len - 1, road_len - s - 1, -1):  # 尝试着开更远
                if road.status[j, i] != 0:  #
                    front_car = self.carList[road.status[j, i]]  # 前车
                    if front_car.status == 0:
                        return False, -1, -1  # 前车未调度，无法发车
                    else:
                        j += 1  # 前车已停，发车到上一格
                        break
            if j == road_len: continue  # 如果是道路最后一格，检查下一条车道
            return True, j, i  # 成功发车, 返回目的地的坐标
        return False, -1, -1  # 道路堵死，无法发车

        # 优先级车辆尝试发车

    def runPriorityCar(self, cross_id):
        if cross_id in self.priorityList.keys():  # 遍历所有当前时刻有优先车辆的交口
            for car_id in self.priorityList[cross_id][:]:
                car = self.carList[car_id]
                if car.start_time > self.current_time: continue
                if car.speed > 4:
                    if car.is_preset == 0 and self.sum_carnum >= self.max_schedule_carnum * 2:
                        car.start_time += self.delay_time
                        continue
                road = self.roadList[car.next_edge]
                s = min(car.speed, road.limit_rate)  # 可以行驶的距离
                is_depature, j, i = self.RunToRoad(road, s)
                if is_depature:
                    self.priorityList[cross_id].remove(car_id)
                    self.updateDepartCar(car, road, j, i)
                    self.scheduled_carnum += 1  # 此时已经调度完毕

    # 车辆发车(每个时间片的最后发车)
    def driveCarInGarage(self):
        delay_time = self.delay_time
        # 先发车当前时间未调度的优先车辆
        for cross_id in self.crossList:
            if cross_id in self.priorityList.keys():  # 遍历所有当前时刻有优先车辆的交口
                for car_id in self.priorityList[cross_id][:]:
                    car = self.carList[car_id]
                    if car.start_time > self.current_time: continue
                    if car.speed > 4:
                        if car.is_preset == 0 and self.sum_carnum >= self.max_schedule_carnum * 2:
                            car.start_time += delay_time
                            continue
                    road = self.roadList[car.next_edge]  # 优先级车辆都找过方向了
                    s = min(car.speed, road.limit_rate)  # 可以行驶的距离
                    is_depature, j, i = self.RunToRoad(road, s)
                    if not is_depature:  # 表示此次没有出库，出发时间后移1
                        car.start_time += delay_time
                    else:
                        self.updateDepartCar(car, road, j, i)
                        self.priorityList[cross_id].remove(car.id)

        for car_id in sorted(self.depatureList[self.current_time]):  # 按id号顺序发车!!
            car = self.carList[car_id]  # 重新获取该车的引用
            if car.speed > 2:
                if car.is_preset == 0 and self.sum_carnum >= self.max_schedule_carnum:
                    car.start_time += delay_time
                    self.depatureList[self.current_time + delay_time].append(car.id)
                    continue
            startNode = self.crossList[car.start_point]
            endNode = self.crossList[car.end_point]
            road_id = self.get_next_road(car, startNode, endNode)
            road = self.roadList[road_id]
            if car.speed > 2:
                if car.is_preset == 0 and road.car_num > road.num_lane * road.road_len * 4 / 5:
                    car.start_time += delay_time
                    self.depatureList[self.current_time + delay_time].append(car.id)
                    continue
            s = min(car.speed, road.limit_rate)  # 可以行驶的距离
            is_depature, j, i = self.RunToRoad(road, s)
            if not is_depature:  # 表示此次没有出库，出发时间后移1
                car.start_time += delay_time
                self.depatureList[self.current_time + delay_time].append(car.id)
            else:
                self.updateDepartCar(car, road, j, i)

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
                # if car.id == 69517:
                #     print("debug")
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
                            status[i, channel_id] = 0
                            tmp_pos = i - s
                        # 统计量
                        self.scheduled_carnum += 1
                        self.cur_schedule_carList.remove(car.id)
                        # 车辆
                        car.status = 1  # 车状态变为终止，其他不变
                        car.last_edge = car.cur_edge
                    else:
                        if s >= i - tmp_pos:
                            tmp_pos = i
                            if st != -1: return  # 需要再考虑
                            continue
                        else:
                            status[i - s, channel_id] = status[i, channel_id]
                            car.cur_pos = i - s
                            status[i, channel_id] = 0
                            tmp_pos = i - s
                            # 统计量
                            self.scheduled_carnum += 1
                            self.cur_schedule_carList.remove(car.id)
                            # 车辆
                            car.status = 1  # 车状态变为终止，其他不变
                            car.last_edge = car.cur_edge
                            flag = True  # 标记前方有标记为终态的车
                else:  # 状态为终止状态
                    flag = True  # 标记前方有标记为终态的车
                    tmp_pos = i  # !!一定要更新这里!!

    def road_to_dir(self, car, road_id):
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

    def get_next_road(self, car, startNode, endNode):
        if car.next_edge != -1 and car.next_edge != car.cur_edge:  # 求过方向了就不用求了
            return car.next_edge
        road_id = -1
        if car.is_preset:  # 如果是预置车辆
            road_id = car.path[car.path_pos]
            road = self.roadList[road_id]
            if car.cur_point != road.u:
                road_id = road_id + 100000
        else:
            dis = self.calculateshortestpath(car.cur_edge, car.speed, startNode, endNode)
            # print(str(startNode.id) + "->" + str(endNode.id))
            nowNode = endNode
            while nowNode.pre != -1:
                car.next_next_edge = road_id
                road_id = nowNode.pre.road_id
                nowNode = self.crossList[nowNode.pre.u]
        car.next_edge = road_id
        return road_id

    def get_direction(self, car):
        # if car.id == 24371:
        #     print("debug")
        # print(str(car.id) + " get direction")
        if car.cur_point == car.end_point:  # 到达终点，方向定为直行
            car.cur_dir = 'D'
            return
        if car.cur_dir != '' and car.last_edge == car.cur_edge:  # 不用改变方向
            return
        if car.next_next_edge != -1:
            car.next_edge = car.next_next_edge
            car.next_next_edge = -1
            self.road_to_dir(car, car.next_edge)
            return
        startNode = self.crossList[car.cur_point]
        endNode = self.crossList[car.end_point]
        road_id = self.get_next_road(car, startNode, endNode)
        self.road_to_dir(car, road_id)

    def dealwith_deadlock(self):
        # 更新方向
        for i in self.cur_schedule_carList:  # 不包括刚刚发车的优先车辆
            tmp_car = self.carList[i]
            # print(tmp_car.cur_dir + " " + str(tmp_car.cur_edge) + " " + str(tmp_car.cur_point))
            tmp_road = self.roadList[tmp_car.cur_edge]
            if tmp_car.is_preset == 0 and tmp_road.scheduleList[0] == tmp_car.id:  # 此车是路口第一优先级
                if tmp_car.id in self.modifyList: continue
                print(str(tmp_car.id) + " " + tmp_car.cur_dir + " "
                      + str(tmp_car.cur_edge) + " " + str(tmp_car.cur_point))
                idx1 = self.crossList[tmp_car.cur_point].edgeList1.index(
                    tmp_car.cur_edge)
                if tmp_car.cur_dir == 'R' or tmp_car.cur_dir == 'L':
                    idx2 = (idx1 + 2) % 4
                    towards_road_id = self.crossList[tmp_car.cur_point].edgeList2[idx2]
                    if towards_road_id != -1:
                        self.modifyList.append(tmp_car.id)
                        print("modify " + str(tmp_car.id))
                        tmp_car.cur_dir = 'D'
                        tmp_car.sure_dir = ''
                        break
                elif tmp_car.cur_dir == 'D':
                    idx2 = (idx1 + 1) % 4
                    left_road_id = self.crossList[tmp_car.cur_point].edgeList2[idx2]
                    if left_road_id != -1:
                        self.modifyList.append(tmp_car.id)
                        print("modify " + str(tmp_car.id))
                        tmp_car.cur_dir = 'L'
                        tmp_car.sure_dir = ''
                        break
                    idx2 = (idx1 + 3) % 4
                    right_road_id = self.crossList[tmp_car.cur_point].edgeList2[idx2]
                    if right_road_id != -1:
                        self.modifyList.append(tmp_car.id)
                        print("modify " + str(tmp_car.id))
                        tmp_car.cur_dir = 'R'
                        tmp_car.sure_dir = ''
                        break

        # 发生回滚时，本轮已有部分优先级车辆出库
        self.schedule_carList = self.back_schedule_carList[:]
        for car_id in self.schedule_carList:  # 还原调度车辆
            car = self.carList[car_id]
            if car.cur_dir == '':
                self.back_car_dir[car_id] = car.sure_dir  # 本轮已经调度过的车
            else:
                self.back_car_dir[car_id] = car.cur_dir

        for car_id in self.back_priorityList:
            if car_id not in self.priorityList.keys():  # 如果优先车辆不在列表中，说明已经被调度
                car.status = 0
                if not car.is_preset:
                    car.path.pop()
                else:
                    car.path_pos -= 1
                car.cur_lane = -1
                car.cur_pos = -1
                car.cur_point = car.start_point
                car.last_edge = -1
                car.cur_edge = -1

        # 还原已经出库的优先车辆
        self.priorityList = copy.deepcopy(self.back_priorityList)

        for car in self.back_carList.values():
            back_car = copy.copy(car)
            back_car.path = car.path[:]
            self.carList[car.id] = back_car
            # print(str(back_car.id) + " " + str(back_car.start_time))

        for road in self.back_roadList.values():
            back_road = copy.copy(road)
            back_road.status = road.status.copy()
            self.roadList[road.road_id] = back_road

        self.sum_carnum = self.back_sum_carnum
        self.finished_carnum = self.back_finished_carnum
        self.current_time -= 1
        self.finished_speedList = self.back_speedList[:]

    def initCarDir(self):
        # 初始化当前时间片待发车优先车辆的方向(获取下一条边)
        for carList in self.priorityList.values():  # 遍历各个路口
            for car_id in carList:
                car = self.carList[car_id]
                if car.start_time > self.current_time: continue
                startNode = self.crossList[car.start_point]
                endNode = self.crossList[car.end_point]
                self.get_next_road(car, startNode, endNode)

        self.scheduled_carnum = 0  # 已调度车辆数置零
        self.cur_schedule_carList = []
        for car_id in self.schedule_carList:  # 只管调度中的车辆
            car = self.carList[car_id]
            # 状态还原
            car.status = 0  # 车辆状态置为等待行驶
            car.sure_dir = ''  # 每轮重置
            self.cur_schedule_carList.append(car_id)
            if self.deadlock_flag:  # 若上一轮发生死锁，不用重新找方向
                if car_id in self.back_car_dir.keys():
                    car.cur_dir = self.back_car_dir[car_id]
                    car.next_next_edge = -1  # 有的车被强制改变方向！！
                else:
                    print("wrong!!!!!!")
            else:
                # 只要能到路口的车求方向，其他车方向不变
                if car.cur_pos < min(car.speed, self.roadList[car.cur_edge].limit_rate):
                    self.get_direction(car)  # 获取车辆方向
            # print("car" + str(car.id) + ": " + car.cur_dir + "\t" + str(car.path))

    def createCarSequence(self):
        for road in self.roadList.values():
            if road.car_num == 0: continue
            # print("road " + str(road.road_id) + " createSequence")
            # if road.road_id == 106732:
            #     print("debug")
            x = [0] * road.num_lane
            while True:
                tmp_car = -1
                for i in range(road.num_lane):  # 遍历车道
                    if x[i] >= road.road_len: continue
                    car_id = road.status[x[i]][i]
                    while car_id == 0 or car_id != 0 and self.carList[car_id].status == 1:  # 忽略已经调度的车
                        x[i] += 1
                        if x[i] >= road.road_len:
                            break
                        car_id = road.status[x[i]][i]
                    if x[i] >= road.road_len: continue  # 说明当前车道没有车辆了
                    car = self.carList[car_id]
                    if car.is_priority == 1:
                        if tmp_car == -1:
                            tmp_car = car
                        elif tmp_car.cur_pos > car.cur_pos:
                            tmp_car = car
                if tmp_car == -1:
                    break
                else:
                    road.scheduleList.append(tmp_car.id)
                    x[tmp_car.cur_lane] += 1

            for i in range(road.road_len):  # 遍历每一行
                for j in range(road.num_lane):  # 遍历车道
                    car_id = road.status[i][j]
                    if car_id != 0 and self.carList[car_id].status == 0 and car_id not in road.scheduleList:
                        road.scheduleList.append(car_id)
                        k = i + 1
                        while k < road.road_len:  # 检查其后车辆是否属于优先级车辆
                            tmp_id = road.status[k][j]
                            k += 1
                            if tmp_id == 0: continue
                            tmp_car = self.carList[tmp_id]
                            if tmp_car.status == 1: continue
                            if tmp_car.is_priority == 1:
                                road.scheduleList.append(tmp_id)
                            else:
                                break

    def maintainCarSequence(self, road):
        if len(road.scheduleList) > 0:
            car_id = road.scheduleList[0]
            car = self.carList[car_id]
            while car.status == 1:  # 如果已经调度，则取下一车辆
                road.scheduleList.pop(0)
                if len(road.scheduleList) == 0: break
                car_id = road.scheduleList[0]
                car = self.carList[car_id]

    def work(self, car_path, road_path, cross_path, preset_answer_path):
        self.initFile(car_path, road_path, cross_path, preset_answer_path)
        self.current_time = 0
        self.deadlock_flag = False

        def cmp(u, v):
            if u > 100000: u -= 100000
            if v > 100000: v -= 100000
            if u < v:
                return -1
            else:
                return 1

        sum_start = time.clock()
        while self.current_time < self.max_time and self.finished_carnum < len(self.carList):
            start = time.clock()
            self.current_time += 1  # 系统时间+1
            print(
                "\r--current time-- : %d --current schedule-- : %d --current finish-- : %d --finished pricar-- : %d" % (
                    self.current_time, self.sum_carnum, self.finished_carnum, self.finished_pricar), end='')

            # if self.current_time == 2:
            #     print("debug")
            # 预处理：获取当前需要调度的车辆，以及当前时刻可以发车的优先车辆的前进方向（实时调度）
            tmp_start = time.clock()
            self.initCarDir()
            tmp_end = time.clock()
            # print('Preprocess Car time: %s Seconds' % (tmp_end - tmp_start))

            # 第一步，道路内车辆的标定与驱动
            tmp_start = time.clock()
            for road in self.roadList.values():
                for channle_id in range(road.num_lane):
                    self.driveAllCarJustOnRoadToEndState(road.road_id, channle_id, -1)
            tmp_end = time.clock()
            # print('Preprocess Road time: %s Seconds' % (tmp_end - tmp_start))

            # 第二步，优先车辆上路，按路口遍历
            for cross_id in self.crossList:
                self.runPriorityCar(cross_id)

            # 第三步，创建道路优先级队列
            self.createCarSequence()

            # 第四步，调度所有等待车辆
            tmp_start = time.clock()
            self.last_scheduled_carnum = -1
            self.has_finined_car = False
            self.deadlock_flag = False  # 还原死锁标志
            while self.scheduled_carnum < self.sum_carnum:
                if self.last_scheduled_carnum != -1:
                    if not self.has_finined_car and self.last_scheduled_carnum == self.scheduled_carnum:
                        # while True:
                        #     print("deadlock!!!!")
                        #     time.sleep(10)
                        print("deadlock!!!!")
                        # self.deadlock_time += 1
                        self.deadlock_flag = True
                        # self.dealwith_deadlock()
                        break
                self.has_finined_car = False
                self.last_scheduled_carnum = self.scheduled_carnum
                # print("current scheduled_carnum: " + str(self.scheduled_carnum))
                # print("current sum_carnum: " + str(self.sum_carnum))
                for cross in self.sorted_crossList:
                    for road_id in sorted(cross.edgeList1,
                                          key=functools.cmp_to_key(cmp)):  # 按id顺序遍历
                        # print(str(cross.id) + ': ' + str(road_id))
                        if road_id == -1: continue
                        road = self.roadList[road_id]
                        while len(road.scheduleList) > 0:  # 循环执行，若能调度，尽可能多的调度
                            # 1. 获取调度车辆
                            car_id = road.scheduleList[0]
                            car = self.carList[car_id]
                            # 2. 判断是否有冲突
                            if self.conflict(car, cross): break  # 若冲突，此道路直接跳过！！
                            # 3. 若可以调度，对当前车道进行调度
                            road_id = car.cur_edge
                            channel_id = car.cur_lane
                            st = car.cur_pos
                            # print("move car " + str(car.id))
                            # if car.id == 24371:
                            #     print("debug")
                            if self.moveToNextRoad(car):
                                self.driveAllCarJustOnRoadToEndState(road_id, channel_id, st)
                                self.maintainCarSequence(road)  # 对于该道路的优先级进行维护
                                # 优先车辆上路
                                # for cross_id in self.crossList:
                                #     self.runPriorityCar(cross_id)
                                # 只影响两个路口
                                self.runPriorityCar(cross.id)
                                self.runPriorityCar(self.crossList[self.roadList[road_id].u])
                            else:
                                break
            tmp_end = time.clock()
            # print('Scheduling time: %s Seconds' % (tmp_end - tmp_start))
            # 第五步，发车
            if not self.deadlock_flag:  # 若未发生死锁，存储上一轮的所有状态信息
                # 车库中的车辆上路行驶
                tmp_start = time.clock()
                self.driveCarInGarage()
                tmp_end = time.clock()
                # print('Depart time: %s Seconds' % (tmp_end - tmp_start))
                # 保存当前状态
                # self.save()
            else:
                break
            end = time.clock()
            # print('Running time: %s Seconds' % (end - start))
            sum_end = time.clock()
            # print('Sum time: %s Seconds' % (sum_end - sum_start))

    def save(self):
        self.back_sum_carnum = self.sum_carnum
        self.back_finished_carnum = self.finished_carnum
        self.back_schedule_carList = self.schedule_carList[:]
        self.back_speedList = self.finished_speedList[:]

        # 备份已经出库的优先车辆
        self.back_priorityList = copy.deepcopy(self.priorityList)

        tmp_start = time.clock()
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

    def conflict(self, car, cross):
        idx = cross.edgeList1.index(car.cur_edge)  # 表示当前的road_id
        if car.cur_dir == 'L':
            # 考虑右边直行的车
            road_onright_id = cross.edgeList1[(idx + 3) % 4]
            if road_onright_id != -1:
                road_onright = self.roadList[road_onright_id]
                if len(road_onright.scheduleList) > 0:
                    car3_id = road_onright.scheduleList[0]
                    car3 = self.carList[car3_id]
                    if car.is_priority <= car3.is_priority:  # 优先级高就不考虑了
                        if car3.cur_dir == 'D':
                            return True
            # 考虑前边右转的车(非优先级车辆才考虑)
            if car.is_priority == 0:
                road_towards_id = cross.edgeList1[(idx + 2) % 4]
                if road_towards_id != -1:
                    road_towards = self.roadList[road_towards_id]
                    if len(road_towards.scheduleList) > 0:
                        car2_id = road_towards.scheduleList[0]
                        car2 = self.carList[car2_id]
                        if car2.is_priority == 1 and car2.cur_dir == 'R':  # 必须是优先级且右转
                            return True
        elif car.cur_dir == 'R':
            # 考虑左边直行的车
            road_onleft_id = cross.edgeList1[(idx + 1) % 4]
            if road_onleft_id != -1:
                road_onleft = self.roadList[road_onleft_id]
                if len(road_onleft.scheduleList) > 0:
                    car1_id = road_onleft.scheduleList[0]
                    car1 = self.carList[car1_id]
                    if car.is_priority <= car1.is_priority:  # 优先级高就不考虑了
                        if car1.cur_dir == 'D':
                            return True
            # 考虑对面左转的车
            road_towards_id = cross.edgeList1[(idx + 2) % 4]
            if road_towards_id != -1:
                road_towards = self.roadList[road_towards_id]
                if len(road_towards.scheduleList) > 0:
                    car2_id = road_towards.scheduleList[0]
                    car2 = self.carList[car2_id]
                    if car.is_priority <= car2.is_priority:  # 优先级高就不考虑了
                        if car2.cur_dir == 'L':
                            return True
        elif car.cur_dir == 'D':  # (非优先级车辆才考虑)
            if car.is_priority == 0:
                # 考虑左边左转的车
                road_onleft_id = cross.edgeList1[(idx + 1) % 4]
                if road_onleft_id != -1:
                    road_onleft = self.roadList[road_onleft_id]
                    if len(road_onleft.scheduleList) > 0:
                        car1_id = road_onleft.scheduleList[0]
                        car1 = self.carList[car1_id]
                        if car1.is_priority == 1 and car1.cur_dir == 'L':
                            return True
                # 考虑右边右转的车
                road_onright_id = cross.edgeList1[(idx + 3) % 4]
                if road_onright_id != -1:
                    road_onright = self.roadList[road_onright_id]
                    if len(road_onright.scheduleList) > 0:
                        car3_id = road_onright.scheduleList[0]
                        car3 = self.carList[car3_id]
                        if car3.is_priority == 1 and car3.cur_dir == 'R':
                            return True
        return False

    # 车辆到达终点
    def updateCarToEnd(self, car):
        # 统计量
        self.has_finined_car = True
        self.sum_carnum -= 1  # 调度总车辆-1
        self.schedule_carList.remove(car.id)
        self.cur_schedule_carList.remove(car.id)
        self.finished_carnum += 1
        self.finished_speedList[car.speed] += 1
        if car.is_priority:
            self.finished_pricar += 1
        # 道路
        road = self.roadList[car.cur_edge]
        road.status[car.cur_pos][car.cur_lane] = 0  # 清理车辆
        road.car_num -= 1  # 该道路车辆-1
        if road.car_num == 0:
            road.mean_speed = road.limit_rate
        else:
            road.mean_speed = (road.mean_speed * (road.car_num + 1) - car.speed) * 1.0 / road.car_num
        # 车辆
        car.status = 1
        car.end_time = self.current_time

    # 下一条道路可进入，且没有阻挡
    def updateCarToNextRoad(self, car, road, cur_road, j, i):  # road=>cur_road(j,i)
        # 统计量
        self.scheduled_carnum += 1  # 调度车辆+1
        self.cur_schedule_carList.remove(car.id)
        # 道路
        cur_road.status[j, i] = car.id  # 调度车辆
        cur_road.car_num += 1  # 该道路车辆＋1
        cur_road.mean_speed = (cur_road.mean_speed * (
                cur_road.car_num - 1) + car.speed) * 1.0 / cur_road.car_num
        road.status[car.cur_pos][car.cur_lane] = 0  # 清理车辆
        road.car_num -= 1
        if road.car_num == 0:
            road.mean_speed = road.limit_rate
        else:
            road.mean_speed = (road.mean_speed * (
                    road.car_num + 1) - car.speed) * 1.0 / road.car_num
        # 车辆
        if not car.is_preset:
            car.path.append(cur_road.road_id)
        else:
            car.path_pos += 1
        car.status = 1
        car.cur_point = cur_road.v
        car.cur_lane = i
        car.cur_pos = j
        car.last_edge = car.cur_edge
        car.cur_edge = cur_road.road_id
        car.sure_dir = car.cur_dir
        car.cur_dir = ''

    # 下一条道路满，或下一条道路行驶距离为0
    def updateCarToTopFront(self, car):
        # 统计量
        self.scheduled_carnum += 1  # 调度车辆+1
        self.cur_schedule_carList.remove(car.id)
        # 道路 road.car_num 不变
        road = self.roadList[car.cur_edge]
        road.status[car.cur_pos][car.cur_lane] = 0  # 清理车辆
        road.status[0][car.cur_lane] = car.id
        # 车辆
        car.status = 1
        car.cur_pos = 0
        car.last_edge = car.cur_edge

    def moveToNextRoad(self, car):
        # 终止条件： 车的cur_point即end_point，此时不用再考虑冲突
        if car.cur_point == car.end_point:
            self.updateCarToEnd(car)
            return True
        road = self.roadList[car.cur_edge]
        cur_road = self.roadList[car.next_edge]  # 获取车辆行驶的下一条道路
        s = min(car.speed, cur_road.limit_rate)  # 可以行驶的距离
        # 按规则减去之前那条路已经行驶的距离
        if s > car.cur_pos:
            s -= car.cur_pos
        else:  # s==0
            self.updateCarToTopFront(car)
            return True
        road_len = cur_road.road_len
        for i in range(cur_road.num_lane):  # 遍历车道
            j = road_len
            for j in range(road_len - 1, road_len - s - 1, -1):  # 尝试着开更远
                if cur_road.status[j, i] != 0:  #
                    front_car = self.carList[cur_road.status[j, i]]  # 前车
                    if front_car.status == 0:
                        return False  # 前车未调度，无法调度
                    else:
                        j += 1  # 前车已停，发车到上一格
                        break
            if j == road_len: continue  # 如果是道路最后一格，检查下一条车道
            self.updateCarToNextRoad(car, road, cur_road, j, i)
            return True  # 调度到下条道路(j,i)位置
        self.updateCarToTopFront(car)
        return True  # 道路堵死， 调度到路口最前
