# coding=utf-8
import numpy as np


class Node:

    def __init__(self, node_id):
        self.id = node_id
        self.edgeList1 = []  # 邻接道路列表（边信息，用边，因为边含节点）
        self.edgeList2 = []  # 邻接道路列表（边信息，用边，因为边含节点）
        self.vis = False
        self.dis = 0x3f3f3f3f
        self.pre = -1  # 边信息，用边，因为边含节点

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
        self.scheduleList = []
        # print("road(u,v,len,channel): " + str(road_id) + ' ' + str(u) + ' ' + str(v) + ' ' + str(road_len) + ' ' + str(num_lane))


class Car:

    def __init__(self, car_id, start_point, end_point, speed, start_time, is_priority, is_preset):
        self.id = car_id
        self.speed = speed
        self.start_point = start_point
        self.end_point = end_point
        self.start_time = start_time
        self.is_priority = is_priority
        self.is_preset = is_preset
        self.end_time = 10000
        self.status = 0  # 车的状态，0表示等待状态，1表示终止状态
        self.path = []  # 车的导航路径，即最终结果
        self.path_pos = -1  # 导航路径的位置，供预置车辆使用
        self.cur_point = start_point  # 下一个路口（即将到达)的id号
        self.last_edge = -1  # 上一轮所在道路id
        self.cur_edge = -1  # 当前所在道路id
        self.next_edge = - 1  # 下条道路id,和cur_dir对应
        self.next_next_edge = -1  # 下下条道路id
        self.cur_lane = -1  # 当前所在车道id
        self.cur_pos = -1  # 当前所在的排数
        self.cur_dir = ''  # 下一个路口的转弯方向, 'D'|'L'|'R'
        self.sure_dir = ''  # 表示车辆在当前时候已经调度下所使用的方向
        # print("car(start->end): " + str(car_id) + ' ' + str(start_point) + ' ' + str(end_point))
