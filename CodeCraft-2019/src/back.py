'''
       行不通！！因为无法预先判断，车辆拐弯的时候，进入的是下条道路的哪一个车道
       判断是否存在死锁，若存在，改其中一个路口的车辆的方向
       car : 当前跟踪到的车
       dir : 右环还是左环
       kth : 第几次转弯（一定是第4次并且回到访问过的点，才代表有环）
   '''

#
# def check_deadlock(self, car, dir, kth): # ；
#     cur_road = self.roadList[car.cur_edge]
#     x = car.cur_pos
#     y = car.cur_lane
#     s = min(car.speed, cur_road.limit_rate)  # 可以行驶的距离
#
#     if s > x: # 说明行驶距离可以超过路，求出剩余行驶距离
#         s -= x
#     else:  # 说明行驶距离无法超过路口
#         return
#
#     for i in range(x - 1, max(-1, x - s - 1), -1):
#         if cur_road.status[i][y] != 0:
#             new_car = self.carList[cur_road.status[i][y]]
#             if new_car.cur_dir == 'R':

import math


# TODO 不需要求三个方向的最短路，可以优化！
def get_direction(self, car):
    if car.cur_edge == -1:  # 初始方向都定为直行
        car.cur_dir = 'D'
        return

    min_len = 0x3f3f3f3f
    road_id = -1
    # 遍历该出发点的每一个出口，粗略估计每条路的行驶代价
    for edge_id in self.crossList[car.cur_point].edgeList2:
        if edge_id == -1: continue
        if math.fabs(edge_id - car.cur_edge) == 100000: continue  # 去掉来时的方向
        edge = self.roadList[edge_id]

        startNode = self.crossList[edge.v]
        endNode = self.crossList[car.end_point]
        dis = self.calculateshortestpath(car.speed, startNode, endNode)

        edge_weight = math.ceil(edge.road_len * 1.0 / min(car.speed, edge.limit_rate))  # 边的权重为长度除以限速，粗略估计
        status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数
        dis += edge_weight + 3 * status_weight

        if dis < min_len:
            min_len = dis
            road_id = edge_id

    if road_id == -1: return min_len
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


def driveCarInGarage(self):
    # 这里不能遍历完整的车列表，代价太大
    for car in self.sorted_carList:
        if car.start_time == self.current_time:
            car = self.carList[car.id]  # 重新获取该车的引用
            if len(self.schedule_carList) > 500:
                car.start_time += 1
                continue

            min_len = 0x3f3f3f3f
            road_id = -1
            # 遍历该出发点的每一个出口，粗略估计每条路的行驶代价
            for edge_id in self.crossList[car.start_point].edgeList2:
                if edge_id == -1: continue
                edge = self.roadList[edge_id]

                startNode = self.crossList[edge.v]
                endNode = self.crossList[car.end_point]
                dis = self.calculateshortestpath(car.speed, startNode, endNode)

                edge_weight = math.ceil(edge.road_len * 1.0 / min(car.speed, edge.limit_rate))  # 边的权重为长度除以限速，粗略估计
                status_weight = math.ceil(edge.car_num * 1.0 / edge.num_lane)  # 路况权重为车的数量除以车道数
                dis += edge_weight + 3 * status_weight

                # edge_weight = math.ceil(edge.road_len * 1.0 / edge.min_speed)
                # dis += edge_weight

                if dis < min_len:
                    min_len = dis
                    road_id = edge_id

            if road_id == -1: continue
            cur_road = self.roadList[road_id]
            num_lane = cur_road.num_lane
            road_len = cur_road.road_len

            if cur_road.car_num > num_lane * road_len / 5:
                car.start_time += 1
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
                    # 车辆
                    car.status = 1
                    car.path.append(cur_road.road_id)
                    car.cur_lane = i
                    car.cur_pos = j
                    car.cur_edge = cur_road.road_id
                    car.cur_point = cur_road.v
                    # print(str(car.id) + " depart!!")
                    is_depature = True
                    break

            if not is_depature:  # 表示此次没有出库，出发时间后移1
                car.start_time += 1
