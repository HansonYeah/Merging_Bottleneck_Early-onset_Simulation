#coding=utf-8
import random
import math
import numpy as np
import D1xodrRoads_HXL as Roads
from functools import reduce
import time
import json

class Vehicle():
    def __init__(self, graph, id, veh_type, path_id_lst, plot_vehicle, veh_params, sim_step):
        self.graph = graph
        self.id = id
        self.length = 5 + random.uniform(-0.5,0.5)
        self.width = 1.8 + random.uniform(-0.1,0.1)

        self.max_acc = 1.6
        self.cf_hwd = 1.0 #0.97
        self.mlc_fhwd = 0.91
        self.mlc_bhwd = 0.51
        self.dlc_fhwd = 0.37
        self.dlc_bhwd = 0.51
        self.dlc_vgain = 0 #-1
        self.dlc_sgain = 5
        self.mlc_dist = 180
        self.net_dist = 1.0 + random.uniform(-0.2, 0.2)
        self.reac_time = 1.0 + random.uniform(-0.2, 0.2)
        self.max_acc = 1.6 + random.uniform(-0.2, 0.2)
        self.comfort_dec = 2.0 + random.uniform(-0.2, 0.2)
        self.acc = 0

        self.next_lane = None
        self.path_id = path_id_lst
        self.path = [graph.get_link(i) for i in self.path_id]
        self.path_order = 0
        self.current_link = self.path[0]
        if self.current_link.id == -1:
            start_pos = 300
        else:
            start_pos = 0
        self.position = start_pos
        self.desired_speed = veh_params.desired_speed + random.uniform(-3, 3)
        in_lane_lst = [lane for lane in self.current_link.lane_lst]
        for car in graph.vehicles.values():
            if car.current_link.id != self.current_link.id:
                continue
            if car.id != 0 and car.current_lane in in_lane_lst and car.position - car.length < start_pos + self.length + self.net_dist:
                in_lane_lst.remove(car.current_lane)
        if not in_lane_lst:
            self.current_lane = self.current_link.lane_lst[random.randint(0, len(self.current_link.lane_lst)-1)]
            self.find_beginfront()
        else:
            if self.current_link.id == -3 and random.random() < 0.1: #没用
                self.current_lane = self.current_link.lane_lst[2]
                self.find_front()
                self.speed = self.front.speed
            else:
                front_dist = []
                front_speed = []
                for inlane in in_lane_lst:
                    self.current_lane = inlane
                    self.find_front()
                    front_dist.append(self.front.dist)
                    front_speed.append(self.front.speed + random.uniform(-1,1))
                max_dist = max(front_dist)
                max_idx = front_dist.index(max_dist)
                self.current_lane = in_lane_lst[max_idx]
                self.speed = front_speed[max_idx]

        [self.world_x, self.world_y, self.heading] = Roads.lanepos2_worldxy(self.current_lane, self.position)
        self.speedy = 0
        self.not_traj = 0
        self.lc_flag = 0
        self.lc_phase = 0
        self.motiv_time = 0
        self.static_time = 0
        self.lc_traj = []
        self.type = veh_type
        self.status = 1 #1-D1VEH, 0-DIE, 2-D2VEH, 3-TRAJ
        self.sim_step = sim_step
        self.plot_vehicle = plot_vehicle
        self.be_cutin = [] #表征路肩车道车辆是否正在被cutin的状态，也就是前车的换道阶段为3
        self.sim_time = 0
        self.traj_rank = 0
        self.danger = 3
        self.lateral = 0
        self.flc = 0
        self.scatter = 0
        self.overreact = 0
        coop = random.random() < 0.1 and 1 or 0
        self.coop_lc = coop
        self.coop_dec = coop
        self.coop_yes = 0
        self.coop_arlc = random.random() < 0.3 and 1 or 0
        randn = random.random()
        self.mlc_gapdec = randn < 0.1 and -0.5 or 0
        self.mlc_gapdec = randn > 0.9 and -1 or 0
        self.overdec_ratio = random.random() < 1 and 1 or 0
        self.para_drive = random.random() < 0.86 and 1 or 0  # （并行式汇入）
        self.para_yes = 0
        self.ar_lc = random.random() < 0.51 and 1 or 0  # （主动-回应汇入）
        self.end_merge = random.random() < 0.51 and 1 or 0  # 延迟汇入（末端汇入）
        self.end_merge_pos = 0.4 + random.gauss(0, 0.05)
        self.late_lc_yes = 0
        self.chain_lc_dec = random.random() < 0.9 and 1 or 0
        self.coop_arlc_yes = 0
        self.left_lc = 0

        self.turn_take = 1 #if self.ar_lc and not self.para_drive else 0
        self.lateral_res = random.random() < 0.4 and -abs(random.gauss(0, 0.15)) or abs(random.gauss(0, 0.2)) #侧向安全的随机数
        self.lateral_offset = 0.0 #侧向距离车道中心线的横向距离
        self.dec_duration = 0.0 #减速持续时间
        self.time_2last_lc = 0.0 #距离上一次换道过去的时间，用于避免连续性换道
        graph.vehicles[self.id] = self

    @property
    def is_ramp_veh(self): #是在普通路段还是虚拟路段（连接段）上
        if (self.current_link.id == -3 and self.current_lane.id <= -5): # (self.current_link.id == -3 and self.current_lane.id == -4) or
            return 1
        else:
            return 0

    @property
    def is_shoulder_veh(self):  # 是在普通路段还是虚拟路段（连接段）上
        if (self.current_link.id == -3 and self.current_lane.id == -4): #  (self.current_link.id == -3 and self.current_lane.id == -3) or
            return 1
        else:
            return 0

    @property
    def on_link(self):  # 是在普通路段还是虚拟路段（连接段）上
        return self.current_link.junction_id == -1 and 1 or 0

    @property
    def has_mlc_desire(self):  # 是在普通路段还是虚拟路段（连接段）上
        if self.position > 3 and self.current_lane not in self.desired_lane_lst:
            return 1
        else:
            return 0

    ########################################
    #用于一二维模型转化部分的方法
    def draw(self):
        p = Roads.get_rect(self.world_x, self.world_y, self.width, self.length, self.heading)
        xdata = [e[0] for e in p]
        ydata = [e[1] for e in p]
        self.plot_vehicle.set_data(xdata, ydata)
        # if self.late_lc_yes:
        #     self.plot_vehicle.set_color('yellow')
        # elif self.coop_yes:
        #     self.plot_vehicle.set_color('green')
        # elif self.coop_arlc_yes:
        #     self.plot_vehicle.set_color('blue')
        # else:
        self.plot_vehicle.set_color('red')

    ########################################

    def IDM(self, front_speed, gap, dv0=math.inf):#IDM跟驰模型
        #输入变量front_speed前车速度，space前车车尾与本车车头之间的车身净距
        net_dist = self.net_dist
        v = max(0, self.speed)
        fv = front_speed
        dv = min(self.desired_speed, self.current_lane.speed_limit)
        dv = min(dv, dv0)
        if self.current_link.id == -4 and self.position > 300:  # 用来模拟末端车辆速度受限的情况
            dv = 60 / 3.6

        rt = self.reac_time
        ma = self.max_acc
        cd = self.comfort_dec
        if self.overreact and self.is_shoulder_veh and self.rightfront.vehicle and self.rightfront.vehicle.lc_phase == 2:
            net_dist = net_dist * 1.2
            rt = rt * 1.2
        if gap < net_dist:
            acc = (0 - self.speed) / self.sim_step
        else:
            desired_dist = net_dist + 2 * np.math.sqrt(v/dv) + rt*v + v*(v-fv) / 2.0 / np.math.sqrt(ma * cd)
            acc = ma*(1-(v/dv) ** 4 - (desired_dist / gap) ** 2)
        return acc

    def follow_in_lc(self): # 对应于文档中的6.1.7换道影响模型，处于换道阶段2的车辆还要对目标车道前车进行跟驰
        if self.lc_phase == 2:
            if self.lc_flag == 1:
                front_speed = self.leftfront.speed
                gap = self.leftfront.gap
            else:
                front_speed = self.rightfront.speed
                gap = self.rightfront.gap
            vehicle_vir_acc = self.IDM(front_speed, gap)
            self.acc = min(self.acc, vehicle_vir_acc)

    def follow_lc_veh(self):  # 对应于文档中的6.1.7换道影响模型，处于换道阶段2的车辆还要对目标车道后车产生影响
        if self.is_shoulder_veh and self.front.is_movable() and self.front.vehicle.lc_phase == 3:
            if self.front.id not in self.be_cutin:
                self.be_cutin.append(self.front.id)
        if self.leftfront.is_movable() and self.leftfront.vehicle.lc_phase == 2 and self.leftfront.vehicle.lc_flag == 2:
            follow_lc_acc = self.IDM(self.leftfront.speed, self.leftfront.gap - 2)
            self.acc = min(self.acc, follow_lc_acc)
        if self.rightfront.is_movable() and self.rightfront.vehicle.lc_phase == 2 and self.is_shoulder_veh and self.position >self.current_lane.length * 0.66: #对加速车道末端车辆提前反应进入
            follow_lc_acc = self.IDM(max(3, self.rightfront.speed), self.rightfront.gap - 2)
            self.acc = min(self.acc, follow_lc_acc)
            return
        if self.rightfront.is_movable() and self.rightfront.vehicle.lc_phase == 2 and self.rightfront.vehicle.lc_flag == 1:
            follow_lc_acc = self.IDM(self.rightfront.speed, self.rightfront.gap - 2)
            self.acc = min(self.acc, follow_lc_acc)

    def find_nextlane(self): #对应于文档中的4.2,查找本车的下一车道，即根据路径确定下游将要走的车道
        if self.at_last_link() or self.current_lane not in self.desired_lane_lst:
            self.next_lane = None
            return
        nextlane_lst = []
        for lane in self.current_lane.out_lane_lst:
            if lane.ownner.id in self.path_id:
                nextlane_lst.append(lane)
        if not nextlane_lst: #如果为空，说明当前车道非期望车道，但距离必须换道点还比较远
            self.next_lane = None
            return
        self.next_lane = nextlane_lst[random.randint(0, len(nextlane_lst) - 1)]


    def follow_by_mlc(self): #对应于文档中的6.1.5强制性换道条件下的跟驰
        if self.is_ramp_veh and self.position < self.current_lane.length*0.8:
            lanes2change0 = 1
        else:
            lanes2change0 = self.get_lanes2change()
        if lanes2change0 == 0:
            return
        if self.position > self.current_lane.length * 0.8 and self.speed < 3 and self.leftfront.gap / max(0.1, self.speed) > 1 and self.lc_flag:
            return
        if self.end_merge and self.position < self.end_merge_pos * self.current_lane.length:
            return

        lanes2change = abs(lanes2change0)
        if lanes2change == 1:
            dist2stop = self.rest_length() - lanes2change * 10 - self.current_link.lane_lst.index(self.current_lane) * 0
        else:
            dist2stop = self.rest_length() - lanes2change * 25 - self.current_link.lane_lst.index(self.current_lane) * 0
        front_speed = 5
        gap = dist2stop
        if self.leftfront.speed > 5:
            vehicle_mlc_acc = self.IDM(front_speed, gap, self.leftfront.speed * 1.1)
        else:
            vehicle_mlc_acc = self.IDM(front_speed, gap, self.leftfront.speed + 5)
        self.acc = min(self.acc, vehicle_mlc_acc)

    def is_sim(self):
        return self.status == 1

    def get_frontlink(self, k):
        if self.path_order + k < len(self.path):
            return self.path[self.path_order + k]
        else:
            return None

    def get_vehicles_on_front_link(self, graph_pre): #查找下游link上的车辆
        front_view_dist = 100
        front1_vehicles, front2_vehicles, front3_vehicles = graph_pre.get_vehicles_in_front_link(self.get_frontlink(0), self.get_frontlink(1), self.get_frontlink(2))

        for one in front1_vehicles[::-1]:
            if one.position <= self.position or not self.at_same_lane(one):
                front1_vehicles.remove(one)
        if front1_vehicles:
            return front1_vehicles, [], []

        for one in front2_vehicles[::-1]:
            if one.current_lane not in self.current_lane.out_lane_lst or self.rest_length() + one.position > front_view_dist:
                front2_vehicles.remove(one)
        if front2_vehicles:
            return  front1_vehicles, front2_vehicles, []

        front_link1 = self.get_frontlink(1)
        if not front_link1 or front_link1.lane_lst[0].length > front_view_dist:
            return front1_vehicles, front2_vehicles, []
        for one in front3_vehicles[::-1]:
            lane_inbetween = self.graph.get_lane_inbetween(self.current_lane, one.current_lane)
            if lane_inbetween is None or self.rest_length() + front_link1.lane_lst[0].length + one.position > front_view_dist:
                front3_vehicles.remove(one)
        return front1_vehicles, front2_vehicles, front3_vehicles

    def find_beginfront(self):#初始化时找当前车道前车
        f1 = self.graph.get_vehicles_in_lane(self.current_lane)
        if f1:  # if the front vehicle is in the current lane
            one = reduce(lambda x, y: x.position < y.position and x or y, f1)
            self.position = one.position - self.desired_speed * 1
            self.speed = one.speed
            return

    def find_front(self): #找前车
        f1, f2, f3 = self.get_vehicles_on_front_link(self.graph)  # find the frontvehicles according to the road topo

        self.front = surr_vehicle(self.desired_speed)

        if f1:  # if the front vehicle is in the current lane
            one = reduce(lambda x, y: x.position < y.position and x or y, f1)
            if one.position - self.position < self.front.dist:
                front_dist = one.position - self.position
                front_gap = front_dist - one.length
                self.front.update(one, front_dist, front_gap)
            return

        if f2:
            lanes = {}  # record the closest vehicle in different lanes:lane:vehicle
            for one in f2:
                if one.current_lane.id not in lanes:
                    lanes[one.current_lane.id] = surr_vehicle(self.desired_speed)

                if one.position < lanes[one.current_lane.id].dist:
                    front_dist = one.position + self.rest_length()
                    front_gap = front_dist - one.length
                    lanes[one.current_lane.id].update(one, front_dist, front_gap)
            self.front = reduce(lambda e1, e2: e1.dist > e2.dist and e1 or e2, lanes.values())
            return

        if f3:
            lanes = {}
            for one in f3:
                if one.current_lane.id not in lanes:
                    lanes[one.current_lane.id] = surr_vehicle(self.desired_speed)

                lane_inbetween = self.graph.get_lane_inbetween(self.current_lane, one.current_lane)
                if self.rest_length() + lane_inbetween.length + one.position < lanes[one.current_lane.id].dist:
                    front_dist = self.rest_length() + lane_inbetween.length + one.position
                    front_gap = front_dist - one.length
                    lanes[one.current_lane.id].update(one, front_dist, front_gap)
            self.front = reduce(lambda e1, e2: e1.dist > e2.dist and e1 or e2, lanes.values())
        return

    def find_nearbyveh(self): #对应于文档5.2，找周围车辆：左右车道前后车
        rightfront = surr_vehicle(self.desired_speed)
        rightbehind = surr_vehicle()
        leftfront = surr_vehicle(self.desired_speed)
        leftbehind = surr_vehicle()

        [lvehs, rvehs] = self.graph.get_vehicles_in_lanes(self.current_lane.llane, self.current_lane.rlane)

        if self.current_lane.llane:
            for one in lvehs:
                if one.position > self.position and one.position - self.position < leftfront.dist:
                    front_dist = one.position - self.position
                    front_gap = front_dist - one.length / 2.0 - self.length / 2.0
                    leftfront.update(one, front_dist, front_gap)
                if one.position < self.position and self.position - one.position < leftbehind.dist:
                    behind_dist = self.position - one.position
                    behind_gap = behind_dist - self.length / 2.0 - self.length / 2.0
                    leftbehind.update(one, behind_dist, behind_gap)
        if self.current_lane.rlane:
            # for one in self.graph.get_vehicles_in_lane(self.current_lane.rlane):
            for one in rvehs:
                if one.position > self.position and one.position - self.position < rightfront.dist:
                    front_dist = one.position - self.position
                    front_gap = front_dist - one.length / 2.0 - self.length / 2.0
                    rightfront.update(one, front_dist, front_gap)
                if one.position < self.position and self.position - one.position < rightbehind.dist:
                    behind_dist = self.position - one.position
                    behind_gap = behind_dist - self.length / 2.0 - self.length / 2.0
                    rightbehind.update(one, behind_dist, behind_gap)

        if self.current_lane.llane:
            if not leftbehind.is_movable(): #如果当前车道左侧车道没有前后车，就再往上/下游车道查找左后/前车
                all_vehicles = []
                for inlane in self.current_lane.llane.in_lane_lst:
                    v = self.graph.get_vehicles_in_lane(inlane)
                    all_vehicles.extend(v)
                for one in all_vehicles:
                    if one.current_lane.length - one.position + self.position < leftbehind.dist:
                        behind_dist = one.current_lane.length - one.position + self.position
                        behind_gap = behind_dist - self.length / 2.0 - self.length / 2.0
                        leftbehind.update(one, behind_dist, behind_gap)
            if not leftfront.is_movable() and not self.at_last_link():
                all_vehicles = []
                lanes = {}
                for outlane in self.graph.conn_lanes_of_nextlink(self.current_lane.llane, self.path[self.path_order + 1]):
                    v = self.graph.get_vehicles_in_lane(outlane)
                    all_vehicles.extend(v)
                    lanes[outlane.id] = surr_vehicle(self.desired_speed)
                for one in all_vehicles:
                    if one.position + self.rest_length() < lanes[one.current_lane.id].dist:
                        front_dist = one.position + self.rest_length()
                        front_gap = front_dist - one.length / 2.0 - self.length / 2.0
                        lanes[one.current_lane.id].update(one, front_dist, front_gap)
                front_dist = 0
                for one in lanes:
                    if lanes[one].dist > front_dist:
                        front_dist = lanes[one].dist
                        leftfront = lanes[one]

        if self.current_lane.rlane:
            if not rightbehind.is_movable(): #如果当前车道右侧车道没有前后车，就再往上/下游车道查找右后/前车
                all_vehicles = []
                for inlane in self.current_lane.rlane.in_lane_lst:
                    v = self.graph.get_vehicles_in_lane(inlane)
                    all_vehicles.extend(v)
                for one in all_vehicles:
                    if one.current_lane.length - one.position + self.position < rightbehind.dist:
                        behind_dist = one.current_lane.length - one.position + self.position
                        behind_gap = behind_dist - self.length / 2.0 - self.length / 2.0
                        rightbehind.update(one, behind_dist, behind_gap)
            if not rightfront.is_movable() and not self.at_last_link():
                all_vehicles = []
                lanes = {}
                for outlane in self.graph.conn_lanes_of_nextlink(self.current_lane.rlane, self.path[self.path_order + 1]):
                    v = self.graph.get_vehicles_in_lane(outlane)
                    all_vehicles.extend(v)
                    lanes[outlane.id] = surr_vehicle(self.desired_speed)
                for one in all_vehicles:
                    if one.position + self.rest_length() < lanes[one.current_lane.id].dist:
                        front_dist = one.position + self.rest_length()
                        front_gap = front_dist - one.length / 2.0 - self.length / 2.0
                        lanes[one.current_lane.id].update(one, front_dist, front_gap)
                front_dist = 0
                for one in lanes:
                    if lanes[one].dist > front_dist:
                        front_dist = lanes[one].dist
                        rightfront = lanes[one]

        if self.current_lane.llane:
            if not leftfront.is_movable() and self.path_order + 2 < len(self.path) and len(self.current_lane.llane.out_lane_lst) > 0:
                all_vehicles = []
                lanes = {}
                for outlane in self.graph.conn_lanes_of_nextlink(self.current_lane.llane.out_lane_lst[0], self.path[self.path_order + 2]):
                    v = self.graph.get_vehicles_in_lane(outlane)
                    all_vehicles.extend(v)
                    lanes[outlane.id] = surr_vehicle(self.desired_speed)
                for one in all_vehicles:
                    if one.position + self.rest_length() + self.current_lane.llane.out_lane_lst[0].length < lanes[one.current_lane.id].dist:
                        front_dist = one.position + self.rest_length() + self.current_lane.llane.out_lane_lst[0].length
                        front_gap = front_dist - one.length / 2.0 - self.length / 2.0
                        lanes[one.current_lane.id].update(one, front_dist, front_gap)
                front_dist = 0
                for one in lanes:
                    if lanes[one].dist > front_dist:
                        front_dist = lanes[one].dist
                        leftfront = lanes[one]

        if self.current_lane.rlane:
            if not rightfront.is_movable() and self.path_order + 2 < len(self.path) and len(self.current_lane.rlane.out_lane_lst) > 0:
                all_vehicles = []
                lanes = {}
                for outlane in self.graph.conn_lanes_of_nextlink(self.current_lane.rlane.out_lane_lst[0], self.path[self.path_order + 2]):
                    v = self.graph.get_vehicles_in_lane(outlane)
                    all_vehicles.extend(v)
                    lanes[outlane.id] = surr_vehicle(self.desired_speed)
                for one in all_vehicles:
                    if one.position + self.rest_length() + self.current_lane.rlane.out_lane_lst[0].length < lanes[one.current_lane.id].dist:
                        front_dist = one.position + self.rest_length() + self.current_lane.rlane.out_lane_lst[0].length
                        front_gap = front_dist - one.length / 2.0 - self.length / 2.0
                        lanes[one.current_lane.id].update(one, front_dist, front_gap)
                front_dist = 0
                for one in lanes:
                    if lanes[one].dist > front_dist:
                        front_dist = lanes[one].dist
                        rightfront = lanes[one]

        self.rightfront = rightfront
        self.rightbehind = rightbehind
        self.leftfront = leftfront
        self.leftbehind = leftbehind

    def find_mergeveh(self): #找合流车辆，路网0820
        self.merge_veh = surr_vehicle(self.desired_speed)

        if self.is_shoulder_veh and self.rightfront.gap < self.front.gap and self.lc_flag != 1:
            self.merge_veh = self.rightfront
            return

        if self.current_link.id != -8 or self.current_lane.id != -3:
            return

        mergevehs = self.graph.get_vehicles_in_link(self.graph.get_link(-14))
        merge_view_dist = 30
        merge_veh = surr_vehicle(self.desired_speed)

        if self.rest_length() > merge_view_dist:
            return
        for v in mergevehs:
            merge_dist = self.rest_length() - v.rest_length()
            if merge_dist > 0 and merge_dist < merge_veh.dist:  # 将距离合流点更近的车辆作为合流前车
                merge_gap = merge_dist - v.length
                merge_veh.update(v, merge_dist, merge_gap)
        if merge_veh.gap < self.front.gap:
            merge_veh.dist = self.rest_length()  # 将merge dist更新为车辆距离冲突点距离
            self.merge_veh = merge_veh
            return

        if not merge_veh.is_movable():  # 否则，再往下游找一个车道
            mergevehs = self.graph.get_vehicles_in_lane(self.graph.get_link(-14).lane_lst[0].out_lane_lst[0])
            for v in mergevehs:
                merge_dist = self.rest_length() + v.position
                if merge_dist > 0 and merge_dist < merge_veh.dist:  # 将距离合流点更近的车辆作为合流前车
                    merge_gap = merge_dist - v.length
                    merge_veh.update(v, merge_dist, merge_gap)
        if merge_veh.gap < self.front.gap:
            self.merge_veh = merge_veh

    def find_behind(self):
        bvehs = self.graph.get_vehicles_in_lane(self.current_lane)  # find the frontvehicles according to the road topo

        self.behind = surr_vehicle(0)

        if not bvehs:
            return
        for bveh in bvehs:
            if self.position > bveh.position and self.position - bveh.position < self.behind.dist:
                behind_dist = self.position - bveh.position
                behind_gap = behind_dist - self.length
                self.behind.update(bveh, behind_dist, behind_gap)

    def find_surr_vehs(self):
        self.find_front() #找本车道前车，对应于文档5.1
        self.find_behind() #只找本车道后车
        self.find_nearbyveh() #找相邻车道前后车，对应于文档5.2
        self.find_mergeveh()

    def rest_length(self): #当前车道剩余长度
        return self.current_lane.length - self.position

    def at_same_lane(self, v):
        return self.current_lane.id == v.current_lane.id

    def at_last_link(self): #是否在本车路径中的最后一个路段
        if self.path_order == len(self.path) - 1:
            return True
        else:
            return False

    def calc_link_sequence(self):  #向路径下游找一段距离mlc_view_dist，在该距离内车辆将要行驶的路段
        mlc_view_dist = max(400, 3 * self.speed + 10)
        link_lst = []

        order = self.path_order
        link = self.current_link
        link_lst.append(link)
        mlc_view_dist -= (self.current_lane.length - self.position)

        while mlc_view_dist > 0:
            if order >= len(self.path) - 1:
                break
            order += 1
            link = self.path[order]
            mlc_view_dist -= link.lane_lst[0].length
            link_lst.append(link)
        if len(link_lst) == 1 and not self.at_last_link():
            link_lst.append(self.path[self.path_order + 1])
        return link_lst

    def get_pass_lane(self, link_lst): #遍历当前路段的车道，判断哪些车道的下游车道属于link_lst
        if not link_lst:
            raise()
        link_lst = link_lst[::-1]
        lanes = list(link_lst[0].iter_lane())
        while link_lst[1:]:
            prev_link = link_lst[1]
            #如果从prev_link 到达后一个link没有lane，那就使用prev_link做为最后一个link
            lanes = self.graph.get_sub_lane_to_outlane(prev_link, lanes) or list(prev_link.iter_lane())
            link_lst = link_lst[1:]
        return lanes

    def get_desiredlanes_by_network(self): #对应于文档4.1，确定当前link的期望车道
        if self.on_link:
            links = self.calc_link_sequence()
            lane_lst = self.get_pass_lane(links)
            if not lane_lst : #可能切换的时候刚好本车道剩余长度不足以支持换道，就让车辆再往下走一段路后再换道 or (self.rest_length()<self.cal_lc_dist_with_speed() and len(links)>1 and links[1].lane_lst[0].length>self.rest_length())
                lane_lst = [self.current_lane]
            self.desired_lane_lst = lane_lst
        else:
            self.desired_lane_lst=[self.current_lane]

    def get_lanes2change(self): #计算如果要到期望车道上至少要进行几次换道

        lanes2change = 0
        if self.current_lane in self.desired_lane_lst:
            return lanes2change

        lane = self.current_lane.llane
        while lane :
            lanes2change += 1
            if lane in self.desired_lane_lst:
                return lanes2change
            lane = lane.llane

        lanes2change = 0
        lane = self.current_lane.rlane
        while lane:
            lanes2change -= 1
            if lane in self.desired_lane_lst:
                return lanes2change
            lane = lane.rlane
        print('NOT FOUND', lanes2change)
        return 0

    def get_mangap(self, lanes2change): #强制换道的最小间隙计算，对应于文档6.1间隙选择
        if self.scatter:
            res_dist = 0.2*2.237*(1-np.exp(-0.05*10))
        else:
            res_dist = 0.2 * 2.237 * (1 - np.exp(-0.05 * (self.current_lane.length - self.position - 20 * (lanes2change - 1))))
        gap_lf=1+0.15*2.237*max(0,self.speed-self.leftfront.speed) \
               +0.3*2.237*min(0,self.speed-self.leftfront.speed) \
               +self.mlc_fhwd*2.237*self.speed \
               +res_dist+random.gauss(0, 1)+self.mlc_gapdec
        gap_rf=1+0.15*2.237*max(0,self.speed-self.rightfront.speed) \
               +0.3*2.237*min(0,self.speed-self.rightfront.speed) \
               +self.mlc_fhwd*2.237*self.speed \
               +res_dist+random.gauss(0, 1)+self.mlc_gapdec
        gap_lb=1.5+0.1*2.237*max(0,self.leftbehind.speed-self.speed) \
               +0.35*2.237*min(0,self.leftbehind.speed-self.speed) \
               +self.mlc_bhwd*2.237*self.leftbehind.speed \
               +res_dist+random.gauss(0, 1.5)+self.mlc_gapdec
        gap_rb=1.5+0.1*2.237*max(0,self.rightbehind.speed-self.speed) \
               +0.35*2.237*min(0,self.rightbehind.speed-self.speed) \
               +self.mlc_bhwd*2.237*self.rightbehind.speed \
               +res_dist+random.gauss(0, 1.5)+self.mlc_gapdec
        gap_lf = max(0, gap_lf)
        gap_rf = max(0, gap_rf)
        gap_lb = max(0, gap_lb)
        gap_rb = max(0, gap_rb)
        return gap_lf, gap_rf, gap_lb, gap_rb

    def get_disgap(self): #自由换道的最小间隙计算，对应于文档6.2间隙选择
        dis_gap_lf =1+0.2*2.237*max(0, self.speed-self.leftfront.speed)+\
                   0.35*2.237*min(0,self.speed-self.leftfront.speed)+self.dlc_fhwd*2.237*self.speed+random.gauss(0, 1)
        dis_gap_rf=1+0.2*2.237*max(0,self.speed-self.rightfront.speed)+\
                   0.35*2.237*min(0,self.speed-self.rightfront.speed)+self.dlc_fhwd*2.237*self.speed+random.gauss(0, 1)
        dis_gap_lb=1.5+0.15*2.237*max(0,self.leftbehind.speed-self.speed)+\
                   0.45*2.237*min(0,self.leftbehind.speed-self.speed)+self.dlc_bhwd*2.237*self.leftbehind.speed+random.gauss(0, 1.5)
        dis_gap_rb=1.5+0.15*2.237*max(0,self.rightbehind.speed-self.speed)+\
                   0.45*2.237*min(0,self.rightbehind.speed-self.speed)+self.dlc_bhwd*2.237*self.rightbehind.speed+random.gauss(0, 1.5)
        dis_gap_lf = max(0, dis_gap_lf)
        dis_gap_rf = max(0, dis_gap_rf)
        dis_gap_lb = max(0, dis_gap_lb)
        dis_gap_rb = max(0, dis_gap_rb)
        return dis_gap_lf, dis_gap_rf, dis_gap_lb, dis_gap_rb

    def get_disgap_SH(self): #自由换道的最小间隙计算，对应于文档6.2间隙选择
        surr_speed = self.get_surr_speed()

        if surr_speed > 40/3.6:
            dis_gap_lf = 9.03 + 3.37 * max(0, self.speed - self.leftfront.speed) + random.gauss(0, 1.4)
            dis_gap_rf = 9.03 + 3.37 * max(0, self.speed - self.rightfront.speed) + random.gauss(0, 1.4)
            dis_gap_lb = 3.28 + 1.94 * max(0, self.leftbehind.speed - self.speed) + 0.85 * self.leftbehind.speed + random.gauss(0, 2.5)
            dis_gap_rb = 3.28 + 1.94 * max(0, self.rightbehind.speed - self.speed) + 0.85 * self.rightbehind.speed + random.gauss(0, 2.5)
        else:
            dis_gap_lf = 0.52 + 3.34 * max(0, self.speed - self.leftfront.speed) + 0.5 * self.speed + random.gauss(0, 0.6)
            dis_gap_rf = 0.52 + 0.34 * max(0, self.speed - self.rightfront.speed) + 0.5 * self.speed + random.gauss(0, 0.6)
            dis_gap_lb = 0.81 + 0.34 * max(0, self.leftbehind.speed - self.speed) + 0.4 * min(0,self.leftbehind.speed - self.speed) + 0.1 * self.leftbehind.speed + random.gauss(0, 0.8)
            dis_gap_rb = 0.81 + 0.34 * max(0, self.rightbehind.speed - self.speed) + 0.4 * min(0,self.rightbehind.speed - self.speed) + 0.1 * self.rightbehind.speed + random.gauss(0, 0.8)

        max_dec = 40.0
        min_safe_lf = (self.speed * self.speed - self.leftfront.speed * self.leftfront.speed) / 2.0 / max_dec
        min_safe_rf = (self.speed * self.speed - self.rightfront.speed * self.rightfront.speed) / 2.0 / max_dec
        min_safe_lb = (self.leftbehind.speed * self.leftbehind.speed - self.speed * self.speed) / 2.0 / max_dec
        min_safe_rb = (self.rightbehind.speed * self.rightbehind.speed - self.speed * self.speed) / 2.0 / max_dec

        dis_gap_lf = max(max(0, min_safe_lf), dis_gap_lf)
        dis_gap_rf = max(max(0, min_safe_rf), dis_gap_rf)
        dis_gap_lb = max(max(0, min_safe_lb), dis_gap_lb)
        dis_gap_rb = max(max(0, min_safe_rb), dis_gap_rb)
        return dis_gap_lf, dis_gap_rf, dis_gap_lb, dis_gap_rb

    def get_surr_speed(self):
        surr_speed = []
        surr_speed.append(self.speed)
        if self.front.is_movable():
            surr_speed.append(self.front.speed)
        if self.leftfront.is_movable():
            surr_speed.append(self.leftfront.speed)
        if self.leftbehind.is_movable():
            surr_speed.append(self.leftbehind.speed)
        if self.rightfront.is_movable():
            surr_speed.append(self.rightfront.speed)
        if self.rightbehind.is_movable():
            surr_speed.append(self.rightbehind.speed)
        avg_speed = np.mean(surr_speed)
        return avg_speed

    def get_mangap_flc(self, lanes2change): #强制换道的最小间隙计算，对应于文档6.1间隙选择
        if self.scatter:
            res_dist = 0.2*2.237*(1-np.exp(-0.05*10))
        else:
            res_dist = 0.2 * 2.237 * (1 - np.exp(-0.05 * (self.current_lane.length - self.position - 20 * (lanes2change - 1))))
        gap_lf=1.0+0.15*2.237*max(0,self.speed-self.leftfront.speed) \
               +0.25*2.237*min(0,self.speed-self.leftfront.speed) \
               +0.15*2.237*self.speed \
               +res_dist+random.gauss(0, 1)+self.mlc_gapdec
        gap_rf=1.0+0.15*2.237*max(0,self.speed-self.rightfront.speed) \
               +0.25*2.237*min(0,self.speed-self.rightfront.speed) \
               +0.15*2.237*self.speed \
               +res_dist+random.gauss(0, 1)+self.mlc_gapdec
        gap_lb=1.5+0.1*2.237*max(0,self.leftbehind.speed-self.speed) \
               +0.3*2.237*min(0,self.leftbehind.speed-self.speed) \
               +0.25*2.237*self.leftbehind.speed \
               +res_dist+random.gauss(0, 1.5)+self.mlc_gapdec
        gap_rb=1.5+0.1*2.237*max(0,self.rightbehind.speed-self.speed) \
               +0.3*2.237*min(0,self.rightbehind.speed-self.speed) \
               +0.25*2.237*self.rightbehind.speed \
               +res_dist+random.gauss(0, 1.5)+self.mlc_gapdec
        gap_lf = max(0, gap_lf)
        gap_rf = max(0, gap_rf)
        gap_lb = max(0, gap_lb)
        gap_rb = max(0, gap_rb)
        return gap_lf, gap_rf, gap_lb, gap_rb

    def get_disgap_flc(self): #自由换道的最小间隙计算，对应于文档6.2间隙选择
        dis_gap_lf =1.0+0.15*2.237*max(0, self.speed-self.leftfront.speed)+\
                   0.3*2.237*min(0,self.speed-self.leftfront.speed)+0.2*2.237*self.speed+random.gauss(0, 1)
        dis_gap_rf=1.0+0.15*2.237*max(0,self.speed-self.rightfront.speed)+\
                   0.3*2.237*min(0,self.speed-self.rightfront.speed)+0.2*2.237*self.speed+random.gauss(0, 1)
        dis_gap_lb=1.5+0.15*2.237*max(0,self.leftbehind.speed-self.speed)+\
                   0.4*2.237*min(0,self.leftbehind.speed-self.speed)+0.25*2.237*self.leftbehind.speed+random.gauss(0, 1.5)
        dis_gap_rb=1.5+0.15*2.237*max(0,self.rightbehind.speed-self.speed)+\
                   0.4*2.237*min(0,self.rightbehind.speed-self.speed)+0.25*2.237*self.rightbehind.speed+random.gauss(0, 1.5)
        dis_gap_lf = max(0, dis_gap_lf)
        dis_gap_rf = max(0, dis_gap_rf)
        dis_gap_lb = max(0, dis_gap_lb)
        dis_gap_rb = max(0, dis_gap_rb)
        return dis_gap_lf, dis_gap_rf, dis_gap_lb, dis_gap_rb

    def get_lc_direction(self): #确定换道方向，对应于文档6.2车道选择
        if self.current_lane.llane in self.desired_lane_lst and self.current_lane.rlane in self.desired_lane_lst:
            lc_direction = 0  # 都可换道
            if self.is_shoulder_veh and self.position > self.current_lane.length * 0.2:
                lc_direction = 1  # 可向左换道
        elif self.current_lane.rlane in self.desired_lane_lst:
                lc_direction = -1  # 可向右换道
        elif self.current_lane.llane in self.desired_lane_lst:
                lc_direction = 1  # 可向左换道
        else:
                lc_direction = 2  # 不可换道
        return lc_direction

    def get_mlc_flag(self, lanes2change): #自由换道换道动机计算，对应于文档6.1动机生成
        if self.flc and self.position > self.current_lane.length * 0.5:
            man_gap_lf, man_gap_rf, man_gap_lb, man_gap_rb = self.get_mangap_flc(abs(lanes2change))
        else:
            man_gap_lf, man_gap_rf, man_gap_lb, man_gap_rb = self.get_mangap(abs(lanes2change))
        # 分别针对leftfront（lf),rightfront(rf),leftbehind(lb),rightbehind(rb)
        lc_direction = np.sign(lanes2change)
        self.motiv_time += np.sign(lanes2change)

        if self.current_lane.lmark == 'dashed' and lc_direction > 0 and self.leftfront.gap > man_gap_lf and self.leftbehind.gap > man_gap_lb:
            # if self.leftbehind.vehicle and self.leftbehind.vehicle.lc_flag == 1:
                self.lc_flag = 1  # 向左换道
        elif self.current_lane.rmark == 'dashed' and lc_direction < 0 and self.rightfront.gap > man_gap_rf and self.rightbehind.gap > man_gap_rb:
            self.lc_flag = 2  # 向右换道

    def get_dlc_flag(self, incentive=0): #强制换道换道动机计算，对应于文档6.2动机生成
        if incentive == 1:
            dis_gap_lf, _, dis_gap_lb, _ = self.get_disgap()
            if self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb:
                self.lc_flag = 1
        else:
            dis_gap_lf, dis_gap_rf, dis_gap_lb, dis_gap_rb = self.get_disgap()
            lc_direction = self.get_lc_direction()
            if lc_direction < 2:
                if self.front.is_movable():
                    self.front.vehicle.find_front()
                    front2_veh = self.front.vehicle.front# 前方排队
                    if front2_veh.is_movable():
                        if self.front.speed <= self.speed and front2_veh.speed <= self.front.speed:
                            if self.current_lane.lmark == 'dashed' and (lc_direction >= 0 and self.leftfront.gap > self.front.gap + 2 * self.length
                                                                        and self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb):
                                self.lc_flag = 1
                            # elif self.lateral == 1 and self.position > 80 and self.current_lane.lmark == 'dashed' and (lc_direction >= 0 and self.leftfront.gap > self.front.gap
                            #                                             and self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb):
                            #     self.lc_flag = 1
                            elif self.current_lane.rmark == 'dashed' and (lc_direction <= 0 and self.rightfront.gap > self.front.gap + 2 * self.length
                                                                          and self.rightfront.gap > dis_gap_rf and self.rightbehind.gap > dis_gap_rb):
                                self.lc_flag = 2
                                # if (self.current_link.id == -3 or self.current_link.id == -4) and random.random()<0.95: # -0.95*self.lateral
                                #     self.lc_flag = 0
                    if (self.front.speed < self.desired_speed - 20 / 3.6 and self.front.dist < 3 * self.speed):  # 前方低速车
                        if self.current_lane.lmark == 'dashed' and lc_direction >= 0 and \
                                (self.leftfront.speed > self.front.speed + self.dlc_vgain and self.leftfront.gap > dis_gap_lf) and \
                                (self.leftbehind.gap > dis_gap_lb and self.leftfront.gap > self.front.gap + self.dlc_sgain):
                            self.lc_flag = 1
                        elif self.current_lane.rmark == 'dashed' and (lc_direction <= 0 and
                                self.rightfront.speed > self.front.speed + self.dlc_vgain and self.rightfront.gap > dis_gap_rf and \
                                self.rightbehind.gap > dis_gap_rb and self.rightfront.gap > self.front.gap + self.dlc_sgain):
                            self.lc_flag = 2
                            # if (self.current_link.id == -3 or self.current_link.id == -4) and random.random()<0.95: # -0.95*self.lateral
                            #     self.lc_flag = 0
                    if (self.front.length > 8 and self.front.gap < 2 * self.speed):  # 前方大型车
                        if self.current_lane.lmark == 'dashed' and (lc_direction >= 0 and self.leftfront.speed > self.front.speed + self.dlc_vgain and self.leftfront.gap > dis_gap_lf and \
                                self.leftbehind.gap > dis_gap_lb and self.leftfront.gap > self.front.gap + self.dlc_sgain):
                            self.lc_flag = 1
                        elif self.current_lane.rmark == 'dashed' and (lc_direction <= 0 and self.rightfront.speed > self.front.speed + self.dlc_vgain and self.rightfront.gap > dis_gap_rf and \
                              self.rightbehind.gap > dis_gap_rb and self.rightfront.gap > self.front.gap + self.dlc_sgain):
                            self.lc_flag = 2
                    if self.lateral_offset > 0.3 and self.current_lane.lmark == 'dashed' and (lc_direction >= 0 and self.leftfront.speed > self.front.speed and \
                        self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb and self.leftfront.gap > self.front.gap + 1 * self.length):
                        self.lc_flag = 1
                    elif self.lateral_offset < -0.3 and self.current_lane.rmark == 'dashed' and (lc_direction <= 0 and self.rightfront.speed > self.front.speed and \
                        self.rightfront.gap > dis_gap_rf and self.rightbehind.gap > dis_gap_rb and self.rightfront.gap > self.front.gap + 1 * self.length):
                        self.lc_flag = 2

    def get_dlc_flag_SH(self, incentive=0): #强制换道换道动机计算，对应于文档6.2动机生成
        if self.current_link.id == -1 or self.current_link.id == -5:
            self.get_dlc_flag(incentive)
            return
        if incentive == 1:
            dis_gap_lf, _, dis_gap_lb, _ = self.get_disgap_SH()
            if self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb:
                self.lc_flag = 1
        elif incentive == 2:
            lc_direction = self.get_lc_direction()
            if lc_direction == -1:
                return
            dis_gap_lf, _, dis_gap_lb, _ = self.get_disgap_SH()
            if self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb and self.leftfront.gap + 2 * self.leftfront.speed > self.front.gap + 2 * self.front.speed:
                self.lc_flag = 1
        else:
            dis_gap_lf, dis_gap_rf, dis_gap_lb, dis_gap_rb = self.get_disgap_SH()
            lc_direction = self.get_lc_direction()
            if lc_direction < 2:
                if self.front.is_movable():
                    surr_speed = self.get_surr_speed()
                    right_has_lc = 0
                    rlane = self.current_lane.rlane
                    if rlane:
                        rrlane = rlane.rlane
                        if rrlane:
                            rrvehs = self.graph.get_vehicles_in_lane(rrlane)
                            for rrveh in rrvehs:
                                if rrveh.lc_flag == 1 and rrveh.lc_phase < 3 and abs(rrveh.position - self.position) < 8: #找右右侧车道平行位置的车辆是否有向左换道的车辆，如果有的话我就不向右换道，不然会冲突
                                    right_has_lc = 1                                                                          #lc_phase不能等于3因为3是右右右车道换到右右车道的车辆
                                    break

                    if surr_speed > 40 / 3.6:
                        p_intent = 1 - 1 / ( 1 + math.exp(0.855 * self.dec_duration - 0.710))
                        if p_intent < 0.5:
                            return
                        if self.current_lane.lmark == 'dashed' and (lc_direction >= 0 and self.leftfront.gap + self.leftfront.speed > self.front.gap + self.front.speed
                                and self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb):
                            self.lc_flag = 1
                        elif self.current_lane.rmark == 'dashed' and (lc_direction <= 0 and self.rightfront.gap + self.rightfront.speed > self.front.gap + self.front.speed
                                and self.rightfront.gap > dis_gap_rf and self.rightbehind.gap > dis_gap_rb) and not right_has_lc:
                            if self.current_link.id == -3 and self.current_lane.id == -2 and self.position > 50:  # 上游不要让最内侧车辆换到外侧
                                return
                            self.lc_flag = 2
                    else:
                        right_gx = 3.5
                        if self.rightfront.is_movable() and self.rightfront.gap < 5:
                            right_gx = min(right_gx, self.world_y - self.rightfront.vehicle.world_y)
                        if self.rightbehind.is_movable() and self.rightbehind.gap < 5:
                            right_gx = min(right_gx, self.world_y - self.rightbehind.vehicle.world_y)
                        if self.front.is_movable() and self.front.gap < 0:
                            right_gx = min(right_gx, abs(self.world_y - self.front.vehicle.world_y))
                        if self.behind.is_movable() and self.behind.gap < 0:
                            right_gx = min(right_gx, abs(self.world_y - self.behind.vehicle.world_y))
                        pl_intent = 1 - 1 / ( 1 + math.exp(0.716 * self.position / self.current_lane.length + 0.039 * self.speed - 0.178 * self.acc + 0.091 * self.dec_duration + \
                                                          0.223 * (self.leftfront.speed - self.speed) - 0.46 * right_gx - 0.852))
                        pr_intent = 1 - 1 / (1 + math.exp(0.716 * self.position / self.current_lane.length + 0.039 * self.speed - 0.178 * self.acc + 0.091 * self.dec_duration + \
                            0.223 * (self.speed - self.rightfront.speed - 0.46 * 3.5 - 0.852)))
                        if pl_intent > 0.2 and self.current_lane.lmark == 'dashed' and (lc_direction >= 0 and self.leftfront.gap + self.leftfront.speed > self.front.gap + self.front.speed
                                and self.leftfront.gap > dis_gap_lf and self.leftbehind.gap > dis_gap_lb):
                            self.lc_flag = 1
                        elif pr_intent > 0.5 and self.current_lane.rmark == 'dashed' and (lc_direction <= 0 and self.rightfront.gap + self.rightfront.speed > self.front.gap + self.front.speed
                                and self.rightfront.gap > dis_gap_rf and self.rightbehind.gap > dis_gap_rb) and not right_has_lc:
                            if self.current_link.id == -3 and self.current_lane.id == -2 and self.position > 50:  # 上游不要让最内侧车辆换到外侧
                                return
                            self.lc_flag = 2

    def get_adjacent_lane(self):
        if self.lc_flag == 1:
            return self.current_lane.llane
        elif self.lc_flag == 2:
            return self.current_lane.rlane

    def lc(self):#前提：前车不处于换道状态，本车位于link上，且距离link末端大于10m
        if self.is_ramp_veh and  self.end_merge and (self.front.gap > self.leftfront.gap + 8 or self.front.speed > self.leftfront.speed + 2.5) and self.position < self.end_merge_pos * self.current_lane.length:
            self.late_lc_yes = 1
            return
        self.late_lc_yes = 0
        if self.rest_length() < self.mlc_dist and ((self.para_drive and self.position > self.current_lane.length * 0.8) or (self.ar_lc and self.position > self.current_lane.length * 0.3)) and self.is_ramp_veh:
            self.lc_flag = 1
            self.not_traj = 1
            return
        if (self.front.is_movable() and self.front.vehicle.lc_flag != 0 and self.front.vehicle.lc_phase != 3) or (self.path_order == 0 and self.position < self.length) or \
                (self.current_lane in self.desired_lane_lst and self.behind.is_movable() and self.behind.vehicle.lc_flag != 0 and self.behind.vehicle.lc_phase != 3):
            return
        if not self.has_mlc_desire and (self.rest_length() < 10): # or (self.current_link.id == -2 and self.current_lane.id > -3 and self.rest_length() < 15)
            return
        if self.is_ramp_veh:
            lanes2change = 1
        else:
            lanes2change = self.get_lanes2change()
            # MLC一共需要跨越lanes2change条车道
        #判断换道的关键间隙
        if lanes2change and self.rest_length() < self.mlc_dist:#如果有强制换道需求
            # MLC一共需要跨越lanes2change条车道
            self.get_mlc_flag(lanes2change)
        elif self.is_shoulder_veh and self.position > self.current_lane.length * 0.8 \
            and ((self.rightfront and self.rightfront.gap < 2) or (self.rightbehind and self.rightbehind.gap < 2)) and self.coop_lc: #路肩车道协作向内换道
            self.flc = 0
            self.get_dlc_flag_SH(1)
        elif (self.current_link.id == -3) and self.left_lc: #汇入段最内侧从车道只允许换入不允许换出
            self.get_dlc_flag_SH(2)
        else: #如果没有强制换道需求,             #没有前车 或者 车辆排队时 或 前车刚换进来， 不要让后车换道
            if not self.front.is_movable() or (self.speed < 3 and self.front.gap < 3 and abs(self.lateral_offset) < 0.3) or self.front.vehicle.time_2last_lc < 0.5 \
                   or (self.chain_lc_dec == 1 and self.position > self.current_lane.length * 0.85): #减少横向任意性换道在汇入区末端的数量
                return
            self.get_dlc_flag_SH(0)
        if self.lc_flag != 0:
            self.adj_lane = self.get_adjacent_lane()
            self.lc_phase = 1
            self.motiv_time = 0
            self.get_lc_traj()

    def get_lc_traj(self): #换道轨迹计算，对应于文档6.1换道执行
        lc_dist = self.cal_lc_dist()
        if lc_dist < self.length:
            self.die()
            self.clear_lc()
            return
        end_position = self.position + lc_dist
        end_current_lane = self.adj_lane
        [end_x, end_y, end_heading] = Roads.lanepos2_worldxy(end_current_lane, end_position)
        if not end_x:
            self.die()
            return
        self.lc_traj = self.BezierCurve(self.world_x, self.world_y, end_x, end_y, self.heading, end_heading)

    def cal_lc_dist_with_speed(self):
        if self.speed < 20/3.6: #如果剩余车道长度足够，根据当前车速计算换道轨迹长度
            lc_dist = 2 * self.length
        elif self.speed < 30/3.6:
            lc_dist = 4 * self.length
        elif self.speed < 40/3.6:
            lc_dist = 6 * self.length
        else:
            lc_dist = 8 * self.length
        return lc_dist

    def cal_lc_dist(self): #计算换道距离
        lc_dist = self.cal_lc_dist_with_speed()
        self.adj_lane = self.get_adjacent_lane()
        lanes2change = self.get_lanes2change()
        if abs(lanes2change) > 1: #如果当前车道剩余距离有限，
            lc_dist = min(lc_dist, (self.adj_lane.length - 1 - self.position)/abs(lanes2change))
        else:
            lc_dist = min(lc_dist, self.adj_lane.length - 1 - self.position)
        return lc_dist

    def get_adjacent_lane(self):
        if self.lc_flag == 1:
            return self.current_lane.llane
        elif self.lc_flag == 2:
            return self.current_lane.rlane

    def BezierCurve(self, start_px, start_py, end_px, end_py, start_heading, end_heading): #根据起终点坐标、方向角生成一段贝塞尔曲线
        # fragment_count = np.sqrt((start_px - end_px) ** 2 + (start_py - end_py) ** 2) / 0.1
        fragment_count = 20
        t = np.linspace(0, 1, num=int(fragment_count))
        x1 = start_px * 2.0 / 3 + end_px * 1.0 / 3
        x2 = start_px * 1.0 / 3 + end_px * 2.0 / 3
        y1 = start_py * 2.0 / 3 + end_py * 1.0 / 3
        y2 = start_py * 1.0 / 3 + end_py * 2.0 / 3
        p1_x = (y1 - start_py - math.tan(start_heading + math.pi / 2) * x1 + math.tan(start_heading) * start_px) / \
               (math.tan(start_heading) - math.tan(start_heading + math.pi / 2))
        p1_y = math.tan(start_heading) * (p1_x - start_px) + start_py
        p2_x = (y2 - end_py - math.tan(end_heading + math.pi / 2) * x2 + math.tan(end_heading) * end_px) / \
               (math.tan(end_heading) - math.tan(end_heading + math.pi / 2))
        p2_y = math.tan(end_heading) * (p2_x - end_px) + end_py
        Bx = start_px * (1 - t) ** 3 + 3 * p1_x * t * (1 -t) ** 2 + 3 * p2_x * t ** 2 * (1 -t) + end_px * t ** 3
        By = start_py * (1 - t) ** 3 + 3 * p1_y * t * (1 - t) ** 2 + 3 * p2_y * t ** 2 * (1 - t) + end_py * t ** 3
        return (Bx.tolist(),By.tolist())

    def normal_cf(self): #对应于文档6.1.1
        if self.front.is_movable() and (self.current_link.id == -3): #并行车辆不要跟并行的前车（否则会急减速，没有并行），要跟前前车
            if self.front.gap < 1 and ((self.front.vehicle.not_traj and self.front.vehicle.lc_phase == 3) or (self.not_traj and self.lc_phase == 3)):
                self.front.vehicle.find_front()
                front2_veh = self.front.vehicle.front  # 前方排队
                self.acc = min(self.acc, self.IDM(front2_veh.speed, front2_veh.dist + self.front.gap))
                return
        if self.front.is_movable() and self.front.vehicle.current_link.id == -14 and self.front.gap < 3: #在合流终点处，让合流后车慢慢减速，不然会直接减速到零
            self.acc = min(self.acc, max(self.IDM(self.front.speed, self.front.gap), -self.comfort_dec/2.0))
            return
        if self.front.is_movable() and self.front.vehicle.lc_phase == 3 and self.is_shoulder_veh: #汇入车辆在匝道上
            self.acc = min(self.acc, self.IDM(max(3, self.front.speed), self.front.gap))
        self.acc = min(self.acc, self.IDM(self.front.speed, self.front.gap))

    def turn_taking(self):
        if self.is_shoulder_veh and self.position > self.current_lane.length * 0.8 :#and self.position < 136
            stop_position = 136
            if self.rightfront.vehicle and self.rightfront.speed < 3 and self.rightfront.gap > 0 and self.rightfront.gap < 3 \
                    and (self.rightfront.vehicle.lc_phase == 4 or self.rightfront.vehicle.lc_phase == 1) and not self.rightfront.vehicle.para_drive:
                turn_taking_acc = self.IDM(self.rightfront.speed, stop_position - self.position - self.rightfront.length)
                self.acc = min(self.acc, turn_taking_acc)

    def follow_mergeveh(self): #路肩车道车辆协作减速
        if self.current_link.id == -3 and self.merge_veh.is_movable() and self.rest_length() > 30 and len(self.be_cutin)<1 and self.merge_veh.dist < self.merge_veh.speed * 3: #前车不处于cutin状态
            tclc = 2
            gl = self.merge_veh.dist + self.merge_veh.speed * tclc + self.merge_veh.acc * tclc * tclc / 2 -  self.speed * tclc + self.acc * tclc * tclc / 2
            tclc_ar = 4
            gl_ac = self.merge_veh.dist + self.merge_veh.speed * tclc_ar + self.merge_veh.acc * tclc_ar * tclc_ar / 2 - self.speed * tclc_ar + self.acc * tclc_ar * tclc_ar / 2
            if self.coop_arlc and self.merge_veh.not_traj and (self.speed < self.merge_veh.speed or (
                    self.speed >= self.merge_veh.speed and gl_ac > self.net_dist )): #+ self.cf_hwd * (self.speed - self.merge_veh.speed)
                merge_acc = max(self.IDM(self.merge_veh.speed, self.merge_veh.gap), -self.comfort_dec)
                self.acc = min(self.acc, merge_acc)
                self.coop_arlc_yes = 1
                self.coop_yes = 0
            elif self.coop_dec and self.merge_veh.gap > self.net_dist and (self.speed < self.merge_veh.speed or (self.speed >= self.merge_veh.speed and gl > self. net_dist + self.cf_hwd * (self.speed - self.merge_veh.speed))):
                merge_acc = max(self.IDM(self.merge_veh.speed, self.merge_veh.gap), -self.comfort_dec)
                self.acc = min(self.acc, merge_acc)
                self.coop_yes = 1
                self.coop_arlc_yes = 0
            else:
                self.coop_yes = 0
                self.coop_arlc_yes = 0
        else:
            self.coop_yes = 0
            self.coop_arlc_yes = 0

    def follow_in_arlc(self):
        if self.lc_flag and self.lc_phase == 5 and self.position < 120: #TODO:可能逻辑与并行式汇入冲突
            if self.lc_flag == 1:
                front_speed = self.leftfront.speed
                gap = self.leftfront.gap
            else:
                front_speed = self.rightfront.speed
                gap = self.rightfront.gap
            vehicle_vir_acc = self.IDM(front_speed, gap)
            self.acc = min(self.acc, vehicle_vir_acc)

    def lateral_frict0(self):
        danger_disty = 2.29
        comfort_disty = 2.95
        [rclosest_x, rclosest_y, rclosest_v] = self.get_rclosest_xyv()
        [lclosest_x, lclosest_y, lclosest_v] = self.get_lclosest_xyv()
        acc_dec = 0
        if self.front.gap > self.speed * 2 and not self.current_lane.rlane and self.world_y - rclosest_y > comfort_disty:
            return
        if self.speed > 3 and ((self.world_y - rclosest_y < danger_disty and rclosest_x > self.world_x) or (lclosest_y - self.world_y < danger_disty and lclosest_x > self.world_x)):
            self.acc = - self.comfort_dec / 2.0
            return
        elif self.current_lane.rlane and self.world_y - rclosest_y < comfort_disty and self.speed > 3 and rclosest_x > self.world_x:
            acc_dec = max(self.comfort_dec / 4.0, 0.5 * abs(self.speed - rclosest_v) / abs(self.world_y - rclosest_y - 1.8))
        elif self.current_lane.llane and lclosest_y - self.world_y < comfort_disty and self.speed > 6 and lclosest_x > self.world_x:
            acc_dec = max(self.comfort_dec / 2.0, 0.5 * abs(self.speed - lclosest_v) / abs(self.world_y - lclosest_y - 1.8))
        self.acc -= acc_dec

    def update(self, sim_time):
        self.sim_time = sim_time
        self.get_desiredlanes_by_network()
        self.find_nextlane()
        self.find_surr_vehs()
        self.acc = self.max_acc
        self.normal_cf()
        if self.turn_take and self.lc_flag == 0:
            self.turn_taking()
        if not self.lc_flag or (self.lc_flag and (self.lc_phase == 4 or (self.lc_phase == 5 and self.danger == 1))): #self.lc_phase == 1 or
            self.follow_by_mlc()
        self.follow_in_lc()
        self.follow_lc_veh()
        self.follow_mergeveh()
        self.follow_in_arlc()
        if self.current_link.id == -3:
            self.lateral_frict0()
        self.acc_limit()
        if self.lc_flag == 0 and self.on_link and self.time_2last_lc > 1.0:
            self.lc()
        self.update_pos()

    def acc_limit(self):
        if self.overreact:
            acc_max = -8
        else:
            acc_max = -6
        if self.acc < acc_max:
            self.acc = acc_max

        if self.acc < 0:
            self.dec_duration += self.sim_step
        else:
            self.dec_duration = 0

    def update_pos(self): #用运动学公式更新车辆的速度、位置；并更新车辆的坐标、航向角，如果处于换道状态更新换道阶段
        if self.lc_flag:
            self.time_2last_lc = 0
        else:
            self.time_2last_lc += self.sim_step

        self.speed +=  self.sim_step * self.acc
        if self.speed < 1e-6:
            self.speed = 0
            self.static_time += self.sim_step
        else:
            self.static_time = 0
        if self.static_time > 30:
            self.para_drive = 1
        if self.static_time > 60:
            self.die()
        self_position = self.position + self.sim_step * self.speed
        if self.current_link.id == -5 and self.position < 50 and self_position > 50:
            self.graph.productivity += self.speed

        self.position = self_position

        if (self.position > self.current_lane.length) or (self.current_link.id == -4 and self.position > 480):
            if self.at_last_link():
                self.die()
                return
            if self.next_lane is None:
                if self.lc_flag == 1:
                    self.next_lane = self.current_lane.llane.out_lane_lst[0]
                elif self.lc_flag == 2:
                    self.next_lane = self.current_lane.rlane.out_lane_lst[0]
            self.position -= self.current_lane.length
            self.current_lane = self.next_lane
            try:
                self.current_link = self.current_lane.ownner
            except:
                pass
            self.path_order += 1

        [x, y, heading] = Roads.lanepos2_worldxy(self.current_lane, self.position)
        if self.lc_flag: #如果处于换道状态，需要更新lc_phase,lc_flag,以及换道状态下的world_x,world_y,heading
            if self.not_traj:
                self.update_lateral_pos(x, y)
            else:
                self.update_lc_phasexy(x, y, heading)
            self.lateral_offset = self.world_y - y
        else:
            new_world_y = self.lateral_move0(y)
            self.lateral_offset = self.world_y - y
            self.world_x = x
            self.world_y = new_world_y
            self.heading = heading

    def lateral_move0(self, y): #TODO:仅适用该地图
        if self.current_link.id != -3 and self.current_link.id != -8: #self.current_link.id != -2 and self.current_link.id != -17: #不设置这个条件上匝道那段斜的道路会有问题
            return y
        # 主线车辆受外侧车辆压迫的横向偏移
        [_, rlateral_y, _] = self.get_rclosest_xyv()
        rlateral_thd = 2.7 + 0.54 * (rlateral_y - 3.5 * (4 + self.current_lane.id) - 6.82 +1.75) + 6.82 + 3.5 * (self.current_lane.id + 4) - 1.75
        [_, llateral_y, _] = self.get_lclosest_xyv()
        llateral_thd = 2.7 + 0.54 * (self.world_y - 3.5 * (4 + self.current_lane.id) - 6.82 +1.75) + 6.82 + 3.5 * (self.current_lane.id + 5) - 1.75

        if rlateral_thd >= self.world_y and llateral_thd > llateral_y:
            r_gapy = rlateral_thd - self.world_y
            l_gapy = llateral_thd - llateral_y
            if r_gapy <= l_gapy:
                new_world_y = max(self.world_y - min(0.05, self.speed * 0.06), y)
            else:
                new_world_y = min(self.world_y + min(0.05, self.speed * 0.06), rlateral_thd)
        elif rlateral_thd >= self.world_y:
            new_world_y = min(self.world_y + min(0.05, self.speed * 0.06), rlateral_thd)
        elif llateral_thd > llateral_y:
            new_world_y = max(self.world_y - min(0.05, self.speed * 0.06), y)
        else:
            return y

        return new_world_y

    def get_rclosest_xyv(self):
        if not self.current_lane.rlane:
            return [self.world_x, self.world_y + (self.current_lane.width + 0.3) * math.sin(self.heading - math.pi / 2.0), 0]

        lateral_x = -math.inf
        lateral_y = -math.inf
        lateral_v = 0
        rlane_vehs = self.graph.get_vehicles_in_lane(self.current_lane.rlane)
        for veh in rlane_vehs:
            if abs(veh.world_x - self.world_x) < self.length and veh.world_y > lateral_y and veh.world_y < self.world_y:
                lateral_x = veh.world_x
                lateral_y = veh.world_y
                lateral_v = veh.speed

        current_lane_vehs = self.graph.get_vehicles_in_lane(self.current_lane)
        for veh in current_lane_vehs:
            if veh.id == self.id:
                continue
            if abs(veh.world_x - self.world_x) < self.length and veh.world_y > lateral_y and veh.world_y < self.world_y:
                lateral_x = veh.world_x
                lateral_y = veh.world_y
                lateral_v = veh.speed

        return [lateral_x, lateral_y, lateral_v]

    def get_lclosest_xyv(self):
        if not self.current_lane.llane:
            return [self.world_x, self.world_y + (self.current_lane.width + 0.3) * math.sin(self.heading + math.pi / 2.0), 0]

        lateral_x = math.inf
        lateral_y = math.inf
        lateral_v = 0
        llane_vehs = self.graph.get_vehicles_in_lane(self.current_lane.llane)
        for veh in llane_vehs:
            if abs(veh.world_x - self.world_x) < self.length and veh.world_y < lateral_y and veh.world_y > self.world_y:
                lateral_x = veh.world_x
                lateral_y = veh.world_y
                lateral_v = veh.speed

        current_lane_vehs = self.graph.get_vehicles_in_lane(self.current_lane)
        for veh in current_lane_vehs:
            if veh.id == self.id:
                continue
            if abs(veh.world_x - self.world_x) < self.length and veh.world_y < lateral_y and veh.world_y > self.world_y:
                lateral_x = veh.world_x
                lateral_y = veh.world_y
                lateral_v = veh.speed

        return [lateral_x, lateral_y, lateral_v]

    def die(self):
        self.status = 0

    def get_drt_dist(self, drt, vf, vb):
        dec_max = 6
        return drt*vb-(vf*vf-vb*vb)/2/dec_max

    def update_lateral_pos(self, x, y): #汇入车辆的主动回应+并行式汇入横向偏移
        # 确定路肩车道左后车的纵向舒适空间
        drt_danger = 0.65
        drt_safe = 1.26
        dist_danger = min(self.get_drt_dist(drt_danger, self.speed, self.leftbehind.speed), self.get_drt_dist(drt_danger, self.leftfront.speed, self.leftbehind.speed))
        dist_safe =  min(self.get_drt_dist(drt_safe, self.speed, self.leftbehind.speed), self.get_drt_dist(drt_safe, self.leftfront.speed, self.leftbehind.speed))
        if self.lc_phase == 3:
            self_danger = max(self.net_dist, self.get_drt_dist(drt_danger, self.front.speed, self.speed))
        else:
            self_danger = max(self.net_dist, self.get_drt_dist(drt_danger, self.leftfront.speed, self.speed))
        # 横向危险空间
        danger_disty = 2.29
        [_, lateral_y, _] = self.get_lclosest_xyv()
        danger_y = 0
        if lateral_y - self.world_y < danger_disty:
            danger_y = 1

        if self.leftbehind.dist < dist_danger or (self.lc_phase != 3 and self.leftfront.gap < self_danger) or (self.lc_phase == 3 and self.front.gap < self_danger):
            self.danger = 1
        elif self.leftbehind.dist < dist_safe:
            self.danger = 2
        else:
            self.danger = 3

        # 确定后车的让行意图
        lb_yield = 0
        if self.leftbehind.is_movable():
            if self.leftbehind.vehicle.acc < -1 or self.leftbehind.vehicle.acc < self.acc:
                lb_yield = 1

        # 计算横向加加速度
        ay = 0
        if danger_y == 0 and self.para_drive and self.is_ramp_veh and self.position > self.current_lane.length * 0.7:
            ay = self.get_ay()
            self.para_yes = 1
        elif danger_y == 1 or (self.danger == 1 or (self.danger == 2 and lb_yield == 0)) and self.lc_phase != 3: #
            ay = (0 - self.speedy) / self.sim_step
            self.para_yes = 0
        else:
            ay = self.get_ay()
            self.para_yes = 0
        ay = min(ay, min(3, max(self.speed * 0.6, 0.5)))
        self.speedy = max(0, self.speedy + ay * self.sim_step)
        # self.speedy = max(0, min(self.speedy, self.speed * math.tan(15/360*2*math.pi)))
        world_y = max(self.world_y + self.speedy * self.sim_step + 0.5 * ay * self.sim_step * self.sim_step, self.world_y)
        new_heading = math.atan((world_y - self.world_y) / (x - self.world_x)) - math.pi * ((x - self.world_x) < 0)
        if abs(new_heading - self.heading) < 0.15:
            self.heading = min(new_heading, 0.25)
        self.world_y = world_y
        self.world_x = x
        if self.world_y < 1.65: #7.82
            new_lc_phase = 4
        elif self.world_y < 3.4 : # 8.57
            new_lc_phase = 5
        else:
            new_lc_phase = 3

        if self.world_y >= 4.83:  # 表示换道结束 10
            if self.lc_phase != 3:
                self.die()
                print('lc error1, lc finished when lc phase is:', self.lc_phase)
                return
            self.clear_lc()
            return

        if self.lc_phase == 5 and new_lc_phase == 3: #换道阶段从第2阶段变到第3阶段时，要将车辆从原车道移到目标车道上去
            objective_lane = self.get_adjacent_lane()
            if self.current_link.id == -14: #TODO:在汇入区没有完成汇入主线，会直接跳到中间车道
                objective_lane = self.current_lane
            if objective_lane.add_length[1] - objective_lane.add_length[0] > 0.2: #对于shpfile路网需要精细化车道坐标
                [lane_xy, lane_direct,lane_add_length]  = Roads.detail_xy(objective_lane.xy)
            else:
                lane_add_length = objective_lane.add_length
                lane_xy = objective_lane.xy
                lane_direct = objective_lane.direct

            new_addlen = lane_add_length + [self.position]
            new_addlen.sort()
            new_idx = new_addlen.index(self.position)
            idxs = max(0, new_idx - 50)
            idxe = min(new_idx + 50, len(lane_add_length) - 1)
            [sublane_rank, sub_point] = self.get_cross_segment(self.world_x, self.world_y, lane_direct[idxs:idxe],
                                               [lane_xy[0][idxs:idxe],lane_xy[1][idxs:idxe]])
            sublane_rank +=  idxs
            vehicle_vir_position = lane_add_length[sublane_rank]   + \
                                   math.sqrt( (sub_point[0] - lane_xy[0][sublane_rank]) ** 2 + ( sub_point[1] - lane_xy[1][sublane_rank]) ** 2)
            self.current_lane = objective_lane
            self.position = vehicle_vir_position
        self.lc_phase = new_lc_phase

    def get_ay(self):
        k1 = 0.112
        k2 = 0.0007
        k3 = 1.317
        k4 = -0.072
        q = -(self.world_y - 3.4) / 1.75
        if self.lc_phase == 3:
            gl = self.front.gap
            gf = self.speed * 2
            delta_vl = self.front.speed - self.speed
            delta_vf = self.speed
        else:
            gl = self.leftfront.gap
            gf = self.leftbehind.gap
            delta_vl = self.leftfront.speed - self.speed
            delta_vf = self.speed - self.leftbehind.speed
        ay = (k1 * gl + k2 * gf + k3 * delta_vl + k4 * delta_vf) * q * q * q
        if abs(q) < 0.5 and abs(self.speedy) < 0.5 or (self.lc_phase == 3 and ay <= 0):
            ay = 0.5
        return ay

    def update_lc_phasexy(self, x, y, heading): #车辆处于换道状态时，更新车辆坐标是在贝塞尔曲线上更新坐标点的
        [traj_rank, traj_x, traj_y, traj_heading] = self.get_closest_point(x, y, self.lc_traj)
        self.traj_rank = traj_rank
        if traj_rank is None:  # 表示换道结束
            if self.lc_phase != 3:
                self.die()
                print('lc error2, lc finished when lc phase is:', self.lc_phase)
                return
            self.clear_lc()
            self.world_x = x
            self.world_y = y
            self.heading = heading
            self.traj_rank = 0
            return

        self.world_x = traj_x
        self.world_y = traj_y
        self.heading = traj_heading

        if self.overdec_ratio == 1:
            if traj_rank < len(self.lc_traj[0]) * (0.4 - 0.01 * self.speed):
                new_lc_phase = 1
            elif traj_rank < len(self.lc_traj[0]) * 0.45:
                new_lc_phase = 2
            else:
                new_lc_phase = 3
        else:
            if traj_rank < len(self.lc_traj[0]) * (0.3 - 0.01 * self.speed):
                new_lc_phase = 1
            elif traj_rank < len(self.lc_traj[0]) * 0.4:
                new_lc_phase = 2
            else:
                new_lc_phase = 3

        if (self.lc_phase == 1 or self.lc_phase == 2) and new_lc_phase == 3: #换道阶段从第2阶段变到第3阶段时，要将车辆从原车道移到目标车道上去
            objective_lane = self.get_adjacent_lane()

            if objective_lane.add_length[1] - objective_lane.add_length[0] > 0.2: #对于shpfile路网需要精细化车道坐标
                [lane_xy, lane_direct,lane_add_length]  = Roads.detail_xy(objective_lane.xy)
            else:
                lane_add_length = objective_lane.add_length
                lane_xy = objective_lane.xy
                lane_direct = objective_lane.direct

            new_addlen = lane_add_length + [self.position]
            new_addlen.sort()
            new_idx = new_addlen.index(self.position)
            idxs = max(0, new_idx - 50)
            idxe = min(new_idx + 50, len(lane_add_length) - 1)

            [sublane_rank, sub_point] = self.get_cross_segment(traj_x, traj_y, lane_direct[idxs:idxe],
                                               [lane_xy[0][idxs:idxe],lane_xy[1][idxs:idxe]])
            sublane_rank +=  idxs

            vehicle_vir_position = lane_add_length[sublane_rank]   + \
                                   math.sqrt( (sub_point[0] - lane_xy[0][sublane_rank]) ** 2 + ( sub_point[1] - lane_xy[1][sublane_rank]) ** 2)
            self.current_lane = objective_lane
            self.position = vehicle_vir_position
        self.lc_phase = new_lc_phase

    #给出经过某一点(xo,y0)角度为angle的直线，计算跟xy_lst的交点
    def get_cross_segment(self, x0, y0, angle, xy_lst): #点坐标， 点所在线段角度， 被找交点的多段线
        temp_len = 10
        for i in range(len(xy_lst[0]) - 1):
            if isinstance(angle, list) == 0:
                temp_angle = angle
            else:
                temp_angle = angle[i]
            inter_point = is_intersect(x0 + temp_len * math.cos(temp_angle - math.pi/2), y0 + temp_len * math.sin(temp_angle - math.pi/2), \
                                            x0 + temp_len * math.cos(temp_angle + math.pi/2), y0 + temp_len * math.sin(temp_angle + math.pi/2), \
                                            xy_lst[0][i], xy_lst[1][i], xy_lst[0][i + 1], xy_lst[1][i + 1])
            if inter_point!=[]:
                return [i, inter_point]
            if i == len(xy_lst[0]) - 2: #有时候会出现因为角度问题，找不到交点，直接给最近点 or？再找一次角度不同
                [sublane_rank, sub_pointx, sub_pointy, sublane_heading] = self.get_closest_point(x0, y0, xy_lst)
                return [sublane_rank, [sub_pointx, sub_pointy]]

    def get_closest_point(self, x0, y0, xy_lst): #找xy_lst中距离（xo,y0）点最近的点
        if len(xy_lst) < 2:
            print('xy_lst',xy_lst)
        dist = list(map(lambda dx,dy: np.sqrt((dx-x0)**2 + (dy-y0)**2), xy_lst[0], xy_lst[1]))
        min = reduce(lambda d1, d2: d1 < d2 and d1 or d2, dist)
        rank = dist.index(min)
        pointx = xy_lst[0][rank] #不精确，还有偏移没有加上 先这样//TODO
        pointy = xy_lst[1][rank]
        if rank == len(xy_lst[0])-1:
            heading = math.atan((xy_lst[1][rank] - xy_lst[1][rank-1]) / (xy_lst[0][rank] - xy_lst[0][rank-1])) - \
                      math.pi * ((xy_lst[0][rank] - xy_lst[0][rank-1]) < 0)
            return [None, pointx, pointy, heading]
        try:
            heading = math.atan((xy_lst[1][rank + 1] - xy_lst[1][rank]) / (xy_lst[0][rank + 1] - xy_lst[0][rank])) - \
                          math.pi * ((xy_lst[0][rank + 1] - xy_lst[0][rank]) < 0)
        except:
            pass
        return [rank, pointx, pointy, heading]

    def clear_lc(self): #清除换道状态
        self.lc_traj = []
        self.lc_flag = 0
        self.lc_phase = 0
        self.speedy = 0
        self.not_traj = 0
        self.lateral_offset = 0
        self.adj_lane = None

class VehicleType:
    def __init__(self, veh_type):
        self.value = veh_type
        if veh_type == 'car':
            self.length = 4.5
            self.width = 2.0
            self.max_acc = 4.0
            self.desired_speed = random.randint(20,30)
            self.comfort_dec = 2.0
        elif veh_type == 'truck':
            self.length = 10.5
            self.width = 2.6
            self.max_acc = 2.5
            self.desired_speed = random.randint(15,20)
            self.comfort_dec = 1.8

class surr_vehicle(): #周围车辆类
    def __init__(self, speed=0, length=0):
        self.dist = 200
        self.gap = 200
        self.vehicle = None
        self.speed = speed
        self.length = length
        self.acc = 0
        self.not_traj = 0
        self.id = 0

    def is_movable(self):
        return self.vehicle is not None

    def update(self, v, d, g):
        self.vehicle = v
        self.gap = g
        self.dist = d
        self.speed = v.speed
        self.length = v.length
        self.acc = v.acc
        self.not_traj = v.not_traj
        self.id = self.vehicle.id


def is_intersect(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y): #判断线段P1P2和线段P3P4是否相交，如果是返回交点
    m = (p2_x - p1_x) * (p3_y - p1_y) - (p3_x - p1_x) * (p2_y - p1_y)
    n = (p2_x - p1_x) * (p4_y - p1_y) - (p4_x - p1_x) * (p2_y - p1_y)
    p = (p4_x - p3_x) * (p1_y - p3_y) - (p1_x - p3_x) * (p4_y - p3_y)
    q = (p4_x - p3_x) * (p2_y - p3_y) - (p2_x - p3_x) * (p4_y - p3_y)
    if m * n <= 0 and p * q <= 0:
        if abs(p2_x - p1_x) < 1e-3:
            x = p1_x
            y = (p4_y - p3_y) / (p4_x - p3_x) * (x - p3_x) + p3_y
        elif abs(p4_x - p3_x) < 1e-3:
            x = p3_x
            y = (p2_y - p1_y) / (p2_x - p1_x) * (x - p1_x) + p1_y
        else:
            x = ((p4_y - p3_y) / (p4_x - p3_x) * p3_x - (p2_y - p1_y) / (p2_x - p1_x) * p1_x + p1_y - p3_y) / \
                ((p4_y - p3_y) / (p4_x - p3_x) - (p2_y - p1_y) / (p2_x - p1_x))
            y = (p2_y - p1_y) / (p2_x - p1_x) * (x - p1_x) + p1_y
        return [x, y]
    else:
        return []



