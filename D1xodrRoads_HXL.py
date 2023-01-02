#coding=utf-8
#Python2.7
import xml.dom.minidom
import math
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib import animation
from socket import *
from time import ctime
import struct
import copy
from collections import defaultdict
from functools import reduce
import csv
from queue import Queue

class VPath: #车辆路径
    def __init__(self):
        self.oid = []
        self.did = []
        self.flow = []
        self.interval = 1e6
        self.last_time = 0

class Lane(object):
    def __init__(self):
        self.id             = -1 #int，车道编号
        self.link_id        = -1 #所属的Link编号=self.ownner.id
        self.type           = 'driving' #string，车道类型，暂时没用保留
        self.width          = [] #float，车道宽度，暂时不知道车道宽度是否会发生变化(m)
        self.speed_limit    = 80/3.6 #float，车道限速（m/s）
        self.llid           = 0 #int，左侧车道编号
        self.rlid           = 0 #int，右侧车道编号
        self.lborder        = [] #左侧边线采样坐标点
        self.rborder        = [] #右侧边线采样坐标点
        self.lmark          = 'dashed' #string，左侧车道线类型，虚线可变道实线不可变道
        self.rmark          = 'dashed' #string，右侧车道线类型，虚线可变道实线不可变道
        self.llane          = None #Object Lane，左侧车道
        self.rlane          = None #Object Lane，右侧车道
        self.in_lane_id_lst = [] #Int list，上游车道对象编号集合
        self.out_lane_id_lst= [] #Int list，上游车道对象编号集合
        self.out_lane_lst   = [] #Object Lane list，上游车道对象集合
        self.in_lane_lst    = [] #Object Lane list，上游车道对象集合
        self.xy             = [] #Float list，车道中心线采样坐标点
        self.direct         = [] #Float list，车道中心线相邻采样坐标点连线的方向角
        self.add_length     = [] #Float list，车道中心线采样坐标点距离起点的距离
        self.ownner         = None #Object Link，所属的路段对象
        self.index = -1

    @property
    def length(self):
        return self.add_length[-1]

    @property
    def index_id(self):
        return self.link_id * 100 + self.id

    def cut_lane(self, position, up_len, down_len):
        sub_lane = Lane()
        up_pos = position - up_len
        down_pos = position + down_len
        add_uppos = [i for i in self.add_length]
        add_uppos.append(up_pos)
        add_downpos = [i for i in self.add_length]
        add_downpos.append(down_pos)
        add_uppos.sort()
        add_downpos.sort()
        uppos_index = add_uppos.index(up_pos)
        downpos_index = add_downpos.index(down_pos)
        sub_lane.add_length = self.add_length[max(0, uppos_index-1):downpos_index] #position not in add_length, -1
        x = self.xy[0][max(0, uppos_index-1):downpos_index]
        y = self.xy[1][max(0, uppos_index-1):downpos_index]
        sub_lane.xy = [x,y]
        sub_lane.id = self.id
        sub_lane.link_id = self.link_id
        return sub_lane

    def set_ownner(self, link):
        self.ownner = link

    def add_inlane(self, lane):
        if lane not in self.in_lane_lst:
            self.in_lane_lst.append(lane)

    def add_outlane(self, lane):
        if lane not in self.out_lane_lst:
            self.out_lane_lst.append(lane)

    def is_driving_lane(self):
        return self.type == 'driving'

    def is_conn(self):
        return self.ownner.junction_id != -1

    def x(self):
        return self.xy[0]

    def y(self):
        return self.xy[1]

class Link():
    def __init__(self):
        self.id             = -1 #int，路段编号（缺省-1）
        self.junction_id    = -1 #int，交叉口编号（缺省-1表示普通路段）
        self.lane_lst       = [] #Object Lane list，包含的车道对象列表
        self.in_link_lst    = [] #Object Link list，包含的上游路段对象列表
        self.out_link_lst   = [] #Object Link list，包含的下游路段对象列表

    def add_lane(self, lane):
        self.lane_lst.append(lane)

    def iter_lane(self):
        for l in self.lane_lst:
            yield l

    def add_inlink(self, link):
        if link not in self.in_link_lst:
            self.in_link_lst.append(link)

    def add_outlink(self, link):
        if link not in self.out_link_lst:
            self.out_link_lst.append(link)

# 根据车道中心线坐标计算行驶方向和线长度序列
def get_line_feature(xy):
    xy = np.array(xy)
    # n为中心点个数，2为x,y坐标值
    x_prior = xy[0][:-1]
    y_prior = xy[1][:-1]
    x_post = xy[0][1:]
    y_post = xy[1][1:]
    # 根据前后中心点坐标计算【行驶方向】
    dx = x_post - x_prior
    dy = y_post - y_prior
    direction = list(map(lambda d: d > 0 and d or d + 2 * np.pi, np.arctan2(dy, dx))) #沿x轴方向逆时针转过的角度

    length = np.sqrt(dx ** 2 + dy ** 2)
    for i in range(len(length) - 1):
        length[i + 1] += length[i]
    return direction, length.tolist()

class Graph:
    def __init__(self, mtx_time, mtx_detector):
        self.link_map = {}
        self.lane_map = {}
        self.vehicles = {}
        self.path_map = {}
        self.replace_linkmap = {}
        self.replace_lanemap = {}
        self.speed_mtx = np.zeros((mtx_time, mtx_detector))
        self.vehs_mtx = np.zeros((mtx_time, mtx_detector))
        self.productivity = 0

    def add_link(self, link):
        if link.id not in self.link_map:
            self.link_map[link.id] = link
        else:
            raise Exception("Link is existed ?")

    def add_lane(self, lane):
        if lane.index_id not in self.lane_map.keys():
            self.lane_map[lane.index_id] = lane
        else:
            raise Exception("Link is existed ?")

    def get_lane_inbetween(self,lane1, lane2): #get the common lane which are the outlane2 of conn1 and the inlane2 of conn2
        for one1 in lane1.out_lane_id_lst:
            for one2 in lane2.in_lane_id_lst:
                if one1 == one2:
                    return self.get_lane(one1)

    def get_lane(self, lane_id):
        return self.lane_map[lane_id]

    def get_vehicles_in_link(self, link):
        if link is None:
            return []
        vehs = []
        for veh in self.vehicles.values():
            if veh.current_link.id == link.id:
                vehs.append(veh)
        return vehs

    def get_vehicles_in_front_link(self, link0, link1, link2):
        links = [link0, link1, link2]
        link_ids = []
        for link in links:
            if link is None:
                link_ids.append(None)
            else:
                link_ids.append(link.id)
        vehs0 = []
        vehs1 = []
        vehs2 = []
        for veh in self.vehicles.values():
            if veh.current_link.id in link_ids:
                link_index = link_ids.index(veh.current_link.id)
                if link_index == 0:
                    vehs0.append(veh)
                elif link_index == 1:
                    vehs1.append(veh)
                elif link_index == 2:
                    vehs2.append(veh)
        return vehs0, vehs1, vehs2

    def get_vehicles_in_lanes(self, llane, rlane):
        lvehs = []
        rvehs = []
        for veh in self.vehicles.values():
            if llane and veh.current_lane.index_id == llane.index_id:
                lvehs.append(veh)
            if rlane and veh.current_lane.index_id == rlane.index_id:
                rvehs.append(veh)
        return [lvehs, rvehs]

    def get_vehicles_in_lane(self, lane):
        if lane is None:
            return []
        vehs = []
        for veh in self.vehicles.values():
            if veh.current_lane.index_id == lane.index_id:
                vehs.append(veh)
        return vehs

    def get_lane_border(self):
        for link in self.link_map.values():
            for lane in link.iter_lane():
                lborder_x = []
                lborder_y = []
                rborder_x = []
                rborder_y = []
                point_len = len(lane.direct)
                for i in range(0, point_len):
                    lborder_x.append(lane.xy[0][i] + lane.width[i] / 2.0 * math.cos(lane.direct[i] + math.pi / 2.0))
                    lborder_y.append(lane.xy[1][i] + lane.width[i] / 2.0 * math.sin(lane.direct[i] + math.pi / 2.0))
                    rborder_x.append(lane.xy[0][i] + lane.width[i] / 2.0 * math.cos(lane.direct[i] - math.pi / 2.0))
                    rborder_y.append(lane.xy[1][i] + lane.width[i] / 2.0 * math.sin(lane.direct[i] - math.pi / 2.0))
                lane.lborder = [lborder_x] + [lborder_y]
                lane.rborder = [rborder_x] + [rborder_y]

    def build_topo(self):
        self.get_lane_border()
        for link in self.link_map.values():
            lane_id_lst = [l.id for l in link.lane_lst]
            llane_id_lst = [l for l in lane_id_lst if l > 0]
            rlane_id_lst = [l for l in lane_id_lst if l < 0]
            for lane in link.iter_lane():
                lane.set_ownner(link)
                if lane.id in llane_id_lst:
                    if lane.id != min(llane_id_lst):
                        lane.llid = lane.id - 1
                    if lane.id != max(llane_id_lst):
                        lane.rlid = lane.id + 1
                elif lane.id in rlane_id_lst:
                    if lane.id != min(rlane_id_lst):
                        lane.rlid = lane.id - 1
                    if lane.id != max(rlane_id_lst):
                        lane.llid = lane.id + 1
                # self.add_lane(lane)
                # self.lane_map[lane.index_id] = lane #补充lane_map,暂时将lane_id设为唯一的 TODO:此时lane_map中的lane并非具备全部信息

        for link in self.link_map.values():
            for lane in link.iter_lane():
                lane.llane = self.lane_map.get(lane.link_id * 100 + lane.llid, None)
                lane.rlane = self.lane_map.get(lane.link_id * 100 + lane.rlid, None)
                for lid in lane.out_lane_id_lst:
                    outlane = self.lane_map.get(lid)
                    if outlane is not None:
                        lane.out_lane_lst.append(outlane)
                    else:
                        lane.out_lane_id_lst.remove(lid)
                for lid in lane.in_lane_id_lst:
                    inlane = self.lane_map.get(lid)
                    if inlane is not None:
                        lane.in_lane_lst.append(inlane)
                    else:
                        lane.in_lane_id_lst.remove(lid)
                # lane.out_lane_lst = [self.lane_map.get(lid) for lid in lane.out_lane_id_lst]
                # lane.in_lane_lst = [self.lane_map.get(lid) for lid in lane.in_lane_id_lst]

                for in_lane_id in lane.in_lane_id_lst:
                    inlane = self.lane_map.get(in_lane_id)
                    if inlane is not None:
                        if inlane.ownner is None:
                            continue
                        lane.add_inlane(inlane)
                        link.add_inlink(inlane.ownner)
                for out_lane_id in lane.out_lane_id_lst[::-1]:
                    outlane = self.lane_map.get(out_lane_id)
                    if outlane is not None:
                        if outlane.ownner is None: #TODO:某些道路为driving连接border导致border没有ownner
                            continue
                        lane.add_outlane(outlane)
                        link.add_outlink(outlane.ownner)
                    else:
                        lane_index = lane.out_lane_id_lst.index(out_lane_id)
                        lane.out_lane_id_lst.pop(lane_index)
                # self.lane_map[lane.index_id] = lane
                # self.link_map[lane.link_id] = link

    def pair2series(self, pairs, series):
        if len(pairs) == 1:
            series.append(pairs[0])
            return series
        link_pair1 = pairs[0]           #每次取第一个元素，如果有上游/下游link就补充上，并放回；如果均没有上下游link，放到series中作为单独的序列
        pair_count2 = 0
        pair_count0 = 0
        pair_append2 = []
        pair_append0 = []


        for link_pair2 in pairs[1:]:    #先查找下游link
            if link_pair1[-1] == link_pair2[0]:
                pair_count2 += 1
                pair_append2.append(pairs.index(link_pair2))
        if pair_count2 == 1:            #如果只有一个下游就合并到上游link上，删除pairs中的下游关系；如果没有或有多个，就不合并
            [link_pair1.append(x) for x in pairs[pair_append2[0]][1:]]
            pairs.pop(pair_append2[0])

        for link_pair0 in pairs[1:]:    #再查找上游link
            if link_pair1[0] == link_pair0[-1]:
                pair_count0 += 1
                pair_append0.append(pairs.index(link_pair0))
        if pair_count0 == 1:            #如果只有一个下游就合并到上游link上，删除pairs中的下游关系；如果没有或有多个，就不合并
            [pairs[pair_append0[0]].append(x) for x in link_pair1[1:]]
            pairs.remove(link_pair1)

        if pair_count0 != 1 and pair_count2 != 1:   #如果上下游均没有可合并的link，就提出来放到series中
            series.append(link_pair1)
            pairs.remove(link_pair1)

        if pairs:
            return self.pair2series(pairs, series)
        else:
            return series

    def get_sub_lane_to_outlane(self, link, dest_lane_lst):
        ret_lanes = []
        all_lanes = list(link.iter_lane())
        for l in all_lanes:
            if len(set(l.out_lane_lst).intersection(dest_lane_lst)) > 0:
                ret_lanes.append(l)
        return ret_lanes

    def get_path(self, origin, destination):
        paths = []
        path = [origin]
        outlink_id_lst_pre = [origin]
        paths.append(path)
        path_length = 1 #路径包含link的最大数量
        while outlink_id_lst_pre and path_length < 20:
            nextlink_id_lst = outlink_id_lst_pre
            paths_new = []
            outlink_id_lst_pre=[]
            for link_id in nextlink_id_lst:
                outlink_id_lst = []
                nextlink = self.get_link(link_id)
                if not nextlink.out_link_lst:
                    continue
                else:
                    for ll in nextlink.out_link_lst:
                        if ll.id not in outlink_id_lst and ll.id != origin:  # 避免绕一圈
                            outlink_id_lst.append(ll.id)
                            outlink_id_lst_pre.append(ll.id)
                for path in paths:
                    if path[-1] == link_id:
                        for lid in outlink_id_lst:
                            if lid in path:
                                if path not in paths_new:
                                    paths_new.append(path)
                                continue
                            path_new = path + [lid]
                            if path_new not in paths_new:
                                paths_new.append(path_new)
            if paths_new:
                # paths_new = list(np.unique(paths_new))
                paths = paths_new
            # print('path length: ', path_length)
            path_length += 1

            for pathx in paths:
                if pathx[-1] == destination:
                    return pathx

        print('unavailabile OD: ', origin, '  to ', destination)
        raise Exception("No avilabile path ?")

    def find_path(self, origin, destination):
        for path in self.path_map.values():
            if path.oid == origin and path.did == destination:
                return path.path_id_lst

        raise Exception("Invalid path ?")

    def create_path(self, path_set): #生成graph.path_map
        for path in path_set:
            # print('path: ',path)
            if path[0] in self.replace_linkmap.keys():
                path[0] = self.replace_linkmap[path[0]]  #如果路段发生过替换就进行替换
            if path[1] in self.replace_linkmap.keys():
                path[1] = self.replace_linkmap[path[1]]  #如果路段发生过替换就进行替换
            self.add_path(path[0], path[1], path[2])

    def add_path(self, oid, did, flow):
        if oid not in self.path_map:
            path = VPath()
            self.path_map[oid * did] = path
            path.oid = oid
            path.did = did
            for i in range(0, len(flow)):
                path.flow = path.flow + list(np.linspace(flow[i], flow[i], 60))
            path.interval = [3600.0/x for x in path.flow]
            path.path_id_lst = self.get_path(oid, did)
        else:
            raise Exception("Same path is existed ?")

    def update_veh(self, car): #更新路网动态信息：车辆、信号配时
        self.vehicles[car.id] = car

    def get_pos(self, x, y):
        if random.random() > 0.5:
            pp = 1
        else:
            pp = -1
        xytext=(x+(4+int(random.random()*10))*pp, y+(3+int(random.random()*10)*pp))
        return xytext

    def draw_border(self, ax):
        cnames = {'DimGray': '#696969',
                  'Black': '#000000',
                  'DarkBlue': '#00008B',
                  'DarkSlateBlue': '#483D8B'}
        for link in self.link_map.values():
            line_color = random.choice(list(cnames))
            for lane in link.iter_lane():
                if lane.llane is None:
                    ax.plot(lane.lborder[0][6:-1:10], lane.lborder[1][6:-1:10], color=cnames[line_color], linewidth=0.8)  # 画左侧车道线
                else:
                    ax.plot(lane.lborder[0][6:-1:10], lane.lborder[1][6:-1:10], color=cnames[line_color],linestyle='--', linewidth=0.8)  # 画左侧车道线
                if lane.rlane is None:
                    if link.id == -17:
                        ax.plot(lane.rborder[0][6:-1:10], lane.rborder[1][6:-1:10], color=cnames[line_color],linestyle='--',linewidth=0.8)  # 画右侧车道线
                    else:
                        ax.plot(lane.rborder[0][6:-1:10], lane.rborder[1][6:-1:10], color=cnames[line_color], linewidth=0.8)  # 画右侧车道线
        ax.plot(list(np.linspace(160,169,40)), list(np.linspace(-1.6,-1.3,8))+list(np.linspace(-1.3,-0.8,9))[1:]+list(np.linspace(-0.8,0.0,9))[1:]+list(np.linspace(0.0,0.5,9))[1:]+list(np.linspace(0.5,0.8,9))[1:], \
                color=cnames[line_color], linewidth=0.8)  # 画右侧车道线

    def get_link(self, link_id):
        if link_id in self.link_map.keys():
            return self.link_map[link_id]
        else:
            return self.link_map[self.replace_linkmap[link_id]]

    def conn_lanes_of_nextlink(self, lane, link): #获取下一个link上与当前lane有连接的车道
        return [outlane for outlane in lane.out_lane_lst if outlane.ownner is link]

def get_rect(x, y, width, length, angle):
    rect = np.array([(0, width / 2.0), (0, -width / 2.0), (-length, -width / 2.0), (-length, width / 2.0), (0, width / 2.0)])
    theta = angle
    R = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
    offset = np.array([x, y])
    transformed_rect = np.dot(rect, R) + offset
    return transformed_rect

def lanepos2_worldxy(lane, position):  # 根据车辆当前状态更新车辆x,y坐标信息
    if lane is None:
        aaa = 1
    if position > lane.length:
        print('pos is larger than lane length! longer for ', position - lane.length ,' m')
        position = lane.length
        # if position < lane.length + 0.5:
        #     position = lane.length
    temp_len = lane.add_length + [position]
    temp_len.sort()
    temp_rank = temp_len.index(position)
    if temp_rank >= len(lane.direct):
        # print('this is imposible')
        temp_rank -= 1
    heading = lane.direct[temp_rank]
    x = lane.xy[0][temp_rank] + np.cos(heading) * (position - lane.add_length[temp_rank])
    y = lane.xy[1][temp_rank] + np.sin(heading) * (position - lane.add_length[temp_rank])
    return [x, y, heading]

def get_Refline(geometry):
    Rclinex = []
    Rcliney = []
    Rdirect = []
    Radd_length = []
    for Rline in geometry:
        step_length = 0.2  # TODO: # 以0.1m作为步长
        temp_Rclinex = []
        temp_Rcliney = []
        temp_Rlength = 0
        Rstartx = float(Rline.getAttribute('x'))
        Rstarty = float(Rline.getAttribute('y'))
        Rheading = float(Rline.getAttribute('hdg'))
        Rlength = float(Rline.getAttribute('length'))
        if Rlength < 1e-3:
            continue
        temp_Rclinex.append(Rstartx)
        temp_Rcliney.append(Rstarty)
        Rdirect.append(Rheading)
        Radd_length.append(float(Rline.getAttribute('s')))
        Rline_index = geometry.index(Rline)
        if Rline_index < len(geometry) - 1:
            nextRline = geometry[Rline_index + 1]
            nextx = float(nextRline.getAttribute('x'))
            nexty = float(nextRline.getAttribute('y'))
        if Rline.getElementsByTagName('line'):
            while temp_Rlength + step_length < Rlength:
                temp_Rclinex.append(temp_Rclinex[-1] + step_length * math.cos(Rheading))
                temp_Rcliney.append(temp_Rcliney[-1] + step_length * math.sin(Rheading))
                temp_Rlength += step_length
                Rdirect.append(Rheading)
                Radd_length.append(Radd_length[-1] + step_length)
        elif Rline.getElementsByTagName('arc'):
            close2nextp = 0
            arc = Rline.getElementsByTagName('arc')
            curvature = float(arc[0].getAttribute('curvature'))
            delta_alpha = step_length * curvature
            temp_heading = Rheading
            while temp_Rlength + step_length < Rlength:
                #######
                # 用于平滑弧线/螺旋线尾端的累积误差，用直线连接目标点
                if Rline_index < len(geometry) - 1:
                    dist2nextp = math.sqrt((temp_Rclinex[-1] - nextx) ** 2 + (temp_Rcliney[-1] - nexty) ** 2)
                    # if dist2nextp < 0.2:
                    #     break
                    if dist2nextp < 1.0:
                        temp_heading = np.arctan2(nexty - temp_Rcliney[-1], nextx - temp_Rclinex[-1])
                        # if temp_heading < 0:
                        #     temp_heading += math.pi * 2
                        delta_alpha = 0
                        if close2nextp == 0:
                            Rlength = temp_Rlength + dist2nextp
                            close2nextp = 1
                #######
                temp_Rclinex.append(temp_Rclinex[-1] + step_length * math.cos(temp_heading))
                temp_Rcliney.append(temp_Rcliney[-1] + step_length * math.sin(temp_heading))
                temp_Rlength += step_length
                Rdirect.append(temp_heading)
                Radd_length.append(Radd_length[-1] + step_length)
                temp_heading += delta_alpha
        elif Rline.getElementsByTagName('spiral'):  # TODO:连接处做了平滑处理:是由于车道宽度导致的不平滑
            close2nextp = 0
            spiral = Rline.getElementsByTagName('spiral')
            curvStart = float(spiral[0].getAttribute('curvStart'))
            curvEnd = float(spiral[0].getAttribute('curvEnd'))
            temp_heading = Rheading
            while temp_Rlength + step_length < Rlength:
                curvature = (temp_Rlength + 0.5 * step_length) / Rlength * (curvEnd - curvStart) + curvStart
                delta_alpha = step_length * curvature
                if Rline_index < len(geometry) - 1:
                    dist2nextp = math.sqrt((temp_Rclinex[-1] - nextx) ** 2 + (temp_Rcliney[-1] - nexty) ** 2)
                    if dist2nextp < 1.0:
                        temp_heading = np.arctan2(nexty - temp_Rcliney[-1], nextx - temp_Rclinex[-1])
                        # if temp_heading < 0:
                        #     temp_heading += math.pi * 2
                        delta_alpha = 0
                        if close2nextp == 0:
                            Rlength = temp_Rlength + dist2nextp
                            close2nextp = 1
                temp_Rclinex.append(temp_Rclinex[-1] + step_length * math.cos(temp_heading))  # 以0.1m作为步长
                temp_Rcliney.append(temp_Rcliney[-1] + step_length * math.sin(temp_heading))

                temp_Rlength += step_length
                Rdirect.append(temp_heading)
                Radd_length.append(Radd_length[-1] + step_length)
                temp_heading += delta_alpha
        elif Rline.getElementsByTagName('poly3'):
            pass
        elif Rline.getElementsByTagName('paramPoly3'):
            pass
        else:
            raise Exception("Unknown Geometry !!!")
        Rclinex = Rclinex + temp_Rclinex
        Rcliney = Rcliney + temp_Rcliney
    for i in range(1, len(Rclinex) - 1):
        if abs(Rcliney[i + 1] - Rcliney[i]) < 1e-6 and abs(Rclinex[i + 1] - Rclinex[i]) < 1e-6 and i > 0:  # 两个点很接近
            Rdirect[i] = Rdirect[i - 1]

    return Rclinex, Rcliney, Rdirect, Radd_length

def create_road(graph, xodr):
    #得到文档元素对象
    root = xodr.documentElement
    links = root.getElementsByTagName('road')
    for road in links:
        new_link = Link()
        new_link0 = Link()
        new_link.id = int(road.getAttribute('id'))
        new_link0.id = -int(road.getAttribute('id'))
        junction = int(road.getAttribute('junction'))
        new_link.junction_id = junction
        new_link0.junction_id = junction
        temp_link = road.getElementsByTagName('link')
        link_successor_id = None
        link_successor = temp_link[0].getElementsByTagName('successor')
        if link_successor and link_successor[0].getAttribute('elementType') == "road":
            link_successor_id = int(link_successor[0].getAttribute('elementId'))
        link_predecessor_id = None
        link_predecessor = temp_link[0].getElementsByTagName('predecessor')
        if link_predecessor and link_predecessor[0].getAttribute('elementType') == "road":
            link_predecessor_id = int(link_predecessor[0].getAttribute('elementId'))
        plan_view = road.getElementsByTagName('planView')
        geometry = plan_view[0].getElementsByTagName('geometry')
        [Rclinex, Rcliney, Rdirect, Radd_length] = get_Refline(geometry)
        temp_lanes = road.getElementsByTagName('lanes')
        laneSection = temp_lanes[0].getElementsByTagName('laneSection') #TODO：可能有多段section
        lanes = laneSection[0].getElementsByTagName('lane')
        lane_border_list = {}
        lane_width_list = {}
        for lane in lanes:
            new_lane = Lane()
            new_lane.id = int(lane.getAttribute('id')) #为了区分不同车道的情况
            if new_lane.id >= 0:
                new_lane.link_id = new_link.id
            else:
                new_lane.link_id = new_link0.id
            if new_lane.index_id in graph.lane_map.keys():
                new_lane = graph.get_lane(new_lane.index_id)
            else:
                graph.add_lane(new_lane)
            new_lane.type = lane.getAttribute('type')
            width = lane.getElementsByTagName('width')
            if not width:
                continue #如果没有width这个标签说明为地面标线，不是车道
            for k in range(0, len(width)):
                a = float(width[k].getAttribute('a'))
                b = float(width[k].getAttribute('b'))
                c = float(width[k].getAttribute('c'))
                d = float(width[k].getAttribute('d'))
                offset_pre = float(width[k].getAttribute('sOffset'))
                temp_alength = Radd_length + [offset_pre]
                temp_alength.sort()
                temp_index = temp_alength.index(offset_pre)
                roadMark = lane.getElementsByTagName('roadMark') #TODO：暂时没有考虑标线
                # m_width = float(roadMark[0].getAttribute('width'))
                temp_width = [a + b * (s - offset_pre) + c * (s - offset_pre) ** 2 + d * (s - offset_pre) ** 3 for s in Radd_length[temp_index:]]
                new_lane.width[temp_index:] = temp_width
            if new_lane.type != 'driving':
                # if len(lane_border_list) == 0:# 应该先计算中间车道的坐标点，再计算外侧车道
                #     Rclinex = [x + a * math.cos(h + np.sign(new_lane.id) * math.pi / 2.0) for (x, h) in zip(Rclinex, Rdirect)]
                #     Rcliney = [y + a * math.sin(h + np.sign(new_lane.id) * math.pi / 2.0) for (y, h) in zip(Rcliney, Rdirect)]
                lane_border_list[new_lane.id] = new_lane
                lane_width_list[new_lane.id] = new_lane.width
                continue
            speed = lane.getElementsByTagName('speed')
            if speed:
                manx_speed = float(speed[0].getAttribute('max')) #TODO：暂时只考虑单个限速标志
                new_lane.speed_limit = manx_speed
            lane_successor = lane.getElementsByTagName('successor')
            if lane_successor:
                lane_successor_id = int(lane_successor[0].getAttribute('id'))
                try:
                    link_successor_id0 = int(np.sign(lane_successor_id)) * link_successor_id
                    suc_id = link_successor_id0 * 100 + lane_successor_id
                    if suc_id in graph.lane_map.keys():
                        suc_lane = graph.get_lane(suc_id)
                    else:
                        suc_lane = Lane()
                        suc_lane.link_id = link_successor_id0
                        suc_lane.id = lane_successor_id
                        graph.add_lane(suc_lane)
                    if suc_id not in new_lane.out_lane_id_lst and new_lane.id < 0:
                        new_lane.out_lane_id_lst.append(suc_id) #目前lane_id_lst存的都是修正过的车道id
                    elif suc_id not in new_lane.in_lane_id_lst and new_lane.id > 0:
                        new_lane.in_lane_id_lst.append(suc_id)  # 目前lane_id_lst存的都是修正过的车道id
                    if new_lane.index_id not in suc_lane.in_lane_id_lst and new_lane.id < 0:
                        suc_lane.in_lane_id_lst.append(new_lane.index_id)
                    elif new_lane.index_id not in suc_lane.out_lane_id_lst and new_lane.id > 0:
                        suc_lane.out_lane_id_lst.append(new_lane.index_id)
                except:
                    pass
            lane_predecessor = lane.getElementsByTagName('predecessor')
            if lane_predecessor:
                lane_predecessor_id = int(lane_predecessor[0].getAttribute('id'))
                link_predecessor_id0 = int(np.sign(lane_predecessor_id)) * link_predecessor_id
                pre_id = link_predecessor_id0 * 100 + lane_predecessor_id
                if pre_id in graph.lane_map.keys():
                    pre_lane = graph.get_lane(pre_id)
                else:
                    pre_lane = Lane()
                    pre_lane.link_id = link_predecessor_id0
                    pre_lane.id = lane_predecessor_id
                    graph.add_lane(pre_lane)
                if pre_id not in new_lane.in_lane_id_lst and new_lane.id < 0:
                    new_lane.in_lane_id_lst.append(pre_id)
                elif pre_id not in new_lane.out_lane_id_lst and new_lane.id > 0:
                    new_lane.out_lane_id_lst.append(pre_id)
                if new_lane.index_id not in pre_lane.out_lane_id_lst and new_lane.id < 0:
                    pre_lane.out_lane_id_lst.append(new_lane.index_id)
                elif new_lane.index_id not in pre_lane.in_lane_id_lst and new_lane.id > 0:
                    pre_lane.in_lane_id_lst.append(new_lane.index_id)
            lane_border_list[new_lane.id] = new_lane
            lane_width_list[new_lane.id] = new_lane.width

        for lane_id, new_lane in sorted(lane_border_list.items()):
            if lane_id < 0:
                continue
            # if new_lane.type == 'driving' and new_lane.id == 1:
            #     Rclinex0 = Rclinex
            #     Rcliney0 = Rcliney
            # if new_lane.type != 'driving':
            #     Rclinex0 = [x + w * math.cos(h + np.sign(new_lane.id) * math.pi / 2.0) for (x, h, w) in zip(Rclinex, Rdirect, lane_width_list[lane_id])]
            #     Rcliney0 = [y + w * math.sin(h + np.sign(new_lane.id) * math.pi / 2.0) for (y, h, w) in zip(Rcliney, Rdirect, lane_width_list[lane_id])]
            if lane_id - 1 in lane_border_list.keys():
                clinex = [x + (w1 + w2) * 0.5 * math.cos(h + np.sign(new_lane.id) * math.pi / 2.0) for (x, h, w1, w2) in zip(lane_border_list[lane_id-1].xy[0], Rdirect, lane_width_list[lane_id], lane_width_list[lane_id-1])] #应该先计算中间车道的坐标点，再计算外侧车道
                cliney = [y + (w1 + w2) * 0.5 * math.sin(h + np.sign(new_lane.id) * math.pi / 2.0) for (y, h, w1, w2) in zip(lane_border_list[lane_id-1].xy[1], Rdirect, lane_width_list[lane_id], lane_width_list[lane_id-1])]
            else:
                clinex = [x +  w * 0.5 * math.cos(h + np.sign(new_lane.id) * math.pi / 2.0) for (x, h, w) in zip(Rclinex[::-1], Rdirect, lane_width_list[lane_id])]  # 应该先计算中间车道的坐标点，再计算外侧车道
                cliney = [y +  w * 0.5 * math.sin(h + np.sign(new_lane.id) * math.pi / 2.0) for (y, h, w) in zip(Rcliney[::-1], Rdirect, lane_width_list[lane_id])]
            new_lane.xy = [clinex, cliney]
            # new_lane.xy = [clinex[::-np.sign(new_lane.id)], cliney[::-np.sign(new_lane.id)]]  # 车道id为负的话，需要倒序xy坐标
            lane_border_list[new_lane.id] = new_lane
        for lane_id, new_lane in sorted(lane_border_list.items(), reverse=True):
            if lane_id > 0:
                continue
            # if new_lane.type == 'driving' and new_lane.id == -1:
            #     Rclinex1 = Rclinex
            #     Rcliney1 = Rcliney
            # if new_lane.type != 'driving':
            #     Rclinex1 = [x + w * math.cos(h + np.sign(new_lane.id) * math.pi / 2.0) for (x, h, w) in zip(Rclinex, Rdirect, lane_width_list[lane_id])]
            #     Rcliney1 = [y + w * math.sin(h + np.sign(new_lane.id) * math.pi / 2.0) for (y, h, w) in zip(Rcliney, Rdirect, lane_width_list[lane_id])]
            if lane_id + 1 in lane_border_list.keys():
                clinex = [x + (w1 + w2) * 0.5 * math.cos(h + np.sign(new_lane.id) * math.pi / 2.0) for (x, h, w1, w2) in zip(lane_border_list[lane_id+1].xy[0], Rdirect, lane_width_list[lane_id], lane_width_list[lane_id+1])] #应该先计算中间车道的坐标点，再计算外侧车道
                cliney = [y + (w1 + w2) * 0.5 * math.sin(h + np.sign(new_lane.id) * math.pi / 2.0) for (y, h, w1, w2) in zip(lane_border_list[lane_id+1].xy[1], Rdirect, lane_width_list[lane_id], lane_width_list[lane_id+1])]
            else:
                clinex = [x + w * 0.5 * math.cos(h + np.sign(new_lane.id) * math.pi / 2.0) for (x, h, w) in zip(Rclinex, Rdirect, lane_width_list[lane_id])]  # 应该先计算中间车道的坐标点，再计算外侧车道
                cliney = [y + w * 0.5 * math.sin(h + np.sign(new_lane.id) * math.pi / 2.0) for (y, h, w) in zip(Rcliney, Rdirect, lane_width_list[lane_id])]
            # new_lane.xy = [clinex[::-np.sign(new_lane.id)], cliney[::-np.sign(new_lane.id)]]  # 车道id为负的话，需要倒序xy坐标
            new_lane.xy = [clinex, cliney]
            lane_border_list[new_lane.id] = new_lane

        for lane_id, new_lane in lane_border_list.items():
            [new_lane.direct, new_lane.add_length] = get_line_feature(new_lane.xy)
            avg_width = np.mean(new_lane.width)
            if avg_width < 1 or new_lane.type != 'driving':
                continue
            # new_lane.type = new_link.type
            if lane_id > 0:
                new_link.lane_lst.append(new_lane)
            else:
                new_link0.lane_lst.append(new_lane)

        if new_link.lane_lst:
            new_link.lane_lst.sort(key=lambda x: x.id, reverse=True)
            graph.add_link(new_link)
        if new_link0.lane_lst:
            new_link0.lane_lst.sort(key=lambda x: x.id, reverse=False)
            graph.add_link(new_link0)

    junctions = root.getElementsByTagName('junction')
    for junction in junctions:
        junction_id = int(junction.getAttribute('id'))
        connections = junction.getElementsByTagName('connection')
        for connection in connections:
            incomingRoad_id = int(connection.getAttribute('incomingRoad'))
            connectingRoad_id = int(connection.getAttribute('connectingRoad'))
            laneLinks = connection.getElementsByTagName('laneLink')
            for laneLink in laneLinks:
                pre_id = int(laneLink.getAttribute('from'))
                suc_id = int(laneLink.getAttribute('to'))
                if pre_id > 0:
                    incomingRoad = graph.link_map[incomingRoad_id]
                else:
                    incomingRoad = graph.link_map[-incomingRoad_id]
                for lane in incomingRoad.lane_lst:
                    if lane.id == pre_id:
                        new_id = connectingRoad_id * 100 + suc_id
                        if new_id not in lane.out_lane_id_lst:
                            lane.out_lane_id_lst.append(new_id)
                if suc_id > 0:
                    connectingRoad = graph.link_map[connectingRoad_id]
                else:
                    connectingRoad = graph.link_map[-connectingRoad_id]
                for lane in connectingRoad.lane_lst:
                    if lane.id == suc_id:
                        new_id = incomingRoad_id * 100 + pre_id
                        if new_id not in lane.in_lane_id_lst:
                            lane.in_lane_id_lst.append(incomingRoad_id * 100 + pre_id)

    graph.build_topo()

    for link in graph.link_map.values():
        link.lane_lst.sort(key=lambda x: x.id, reverse=False)
        for lane in link.lane_lst:
            if isinstance(lane.width, list):
                lane.width = lane.width[0]


    for link in graph.link_map.values():
        if link.id > 0:
            link.lane_lst.sort(key=lambda x: x.id, reverse=False)
        else:
            link.lane_lst.sort(key=lambda x: x.id, reverse=True)
        for lane in link.lane_lst:
            lane.index = link.lane_lst.index(lane) + 1
        # for lane in link.lane_lst:
        #     if link.id > 0:
        #         lane.index = len(link.lane_lst) - link.lane_lst.index(lane)
        #     else:
        #         lane.index = link.lane_lst.index(lane) + 1

    return graph

# 根据车道中心线坐标计算行驶方向和线长度序列
def get_lane_feature(xy):
    xy = np.array(xy)
    # n为中心点个数，2为x,y坐标值
    x_prior = xy[0][:-1]
    y_prior = xy[1][:-1]
    x_post = xy[0][1:]
    y_post = xy[1][1:]
    # 根据前后中心点坐标计算【行驶方向】
    dx = x_post - x_prior
    dy = y_post - y_prior

    direction = list(map(lambda d: d > 0 and d or d + 2 * np.pi, np.arctan2(dy, dx)))

    length = np.sqrt(dx ** 2 + dy ** 2)
    length = length.tolist()
    for i in range(len(length) - 1):
        length[i + 1] += length[i]
    length.insert(0, 0)
    return direction, length

def read_csv(file_path):#从csv文件中读取数据
    file_path.encode('utf-8')
    file = open(file_path)
    file_reader = csv.reader(file)
    for line in file_reader:
        yield line

def detail_xy(xy): #将原车道中心线上少量的点加密为0.1m间隔的点
    [direct, add_length] = get_lane_feature(xy)
    dist_interval = 0.1
    new_xy = [[], []]
    new_direct = []
    new_add_len = [0]
    temp_length = dist_interval
    for k in range(0, len(xy[0]) - 1):
        new_xy[0].append(xy[0][k])
        new_xy[1].append(xy[1][k])
        new_add_len.append(temp_length)
        new_direct.append(direct[k])
        while temp_length < add_length[k + 1]:
            temp_length += dist_interval
            new_xy[0].append(new_xy[0][-1] + dist_interval * math.cos(direct[k]))
            new_xy[1].append(new_xy[1][-1] + dist_interval * math.sin(direct[k]))
            new_add_len.append(temp_length)
            new_direct.append(direct[k])
    return [new_xy, new_direct, new_add_len]