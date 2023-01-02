from matplotlib import animation
import matplotlib.pyplot as plt
import random
import D1xodrRoads_HXL as D1xodrRoads
import D1Vehicle_view_HXL as D1Vehicles
from GUI_paramset import App
from tkinter import *
import xml.dom.minidom
import time


def Rand_Hdw(flow):  # 根据车辆到达模型确定计算车辆到达间隔，对于文本3.2车辆到达模型
    # flow为小时流量
    # 负指数分布
    if flow == 0:
        return 1e6
    # lamda = flow / 3600.0
    # rand_hdw = random.random()
    # hdw = (- math.log(rand_hdw)) / lamda
    # 均匀到达
    hdw = 3600.0 / flow #+ random.uniform(-0.5, 0.5)
    return hdw

class Sim:
    def __init__(self):
        self.timestamp = 0.0
        self.simulation_step = 0.2
        self.fig, self.ax = plt.subplots()
        plt.xlim(-30, 30)
        plt.ylim(-30, 30)
        plt.axis('equal')
        self.obj_list = {}
        self.obj_num = 0

        path_set = [[-1, -4, [3 * i for i in [1000, 1000, 1000, 1340, 1160, 1120, 1240, 1240, 1140, 1240, 1220, 1140, 1160, 1220, 1200, 1180, 1100, 1120, 1000]]],
                    [-5, -4, [800, 800, 800, 960, 1920, 660, 1620, 1840, 1500, 1920, 1800, 1320, 1400, 1980, 2280, 1140, 1860, 2640, 2000]]]
        roadD1 = D1xodrRoads.Graph(len(path_set[0][2]), 15)  # 初始化xodr路网
        xodr = xml.dom.minidom.parse('hongxuroad.xodr')
        D1xodrRoads.create_road(roadD1, xodr)  # 从xodr文件读取道路信息

        params = App(path_set, root)
        self.sim_duration = float(params.sim_duration.get())
        self.truck_rate = float(params.truck_rate.get())
        self.agsv_rate = float(params.driver_agsv.get())
        self.cnsv_rate = float(params.driver_cnsv.get())
        self.veh_params = params.params
        self.d2_on = int(params.d2_on.get())
        roadD1.create_path(params.od_matrix)  # 根据路网拓扑结构搜索完整路径
        for path in roadD1.path_map.values():
            path.interval = Rand_Hdw(path.flow[0])  # 初始化各条路径的到达时间间隔
        self.roadD1 = roadD1
        self.roadD1.q = 0
        self.roadD1.v = 0

        self.roadD1.draw_border(self.ax)
        self.physical_graph = {}
        self.logical_graph = {}
        self.traj_dict = {}
        self.res_time = 0
        # root.mainloop()

    def BornNewCar(self):
        for path in self.roadD1.path_map.values():
            if self.timestamp - path.last_time + self.res_time < path.interval:
                continue
            else:
                newcar_num = min(2, round((self.timestamp - path.last_time) / path.interval))
                self.res_time = (self.timestamp - path.last_time + self.res_time) - path.interval * newcar_num
                for _ in range(0, newcar_num):
                    self.BornNewSingleCar(path)

    def BornNewSingleCar(self, path):
        path_id_lst = self.roadD1.find_path(path.oid, path.did)
        if self.obj_num%50 == 0:
            print('born a new car! id:', self.obj_num, '  sim_time:', self.timestamp, '  total vehs num:',
              len(self.roadD1.vehicles), ' flow rate (veh/h/ln):', path.flow[int(self.timestamp)]/2)
        plot_vehicle, = self.ax.plot([], [], 'r')
        random_truck = random.random()
        random_driver = random.random()
        if random_truck < self.truck_rate:
            if random_driver < self.agsv_rate:
                params = self.veh_params.params_truck_agsv
            elif random_driver > 1 - self.cnsv_rate:
                params = self.veh_params.params_truck_cnsv
            else:
                params = self.veh_params.params_truck_norm
            D1Vehicles.Vehicle(self.roadD1, self.obj_num, 'truck', path_id_lst, plot_vehicle, params,
                               self.simulation_step)  # 如果时间到达计算的时间间隔就添加一辆该路径的车辆
        else:
            if random_driver < self.agsv_rate:
                params = self.veh_params.params_car_agsv
            elif random_driver > 1 - self.cnsv_rate:
                params = self.veh_params.params_car_cnsv
            else:
                params = self.veh_params.params_car_norm
            D1Vehicles.Vehicle(self.roadD1, self.obj_num, 'car', path_id_lst, plot_vehicle, params,
                               self.simulation_step)
        path.last_time = self.timestamp
        path.interval = Rand_Hdw(path.flow[int(self.timestamp)])  # 生成车辆后重新计算新的时间间隔
        self.obj_num += 1


    def thread_upd(self, vehicles):
        for vehicle in vehicles:
            vehicle.update(self.timestamp)
            self.roadD1.update_veh(vehicle)
            vehicle.draw()

        # end = time.time()
        # print("子进程ID号:%d，time：%f, method: %d" % (os.getpid(), end-start, k))  # os.getpid()进程ID
        # print("子进程ID号:%d，run：%d, time: %f, method: %d" % (os.getpid(), len(vehicles), end - start, k))  # os.getpid()进程ID

    def UpdateVehD1(self):
        del_list = []
        vehs = []
        veh_group = []
        group_num = 200
        veh_count = 0

        for vehicle in self.roadD1.vehicles.values():
            if veh_count > group_num:
                veh_group.append(vehs)
                vehs = []
                veh_count = 0
            veh_count += 1
            if vehicle.status == 0:
                del_list.append(vehicle.id)
            elif vehicle.status == 1:
                vehs.append(vehicle)
        veh_group.append(vehs)

        for vehs in veh_group:
            self.thread_upd(vehs)

        for del_veh_id in del_list:
            self.roadD1.vehicles[del_veh_id].plot_vehicle.set_data([], [])
            del self.roadD1.vehicles[del_veh_id]


    def Update(self, no_use):  #
        self.timestamp += self.simulation_step
        self.BornNewCar()
        time_s = time.time()
        self.UpdateVehD1()
        time_e = time.time()

        if len(self.roadD1.vehicles) > 0 and self.timestamp % 10 == 0:
            print('sim_time1: ', self.timestamp, ' update cost: ', time_e - time_s,
                  ' avg veh cost: ', (time_e - time_s) / len(self.roadD1.vehicles), ' veh nums: ',
                  len(self.roadD1.vehicles))

    def Run(self):
        self.BornNewCar()
        ani = animation.FuncAnimation(self.fig, self.Update, interval=50, blit=False)
        plt.show()

root = Tk()
Sim().Run()