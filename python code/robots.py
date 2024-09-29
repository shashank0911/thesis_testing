from midlevel import *
from toplevel import *
from lowlevel import *
import numpy as np
import time

class Robot:
    def __init__(self, id, x0):
        self.id = id
        self.x0 = x0 
        # self.t_vals = t_vals
        self.midSystem = None
        self.midControl = None
        self.topSystem = None
        self.lowSystem = None
        self.lowControl = None
        self.mid_layer_time = None
        self.low_layer_time = None
        # self.mid_layer_loop_time = None
        # self.low_layer_loop_time = None

class Robots:
    def __init__(self, idList, x0List, t_vals, cellDict):
        self.idList = idList
        self.numRobots = len(idList)
        self.x0List = x0List
        self.t_vals = t_vals
        self.robotList = []
        self.add_robots()
        self.cellList = cellDict

    def add_robots(self):
        for i in range(self.numRobots):
            id = self.idList[i]
            x0 = self.x0List[i, :]
            self.robotList.append(Robot(id, x0))
        
    def initialise_mid_level_system(self, dim):
        T = self.t_vals.T_mid
        for robot in self.robotList:
            robot.midSystem = MidLevelSystem(robot.id, robot.x0, T, dim)

    def initialise_mid_level_control(self, weight, velocityConstraints, inputConstraints):
        for robot in self.robotList:
            robot.midControl = MidLevelControl(weight, velocityConstraints, inputConstraints, robot.midSystem)

    def initialise_low_level_system(self):
        t_vals = self.t_vals
        for robot in self.robotList:
            x = robot.x0[0]
            y = robot.x0[1]
            # v = np.sqrt(robot.x0[2]**2 + robot.x0[3]**2)
            # Change later if according to original value
            theta = 0
            robot.lowSystem = LowLevelSystem(x, y, theta, t_vals)

    def initialise_low_level_control(self, Kp, Kd, sigma):
        for robot in self.robotList:
            robot.lowControl = LowLevelControl(Kp, Kd, sigma)

    def find_high_level_input(self, sequencePair):
        currentCell = self.cellList[sequencePair[0]]
        terminalCell = self.cellList[sequencePair[1]]
        return (currentCell, terminalCell)
    

