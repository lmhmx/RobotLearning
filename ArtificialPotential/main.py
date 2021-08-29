from warnings import showwarning
import numpy as np
import matplotlib.pyplot as plt
from numpy import random


# 地图标志类
class MapMarker:
    OBSTACLE = 300
    TAGERT = 180
    START = 200
    SPACE = 0
    ROBOT = 250
    ROBOT_TRACE = 60
    PLAN_TRACE = 100

# 能量计算类
class EnergyControl:
    def __init__(self,detectionR = 5) -> None:
        
        self.K_Out = 10
        self.K_in = -20
        self.Min_r = 0.1
        self.detectionR = detectionR
        self.MaxEnergy = 5*self.K_Out/((self.detectionR+1)*(self.detectionR+1))
        self.MinEnergy = 5*self.K_in/((self.detectionR+1)*(self.detectionR+1))

    def getEnergy(self,p1,p2,p1_marker,p2_marker):
        # p2 在 p1处产生的势能
        energy = 0
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]
        r = np.max([self.Min_r,np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))])
        if(p1_marker ==MapMarker.OBSTACLE):
            energy = self.MaxEnergy
        elif(p1_marker == MapMarker.TAGERT):
            energy = self.MinEnergy
        elif(p2_marker == MapMarker.SPACE):
            energy = 0
        elif(p2_marker == MapMarker.OBSTACLE):
            energy = self.K_Out/r
        elif(p2_marker == MapMarker.TAGERT):
            energy = self.K_in/r
        else:
            energy = 0
        return energy

# 机器人类
class MyRobot:
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.r = 5
        self.target = []
        self.EnergyCotrol = EnergyControl()
    def setTarget(self,target):
        self.target = target

    def moveByLocalMap(self,localMap):
        H = localMap.shape[0]
        W = localMap.shape[1]

        G_energy = np.zeros((H,W))
        # 局部地图中的机器人位置
        x_robot, y_robot = self.findRobotInMap(localMap)

        for y in range(localMap.shape[0]):
            for x in range(localMap.shape[1]):
                if(not self.isTargetInMap(localMap)):
                    G_energy[y][x] = self.EnergyCotrol.getEnergy(
                            [x-x_robot+self.x,y-y_robot+self.y],self.target,localMap[y][x],MapMarker.TAGERT)
                
                for yy in range(localMap.shape[0]):
                    for xx in range(localMap.shape[1]):
                        G_energy[y][x] = G_energy[y][x] + self.EnergyCotrol.getEnergy(
                                [x-x_robot+self.x,y-y_robot+self.y],[xx-x_robot+self.x,yy-y_robot+self.y],localMap[y][x],localMap[yy][xx])

        # 根据地图中的能量场找到路径'
        path = self.findPath(G_energy,localMap)
        self.x=path[0][0]
        self.y=path[0][1]
        return path


    def findPath(self,G_energy,localMap):
        path = []
        x_robot, y_robot = self.findRobotInMap(localMap)
        
        H = G_energy.shape[0]
        W = G_energy.shape[1]

        current = [x_robot,y_robot]
        while(1):
            x = current[0]
            y = current[1]
            hasSmall = False
            for i in [-1,0,1]:
                for j in [-1,0,1]:
                    if(x+i>=0 and x+i<W and y+j >=0 and y+j <H):
                        if(G_energy[y+j][x+i]<G_energy[current[1]][current[0]]):
                            current = [x+i,y+j]
                            hasSmall = True
            if(hasSmall):
                path.append(current)
            else:
                break
        # 将局部地图信息转换到全局信息中
        for item in path:
            item[0] = item[0]-x_robot+self.x
            item[1] = item[1]-y_robot+self.y
        return path

    def isTargetInMap(self,localMap):
        result = False
        for y in range(localMap.shape[0]):
            for x in range(localMap.shape[1]):
                if(localMap[y][x] == MapMarker.TAGERT):
                    return True
        return result

    def findRobotInMap(self,localMap):
        x_robot = -1
        y_robot = -1
        for y in range(localMap.shape[0]):
            for x in range(localMap.shape[1]):
                if(localMap[y][x] == MapMarker.ROBOT):
                    x_robot = x
                    y_robot = y
                    return x_robot,y_robot
        return x_robot,y_robot

# 地图类
class MyMap:
    def __init__(self,width=23,height=23) -> None:
        # 栅格化后的地图格点大小
        self.block_size = 0.1
        self.grid = np.zeros((width,height))
        self.targetInfo = []
        self.startInfo = []
        self.robotInfo = []
        self.obstacleInfo = []
        self.robotTrace = []
        self.robotPlanner = []
        self.EnergyCotrol = EnergyControl()
        self.robotR = 5
        
    def getAllObstacle(self)->list:
        return self.obstacleInfo
    
    def getTarget(self)->list:
        return self.targetInfo
    def getrobotPos(self)->list:
        return self.robotInfo
    def getW(self):
        return self.grid.shape[1]
    def getH(self):
        return self.grid.shape[0]
    def addObstacle(self,obstacle:list):
        for obs in obstacle:
            x=obs[0]
            y=obs[1]
            self.grid[y][x] = MapMarker.OBSTACLE
            self.obstacleInfo.append([x,y])

    def clearObstacle(self):
        for y in range(self.grid.shape[1]):
            for x in range(self.grid.shape[0]):
                if(self.grid[y][x] == MapMarker.OBSTACLE):
                    self.grid[y][x] = MapMarker.SPACE
        self.obstacleInfo=[]

    def setRobotR(self,r):
        self.robotR = r
        
    def setObstacle(self,obstacle:list):
        self.clearObstacle()
        self.addObstacle(obstacle)

    def setStart(self,start:list):
        x=start[0]
        y=start[1]
        self.grid[y][x] = MapMarker.START
        self.startInfo=[x,y]
    def setTarget(self,target:list):
        x=target[0]
        y=target[1]
        self.grid[y][x] = MapMarker.TAGERT
        self.targetInfo=[x,y]
    def setRobot(self,robot:MyRobot):
        if(len(self.robotInfo)!=0):
            x = self.robotInfo[0]
            y = self.robotInfo[1]
            self.grid[y][x] = MapMarker.SPACE
        x = robot.x
        y = robot.y
        self.grid[y][x] = MapMarker.ROBOT
        self.robotInfo=[x,y]
        self.robotTrace.append([x,y])
    def setRobotPlan(self,planner):
        self.robotPlanner = planner

    def show(self):
        showimg = self.generateShowImage()
        plt.imshow(showimg)

        plt.show()
    def isTargetInMapRange(self):
        x_r = self.robotInfo[0]
        y_r = self.robotInfo[1]

        x_t = self.targetInfo[0]
        y_t = self.targetInfo[1]
        if(np.abs(x_r-x_t)>self.robotR or np.abs(y_r-y_t)>self.robotR):
            return False
        return True
    def get_G_energy(self):
        H = self.grid.shape[0]
        W = self.grid.shape[1]

        

        G_energy = np.zeros((H,W))
        

        for y in range(self.grid.shape[0]):
            for x in range(self.grid.shape[1]):
                if(not self.isTargetInMapRange()):
                    G_energy[y][x] = self.EnergyCotrol.getEnergy(
                            [x,y],self.targetInfo,self.grid[y][x],MapMarker.TAGERT)
                
                for yy in range(-self.robotR,self.robotR+1):
                    for xx in range(-self.robotR,self.robotR+1):
                        if(x+xx>=0 and x+xx<W and y+yy>=0 and y+yy <H):
                            G_energy[y][x] = G_energy[y][x] + self.EnergyCotrol.getEnergy(
                                    [x,y],[x+xx,y+yy],self.grid[y][x],self.grid[y+yy][x+xx])
        return G_energy
    def generateShowImage(self):
        
        
        showImg = self.get_G_energy()
        
        for item in self.robotTrace:
            showImg[item[1]][item[0]] = MapMarker.ROBOT_TRACE
        for item in self.robotPlanner:
            showImg[item[1]][item[0]] = MapMarker.PLAN_TRACE
        for item in self.obstacleInfo:
            showImg[item[1]][item[0]] = MapMarker.OBSTACLE
        showImg[self.robotInfo[1]][self.robotInfo[0]] = MapMarker.ROBOT
        showImg[self.targetInfo[1]][self.targetInfo[0]] = MapMarker.TAGERT
        showImg[self.startInfo[1]][self.startInfo[0]] = MapMarker.START

        return showImg

# 获取机器人能观测到的局部地图
def getRobotRoundMap(map:MyMap,robot:MyRobot):
    minW = np.max([robot.x - robot.r,0])
    maxW = np.min([robot.x+robot.r,map.getW()-1])
    
    minH = np.max([robot.y - robot.r,0])
    maxH = np.min([robot.y+robot.r,map.getH()-1])
    returnMap = map.grid[minH:maxH+1,minW:maxW+1]
    return returnMap

# 生成障碍物
def generateObstacle(info=0):
    obs=[]
    if(info==0):
        for i in range(1,13):
            obs.append([13,i])
            obs.append([14,i])
            
        for i in range(3,15):
            obs.append([i,30])
            obs.append([i,29])

        return obs
    else:
        if(np.random.randint(2) == 1):
            for i in range(1,13):
                obs.append([13,i])
                obs.append([14,i])     
        else:
            for i in range(1,13):
                obs.append([14,i])
                obs.append([15,i])   
        if(np.random.randint(2) == 1):
            for i in range(3,15):
                obs.append([i,30])
                obs.append([i,29])
        else:
            for i in range(3,15):
                obs.append([i,31])
                obs.append([i,30])
        return obs
# 生成起点
def generateStart():
    return [0,0]
# 生成终点
def genetateTarget():
    return [30,30]


if(__name__=="__main__"):
    mymap = MyMap(width=40,height=40)
    obs = generateObstacle()
    start = generateStart()
    target = genetateTarget()
    robot = MyRobot()
    robot.setTarget(target)
    mymap.setObstacle(obs)
    mymap.setStart(start)
    mymap.setTarget(target)
    mymap.setRobot(robot)
    mymap.setRobotR(robot.r)
    plt.figure(1)
    plt.ion()

    while(1):
        if(mymap.getrobotPos() == mymap.getTarget()):
            break
        localMap = getRobotRoundMap(mymap,robot)
        trace_planner = robot.moveByLocalMap(localMap)
        mymap.setRobot(robot)
        mymap.setRobotPlan(trace_planner)
        obs = generateObstacle(info=1)
        mymap.setObstacle(obs)
        mymap.show()
        print([robot.x,robot.y])
        plt.pause(0.1)
    plt.ioff()

    


