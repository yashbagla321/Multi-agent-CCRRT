"""
Multi agent CCRRT implimentation
Author : Yash Bagla (yashbagla321@gmail.com)

Please cite us if you're using this resource
"""

import math
import copy
import matplotlib.pyplot as plt
import random



show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, p_safe, start, goal, obstacleList, obstacleRobot, obstacleObstacle,
                 randArea, expandDis=1.0, goalSampleRate=5, maxIter=500):
        """
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.TotalCost = 0
        self.alpha = 0.1
        self.p_safe = 0.8
        self.start = Node(start[0], start[1], 0, 0.2)
        self.end = Node(goal[0], goal[1], 0, 0.2)
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.obstacleRobot = obstacleRobot
        self.obstacleObstacle = obstacleObstacle

    def Planning(self, animation=True):
        """
        Pathplanning
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind
            newNode.covariance = self.nodeList[nind].covariance*(1 + self.alpha)
            newNode.cost = self.NodeCost(newNode, self.obstacleList, self.alpha)

            if not self.__CollisionCheckObstacle(newNode, nearestNode, self.obstacleList, 0.8):
                continue
            if not self.__CollisionCheckRobot(newNode, nearestNode, self.obstacleRobot, 0.8):
                continue
            if not self.__CollisionCheckRobot(newNode, nearestNode, self.obstacleObstacle, 0.8):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y, 0.2]]
        lastIndex = len(self.nodeList) - 1
        TotalCost = 0
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            TotalCost = TotalCost + node.cost
            path.append([node.x, node.y, node.covariance])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y, 0.2])
                    
        print(path)
        print(len(path))
        return (path, TotalCost)

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                          node.y, self.nodeList[node.parent].y], "bo", ms=30 * node.covariance)
                          
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xg")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms = 30 * size)
            
        for (ax, ay, size) in self.obstacleRobot:
            plt.plot(ax, ay, "om", ms = 30 * size)
            
        for (ax, ay, size) in self.obstacleObstacle:
            plt.plot(ax, ay, "oy", ms = 30 * size)

        plt.axis([-2, 15, -2, 15])
        plt.gca().set_aspect('equal', adjustable='box')

        plt.grid(True)
        plt.pause(0.001)


    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind
        
        
        
    def __CollisionCheckObstacle(self, node, nearestNode, obstacleList, p_safe):

        for (ox, oy, size) in obstacleList:
            pnt = (ox, oy)
            start = (node.x, node.y)
            end = (nearestNode.x, nearestNode.y)
            d = ShortestDistancePointAndLineSegement(pnt, start, end)
            if d <= ((size/2) + (p_safe * (node.covariance/2))):
                return False  # collision
                
        return True  # safe

                        
    def __CollisionCheckRobot(self, node, nearestNode, obstacleRobot, p_safe):
           
        for (ox, oy, size) in obstacleRobot:
            pnt = (ox, oy)
            start = (node.x, node.y)
            end = (nearestNode.x, nearestNode.y)
            d = ShortestDistancePointAndLineSegement(pnt, start, end)
            if d <= ((size/2) + (p_safe * (node.covariance/2))):
                return False  # collision
                
        return True  # safe
        #line segement intersect?        
        x1 = obstacleRobot[0][0]
        y1 = obstacleRobot[0][1]
        x2 = obstacleRobot[1][0]
        y2 = obstacleRobot[1][1]
        x3 = node.x
        y3 = node.y
        x4 = nearestNode.x
        y4 = nearestNode.y
         
    
        num = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)
        den = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)



        if num == 0 and den == 0:
            return False
        elif num != 0 and den != 0 and x3 != x4:
            s_a = num/den
            s_b = (s_a*(x2-x1) + x1 - x3)/(x4 - x3)
            if s_a >= 0 and s_a <= 1 and s_b >= 0 and s_b <= 1:
                return False
            else:
                return True # safe
            
        else:
            return True # safe

        return True  # safe
    
    def NodeCost(self, node, obstacleList, alpha):
        
        cost = 0
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= ((size/2) + node.covariance):
                cost += abs(node.covariance + (size/2) - d)/node.covariance
            
        return cost
                
                
                

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, cost, covariance):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = cost
        self.covariance = covariance
        
        
        
        
def LazyCollisionCheckRobot(node, nearestNode, obstacleRobot, p_safe):
    """
    Lazy Check
    """
           
    for (ox, oy, size) in obstacleRobot:
        pnt = (ox, oy)
        start = (node[0], node[1])
        end = (nearestNode[0], nearestNode[1])
        d = ShortestDistancePointAndLineSegement(pnt, start, end)
        if d <= ((size/2) + (p_safe * 0.2)):
            return False  # collision
                
    return True  # safe
    #line segement intersect?        
    x1 = obstacleRobot[0][0]
    y1 = obstacleRobot[0][1]
    x2 = obstacleRobot[1][0]
    y2 = obstacleRobot[1][1]
    x3 = node.x
    y3 = node.y
    x4 = nearestNode.x
    y4 = nearestNode.y
         
    
    num = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)
    den = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)



    if num == 0 and den == 0:
        return False
    elif num != 0 and den != 0 and x3 != x4:
        s_a = num/den
        s_b = (s_a*(x2-x1) + x1 - x3)/(x4 - x3)
        if s_a >= 0 and s_a <= 1 and s_b >= 0 and s_b <= 1:
            return False
        else:
            return True # safe
            
    else:
        return True # safe

    return True  # safe
    
    
    
def ShortestDistancePointAndLineSegement(pnt, start, end):
    def dot(v,w):
        x,y = v
        X,Y = w
        return x*X + y*Y

    def length(v):
        x,y = v
        return math.sqrt(x*x + y*y)

    def vector(b,e):
        x,y = b
        X,Y = e
        return (X-x, Y-y)

    def unit(v):
        x,y = v
        mag = length(v)
        return (x/mag, y/mag)

    def distance(p0,p1):
            
        return length(vector(p0, p1))

    def scale(v,sc):
        x,y = v
        return (x * sc, y * sc)

    def add(v,w):
        x,y = v
        X,Y = w
        return (x+X, y+Y)
        
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return (dist)

def main():
    print("Start Multi-agent CC-RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        (7, 5, 5),
        (3, 6, 4),
        (5, 10, 5),
        (2, 2, 2),
        (13, 10, 5)
    ]  # [x,y,size]
    start1 = [[1, 0, 0.2]]
    start2 = [[0, 2, 0.2]]
    goal2 = [[12, 2, 0.2]]
    goal1 = [[3, 13, 0.2]]
    path_taken1 = []
    path_taken2 = []
    obstaclePathTaken = []
    obstacleRobot1 = [(0, 2, 0.2),
    (0, 2, 0.2)]
    obstacleRobot2 = [(1, 0, 0.2),
    (1, 0, 0.2)]
    
    obstaclePath = [[10, 0, 0.2], [10, 1, 0.5971967999999999], 
    [10, 2, 0.4976639999999999], 
    [10, 3, 0.4147199999999999], 
    [10, 4, 0.34559999999999996], 
    [10, 5, 0.288], [10, 6, 0.24], 
    [10, 7, 0.2], [10 , 8, 0.2],
    [10, 9, 0.2], [10, 10, 0.2],
    [10, 11, 0.2], [10, 12, 0.2]]
    obstacleObstacle = [(10, 12, 0.2), (10, 11, 0.2)]
    # Set Initial parameters
    
    rrt = RRT(p_safe = 0.8, start = (start1[0][0], start1[0][1]), goal = goal1[0],
              randArea=[-2, 14.5], obstacleList=obstacleList, obstacleRobot = obstacleRobot1, 
              obstacleObstacle = obstacleObstacle)
    path1, cost1 = rrt.Planning(animation=show_animation)
    
    plt.plot([x for (x, y, cost) in path1], [y for (x, y, cost) in path1], '-b', ms = 30 * 0.2)
    
    obstacleRobot2.pop(0)
    obstacleRobot2.append((path1[len(path1) - 2][0], path1[len(path1) - 2][1], 
                            path1[len(path1) - 2][2]))
    
    rrt = RRT(p_safe = 0.8, start = (start2[0][0], start2[0][1]), goal = goal2[0],
              randArea=[-2, 15], obstacleList=obstacleList, obstacleRobot = obstacleRobot2, 
              obstacleObstacle = obstacleObstacle)
    path2, cost2 = rrt.Planning(animation=show_animation)
    
    # plt.plot([x for (x, y, cost) in path2], [y for (x, y, cost) in path2], '-g', ms = 30 * 0.2)
    # plt.plot([x for (x, y, cost) in obstacleObstacle], [y for (x, y, cost) in obstacleObstacle], 
    #             '-k', ms = 30 * 0.2)
    obstacleRobot1.pop(0)
    obstacleRobot1.append((path2[len(path2) - 2][0], path2[len(path2) - 2][1], 
                            path2[len(path2) - 2][2]))
    
    obstacleObstacle.pop(0)
    obstaclePathTaken.append(obstaclePath[len(obstaclePath) - 1])
    obstaclePath.pop((len(obstaclePath) - 1))
    obstacleObstacle.append((obstaclePath[len(obstaclePath) - 2][0], obstaclePath[len(obstaclePath) - 2][1], 
                            path2[len(obstaclePath) - 2][2]))
                            
    #plt.show()
    #plt.pause(5)
    
    
    while len(path1) > 1 or len(path2) > 1:
        if len(path1) > 1:
            rrt = RRT(p_safe = 0.8, start = start1[0], goal = goal1[0],
              randArea=[-2, 14.5], obstacleList=obstacleList, obstacleRobot = obstacleRobot1, 
              obstacleObstacle = obstacleObstacle)
            path_r1, cost1 = rrt.Planning(animation=show_animation)
        
            if LazyCollisionCheckRobot(path1[0], path1[1], obstacleRobot1, 0.8) == False:
                path1 = path_r1
                
            if LazyCollisionCheckRobot(path1[0], path1[1], obstacleObstacle, 0.8) == False:
                path1 = path_r1
                
                
                 
            
            if len(path1) > len(path_r1):
                path1 = path_r1
            
            else:
                path1.remove(path1[(len(path1) - 2)])
            
            # plt.plot([x for (x, y, cost) in path1], [y for (x, y, cost) in path1], '-b', ms = 30 * 0.2)
        
            obstacleRobot2.pop(0)
            if len(path1) > 2:
                obstacleRobot2.append((path1[len(path1) - 2][0], path1[len(path1) - 2][1], 
                            path1[len(path1) - 2][2]))
            else:
                obstacleRobot2.append((goal1[0][0], goal1[0][1], goal1[0][2]))
            path_taken1.append((start1[0][0], start1[0][1], start1[0][2]))    
            start1[0] = path1[(len(path1) - 2)]
            
            
            
        if len(path2) > 1:
            rrt = RRT(p_safe = 0.8, start = start2[0], goal = goal2[0],
              randArea=[-2, 14.5], obstacleList=obstacleList, obstacleRobot = obstacleRobot2, 
              obstacleObstacle = obstacleObstacle)
            path_r2, cost2 = rrt.Planning(animation=show_animation)
            
            if LazyCollisionCheckRobot(path2[0], path2[1], obstacleRobot2, 0.8) == False:
                path2 = path_r2
                
            if LazyCollisionCheckRobot(path2[0], path2[1], obstacleObstacle, 0.8) == False:
                path2 = path_r2
        
        
            if len(path2) > len(path_r2):
                path2 = path_r2
            
            else:
                path2.remove(path2[(len(path2) - 2)])
            
            # plt.plot([x for (x, y, cost) in path2], [y for (x, y, cost) in path2], '-g', ms = 30 * 0.2)
        
            obstacleRobot1.pop(0)
            if len(path2) > 2:
                obstacleRobot1.append((path2[len(path2) - 2][0], path2[len(path2) - 2][1], 
                            path2[len(path2) - 2][2]))
            else:
                obstacleRobot1.append((goal2[0][0], goal2[0][1], goal2[0][2]))
            path_taken2.append((start2[0][0], start2[0][1], start2[0][2]))    
            start2[0] = path2[(len(path2) - 2)]
            
        obstacleObstacle.pop(0)
        
        if len(obstaclePath) > 2:
            obstaclePathTaken.append(obstaclePath[len(obstaclePath) - 1])
            obstaclePath.pop((len(obstaclePath) - 1))
            obstacleObstacle.append((obstaclePath[len(obstaclePath) - 2][0], obstaclePath[len(obstaclePath) - 2][1], 
                            path2[len(obstaclePath) - 2][2]))
        else:
            obstaclePathTaken.append(obstaclePath[0])
            obstacleObstacle.append((obstaclePath[0][0], obstaclePath[0][1], obstaclePath[0][2]))   
        #plt.show()
        #plt.pause(5)

        
    # Draw final path
    print('Final path Robot 1:', path_taken1)
    print('Final path Robot 2:', path_taken2)
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y, cost) in path_taken1], [y for (x, y, cost) in path_taken1], 'bo', ms = 30 * 0.2)
        plt.plot([x for (x, y, cost) in path_taken2], [y for (x, y, cost) in path_taken2], 'go', ms = 30 * 0.2)
        plt.plot([x for (x, y, cost) in path_taken1], [y for (x, y, cost) in path_taken1], '-b', ms = 30 * 0.2)
        plt.plot([x for (x, y, cost) in path_taken2], [y for (x, y, cost) in path_taken2], '-g', ms = 30 * 0.2)  
        plt.plot([x for (x, y, cost) in obstaclePathTaken], [y for (x, y, cost) in obstaclePathTaken], '-m', ms = 30 * 0.2)  
        plt.plot([x for (x, y, cost) in obstaclePathTaken], [y for (x, y, cost) in obstaclePathTaken], 'mo', 
                        ms = 30 * 0.5)  
        plt.grid(True)
        plt.show()
        


if __name__ == '__main__':
    main()
