from typing import List, Any
import math
import problem_solution
import ulity
import os
import cv2 as cv
import re
import numpy as np
import copy
np.seterr(over='ignore') # ignore RuntimeWarning: overflow encountered in ubyte_scalars


class Map:
    def set(self, pos, value):
        self.data[pos[0], pos[1]] = value

    def all_move(self, pos):
        # all move can perform from the position pos
        # return action and new position
        ans = []
        dx = [0,0,1,-1]
        dy = [1,-1,0,0]
        for i in range(4):
            newPos: tuple[int, int] = (pos[0] + dx[i], pos[1] + dy[i])
            #4 directions
            if (not pos == newPos) and ( newPos[0] >= 0) and (newPos[1] >= 0):
                if dx[i] == 0 and dy[i] == 1:
                    ans.append(('right', newPos))
                if dx[i] == 0 and dy[i] == -1:
                    ans.append(('left', newPos))
                if dx[i] == 1 and dy[i] == 0:
                    ans.append(('down', newPos))
                if dx[i] == -1 and dy[i] == 0:
                    ans.append(('up', newPos))
        return ans

class PathWaySearchProblem(problem_solution.SearchProblem):
    def __init__(self, input_pathway):
        self.count_show = 0
        # input is a array contain all data number
        self.bonus_points, self.matrix = ulity.read_file(input_pathway)
        self.map = Map()
        # assign data to init state and goal state
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] == 'S':
                    self.init = (i,j)
                elif self.matrix[i][j]==' ':
                    if (i==0) or (i==len(self.matrix)-1) or (j==0) or (j==len(self.matrix[0])-1):
                        self.goal = (i,j)
                else:
                    pass
    #check is at init state
    def initState(self):
        return self.init

    #check is at goal state
    def isGoal(self, state):
        return self.goal == state
    
    def heuristic(self,state):
        #Distance Manhattan |x1-x2|+|y1-y2|
        h = (math.fabs(state[0] - self.goal[0])) + (math.fabs(state[1]-self.goal[1]))
        return h
       
    def expandSuccessor(self, state):
        ans = []
        for action, newstate in self.map.all_move(state):
            if self.matrix[newstate[0]][newstate[1]] == 'x':
                continue
            #Cost to move 1 step is 1
            addcost = 1
            ans.append((action,newstate,addcost))
        return ans



#DEBUG SESSION : test class PathWaySearchProblem
#Aglo Astar
# 5 maps without bouns points
for i in range(1,6):
    map_name = 'maze_map'
    map_id = str(i)
    map_extesion = '.txt'

    #print id map
    print(' '.join(['Maze Map',map_id]))

    #solve problem
    problem = PathWaySearchProblem(str(map_name+map_id+map_extesion))
    solvers = [problem_solution.A_StarSolution(),problem_solution.UniformSolution()]
    for solver in solvers:
        solver.solve(problem)
        list_actions = solver.actions
        routes = []

        #init State
        routes.append(problem.init)

        #route (go to goal)
        i = 0
        for route in list_actions:
            route = list_actions[i][1]
            routes.append(route)
            i += 1
        
        #Visualize map
        ulity.visualize_maze(problem.matrix,problem.bonus_points,problem.init,problem.goal,routes)





