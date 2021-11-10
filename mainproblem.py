from typing import List, Any
import math
import problem_solution
import create_map
import utility
import os
import re
import numpy as np
import copy
np.seterr(over='ignore') # ignore RuntimeWarning: overflow encountered in ubyte_scalars


class Map:
    through_point_list = []
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
    
    def all_bonus_point(self,pos,bonus_points,goal):
        ans = []
        self.through_point_list.append(pos)
        for i in range(len(bonus_points)+1):
            if i >= len(bonus_points):
                new_point: tuple[int,int] = goal
                reward_point = 100
                ans.append(('hi',new_point,reward_point))
            else:
                new_point: tuple[int,int] = (bonus_points[i][0],bonus_points[i][1])
                reward_point = (bonus_points[i][2])
                if new_point in self.through_point_list:
                    continue
                else: 
                    ans.append(('hi',new_point,reward_point))
        return ans

class PathWaySearchProblem(problem_solution.SearchProblem):
    def __init__(self, input_pathway):
        self.count_show = 0
        # input is a array contain all data number
        self.bonus_points, self.matrix = utility.read_file(input_pathway)
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

class PathWaySearchBounsPoint(problem_solution.SearchProblem):
    def __init__(self, matrix, start_point, end_point):
        # input is a array contain all data number
        self.map = Map()
        # assign data to init state and goal state
        self.init = start_point
        self.goal = end_point
        
    #check is at init state
    def initState(self):
        return self.init

    #check is at goal state
    def isGoal(self, state):
        return self.goal == state

    def get_reward(self):
        return self.reward

    def heuristic(self,state):
        #Distance Manhattan |x1-x2|+|y1-y2|
        h = (math.fabs(state[0] - self.goal[0])) + (math.fabs(state[1]-self.goal[1]))
        return h
       
    def expandSuccessor(self, state):
        ans = []
        for action, newstate in self.map.all_move(state):
            if matrix[newstate[0]][newstate[1]] == 'x':
                continue
            #Cost to move 1 step is 1
            addcost = 1
            ans.append((action,newstate,addcost))
        return ans

class SelectBonusPoint(problem_solution.SearchProblem):
    def __init__(self, bonus_points, matrix):
        self.map = Map()
        # assign data to init state and goal state
        for i in range(len(matrix)):
            for j in range(len(matrix[0])):
                if matrix[i][j] == 'S':
                    self.init = (i,j)
                elif matrix[i][j]==' ':
                    if (i==0) or (i==len(matrix)-1) or (j==0) or (j==len(matrix[0])-1):
                        self.goal = (i,j)
                else:
                    pass
        self.list_points = bonus_points
    #check is at init state
    def initState(self):
        return self.init

    #check is at goal state
    def isGoal(self, state):
        return self.goal == state

    def getInitSate(self):
        return self.init

    def getGoalState(self):
        return self.goal

    def heuristic(self,state):
        #Distance Manhattan |x1-x2|+|y1-y2|
        h = (math.fabs(state[0] - self.goal[0])) + (math.fabs(state[1]-self.goal[1]))
        return h
       
    def expandSuccessor(self, state):
        ans = []
        for action, newstate, reward in self.map.all_bonus_point(state,self.list_points,self.goal):
            addcost = reward
            ans.append((action,newstate,addcost))
        return ans


#DEBUG SESSION : test class PathWaySearchProblem----------------------------------------
create_map.create_maps()
print('Please select 1 or 2:')
print('1. Testing maps without bonus points')
print('2. Testing maps with bonus point')
x = int(input('Your choose: '))
#5 maps without bouns points
if x == 1:
    for i in range(1,6):
        map_name = 'maze_map'
        map_id = str(i)
        map_extesion = '.txt'

        #print id map
        print(' '.join(['Maze Map',map_id]))

        #solve problem
        problem = PathWaySearchProblem(str(map_name+map_id+map_extesion))
        solvers = [problem_solution.DFSSolution(),problem_solution.BFSSolution(),problem_solution.GreedyBestFirstSearchSolution(),problem_solution.A_StarSolution(),problem_solution.UniformSolution()]
        for solver in solvers:
            #get aglorithm name

            
            solver.solve(problem,2)
            list_actions = solver.actions
            routes = []

            #init State
            routes.append(problem.init)

            #route (go to goal)
            k = 0
            for route in list_actions:
                route = list_actions[k][1]
                routes.append(route)
                k += 1
            
            #Visualize map
            utility.visualize_maze(problem.matrix,type(solver),i,problem.bonus_points,problem.init,problem.goal,routes)
 #5 maps with bonus points
elif x == 2:
   
    for s in range(1,6):
        map_name = 'maze_map'
        map_id = str(s)
        map_extesion = '.txt'

        #print id map
        print(' '.join(['Maze Map',map_id]))

        #solve problem child (find many suitable bonus points)
        input_pathway = str(map_name+map_id+map_extesion)
        bonus_points, matrix = utility.read_file(input_pathway)
        
        problem_child = SelectBonusPoint(bonus_points,matrix)
        solver = problem_solution.A_StarSolution()
        solver.solve(problem_child,2)

        #Get init and goal state
        initPoint = problem_child.getInitSate()
        goalPoint = problem_child.getGoalState()

        temp_list = solver.actions
        routes=[]

        #add init state
        routes.append(initPoint)
        list_actions_points = []
        k = 0
        for action in temp_list:
            action = temp_list[k][1]
            list_actions_points.append(action)
            k += 1
        
        #solve proble parent (find the path among two points)
        solver_parents = [problem_solution.DFSSolution(),problem_solution.BFSSolution(),problem_solution.GreedyBestFirstSearchSolution(),problem_solution.A_StarSolution(),problem_solution.UniformSolution()]
        for solver_parent in solver_parents:
            for i in range(len(list_actions_points)):
                if i == 0:
                    problem_parent = PathWaySearchBounsPoint(matrix,initPoint,list_actions_points[i])
                    solver_parent.solve(problem_parent,2)
                    list_actions_parents = solver_parent.actions
                    k = 0
                    for route in list_actions_parents:
                        route = list_actions_parents[k][1]
                        routes.append(route)
                        k += 1

                else:
                    problem_parent = PathWaySearchBounsPoint(matrix,list_actions_points[i-1],list_actions_points[i])
                    solver_parent.solve(problem_parent,2)
                    list_actions_parents = solver_parent.actions
                    k = 0
                    for route in list_actions_parents:
                        route = list_actions_parents[k][1]
                        routes.append(route)
                        k += 1
            utility.visualize_maze(matrix,type(solver_parent),s,bonus_points,initPoint,goalPoint,routes)




        
        
            
    


