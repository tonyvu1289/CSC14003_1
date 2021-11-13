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
    def display_explored(self, state):
        self.explored.append(state)
    def __init__(self, input_pathway):
        self.explored = []
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

class PathWaySearch_mattrix(problem_solution.SearchProblem):
    def __init__(self, matrix, start_point, end_point,reward):
        # input is a array contain all data number
        self.map = Map()
        self.matrix = matrix
        
        self.reward = reward
        # assign data to init state and goal state
        self.init = start_point
        self.goal = end_point
        
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
            if self.matrix[newstate[0]][newstate[1]] == '+':
                addcost += self.reward[newstate]
            ans.append((action,newstate,addcost))
        return ans

class PathWaySearchBonusPoint(problem_solution.SearchProblem):
    '''state is a tuple of point which have already been visited
    last elemnet in list is current position
    '''
    def __init__(self,input_pathway):
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
        self.reward = {}
        self.reward[self.goal]=0
        for x in self.bonus_points:
            self.reward[x[0:2]] = x[-1]
        self.bonus_points = [x[0:2] for x in self.bonus_points]
    #return init state
    def initState(self):
        return (self.init,) #tuple

    #check is at goal state
    def isGoal(self, state):
        return state[-1]==self.goal
    def expandSuccessor(self, state):
        '''
        return list of (action,newstate,addcost)
        action is a list of action to reach new point from last point in state
        new state = oldstate append new pos
        '''
        ans = []
        new_states = []
        for x in self.bonus_points :
            if(x not in state):
                new_state = state + (x,)
                new_states.append(new_state)
        new_state = state + (self.goal,)
        new_states.append(new_state)
        solver = problem_solution.A_StarSolution()
        for new_state in new_states:
            temp_matrix = self.matrix
            for x in new_state[:-1]:
                if(temp_matrix[x[0]][x[1]]=='+'):
                    temp_matrix[x[0]][x[1]] = ' '
                
            problem = PathWaySearch_mattrix(temp_matrix,new_state[-2],new_state[-1],self.reward)
            solver.solve(problem)
            ans.append((solver.actions,new_state,solver.totalCost))
        return ans


#DEBUG SESSION : test class PathWaySearchProblem----------------------------------------
create_map.create_maps()
print('Please select 1 or 2:')
print('1. Testing maps without bonus points')
print('2. Testing maps with bonus point')
x = int(input('Your choose: '))
#5 maps without bouns points
if x == 1:
    for i in range(4,6):
        map_name = 'maze/maze_map'
        map_id = str(i)
        map_extesion = '.txt'

        #print id map
        print(' '.join(['Maze Map',map_id]))

        #solve problem
        problem = PathWaySearchProblem(str(map_name+map_id+map_extesion))
        solvers = [problem_solution.DFSSolution(),problem_solution.BFSSolution(),problem_solution.GreedyBestFirstSearchSolution(),problem_solution.A_StarSolution()]
        for solver in solvers:
 
            solver.solve(problem,2)
            list_actions = solver.actions
            routes = []
            # visited_point = 
            #init State
            routes.append(problem.init)

            #route (go to goal)
            k = 0
            for route in list_actions:
                route = list_actions[k][1]
                routes.append(route)
                k += 1
            
            #Visualize map
            utility.visualize_maze(problem.matrix,type(solver),i,problem.bonus_points,problem.init,problem.goal,routes,problem.explored)
            problem.explored.clear()
 #5 maps with bonus points
elif x == 2:
    for i in range(6,9):
        map_name = 'maze/maze_map'
        map_id = str(i)
        map_extesion = '.txt'

        #print id map
        print(' '.join(['Maze Map',map_id]))

        #solve problem
        problem = PathWaySearchBonusPoint(str(map_name+map_id+map_extesion))
        solvers = [problem_solution.UniformSolution()]
        for solver in solvers:
            solver.solve(problem,2)
            list_actions = solver.actions
            routes = []

            #init State
            routes.append(problem.init)

            #route (go to goal)
            k = 0
            for actions_parrent in list_actions:
                for action in actions_parrent[0]:
                    route = action[1]
                    routes.append(route)
                # route = list_actions[k][1]
                # routes.append(route)
                # k += 1
            
            #Visualize map
            utility.visualize_maze(problem.matrix,type(solver),i,problem.bonus_points,problem.init,problem.goal,routes)
 



        
        
            
    


