from mainproblem import *

create_map.create_maps()
print('Please select 1 or 2:')
print('1. Testing maps without bonus points')
print('2. Testing maps with bonus point')
x = int(input('Your choice: '))
#5 maps without bouns points
if x == 1:
    for i in range(1,6):
        map_name = sys.path[0] +'/../maze/maze_map'
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
        map_name = sys.path[0] +'/../maze/maze_map'
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
 