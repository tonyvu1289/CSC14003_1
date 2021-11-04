import heapq
from collections import deque

class PriorityQueue:
    def __init__(self) -> None:
        # priority is a "element to priority value of that element" dictionary
        self.heap = []
        self.priority = {}
        self.DONE = -10000000

    def isEmpty(self):
        return len(self.priority) == 0

    def add_update(self, element, priorityValue):
        if element in self.priority:  #
            if self.priority[element] <= priorityValue:
                return 0  # not success
        self.priority[element] = priorityValue
        heapq.heappush(self.heap, (priorityValue, element))
        return 1  # success

    def pop_delete(self):
        while len(self.heap) > 0:
            priority, state = heapq.heappop(self.heap)
            if self.priority[state] == self.DONE: continue  # Outdated priority, skip
            self.priority[state] = self.DONE
            return state, priority
        return (None, None)  # Nothing left...


class SearchProblem:
    """class trừu tượng. Những hàm trong class này là hàm ảo"""

    def __init__(self):
        raise NotImplementedError("No __init__ yet!")

    def initState(self): raise NotImplementedError("No initState() implementation yet!")

    def isGoal(self, state): raise NotImplementedError("No isGoal() implementation yet!")

    def expandSuccessor(self, state): raise NotImplementedError("No expandSuccessor implementation yet!")

    def display_explored(self, state): pass


class Solution:
    """class trừu tượng cho những chiến lược tìm kiếm"""

    def __init__(self):
        raise NotImplementedError("No __init__() implementation yet!")

    def solve(self, problem : SearchProblem, display=0):
        raise NotImplementedError("No solve() implementation yet!")


class A_StarSolution(Solution):
    def __init__(self):
        self.heuristic_choice = 1

    def setHeuristic(self, choice):
        self.heuristic_choice = choice

    def solve(self, problem, display=0):
        self.totalCost = 0
        self.totalExpand = 0
        self.actions = []
        # this list is used to display result which is a sequence of actions and the state after the action
        self.cost = {}
        # a dictionary Map a state to how much cost to reach that state
        self.backPointer = {}
        # a dictionary Map a state to its (action,parent state)
        frontier = PriorityQueue()
        # in A_star, i will use priority queue to choose which state have a min g(n)+h(n)
        # with g(n) is a certain cost to reach state n, h(n) is a heuristic func
        # after have solved, frontier is no use. So I will make it wiped out =))))
        frontier.add_update(problem.initState(), 0)
        self.cost[problem.initState()] = 0
        while not frontier.isEmpty():
            parentState = frontier.pop_delete()[0]
            parentCost = self.cost[parentState]
            self.totalExpand = self.totalExpand + 1
            if problem.isGoal(parentState):
                state = parentState
                self.totalCost = self.cost[state]
                while state is not problem.initState():
                    act, par = self.backPointer[state]
                    self.actions.append((act, state))
                    state = par
                self.actions.reverse()
                if display >= 2:
                    for act, reach_state in self.actions:
                        print("Do %s to reach state %s \n" % (act, reach_state))
                if (display >= 1):
                    print("Total node expand : %s \n "
                          "Total cost to reach goal : %s" % (self.totalExpand, self.totalCost))
                return
            if display >= 2:
                print(("Expanding %s with reached cost is %s" % (parentState, parentCost)))
            if display >= 1:
                problem.display_explored(parentState)
            for action, newState, addCost in problem.expandSuccessor(parentState):
                if (newState in self.cost) and (self.cost[newState] <= parentCost + addCost):
                    # tránh bị vòng lặp
                    continue
                if frontier.add_update(newState,
                                       parentCost + addCost + problem.heuristic(newState)):
                    self.cost[newState] = parentCost + addCost
                    self.backPointer[newState] = (action, parentState)

        print("No path found or something happened and my alogoritm failed :(")


class UniformSolution(Solution):
    def __init__(self):
        pass

    def solve(self, problem, display=0):
        self.totalCost = 0
        # total path-cost of solution
        self.totalExpand = 0
        # count nodes have been expanded
        self.actions = []
        # a sequence of action (final solution)
        self.backPointer = {}
        # backPointer is dict state : (action,parent-state)
        frontier = PriorityQueue()
        # In UCS, the priority queue is used to define the minium path-cost node (state,path-cost)
        frontier.add_update(problem.initState(), 0)
        while not frontier.isEmpty():
            parentState, parentCost = frontier.pop_delete()
            self.totalExpand = self.totalExpand + 1
            if problem.isGoal(parentState):
                state = parentState
                self.totalCost = parentCost
                while state is not problem.initState():
                    act, par = self.backPointer[state]
                    self.actions.append((act, state))
                    state = par
                self.actions.reverse()
                if display >= 2:
                    for act, reach_state in self.actions:
                        print("Do %s to reach state %s \n" % (act, reach_state))
                if display >= 1:
                    print("Total node expand : %s \n "
                          "Total cost to reach goal : %s" % (self.totalExpand, self.totalCost))
                return
            if display >= 2:
                print(("Expanding %s with reached cost is %s" % (parentState, parentCost)))
            if display >= 1:
                problem.display_explored(parentState)
            for action, newState, addCost in problem.expandSuccessor(parentState):
                if frontier.add_update(newState, parentCost + addCost):
                    self.backPointer[newState] = (action, parentState)

class GreedyBestFirstSearchSolution(Solution):
    '''Attention! This is may not an optimal solution'''
    def __init__(self):
        self.heuristic_choice = 1

    def setHeuristic(self, choice):
        self.heuristic_choice = choice

    def solve(self, problem, display=0):
        self.totalCost = 0
        self.totalExpand = 0
        self.actions = []
        # this list is used to display result which is a sequence of actions and the state after the action
        self.cost = {}
        # a dictionary Map a state to how much cost to reach that state
        self.backPointer = {}
        # a dictionary Map a state to its (action,parent state)
        frontier = PriorityQueue()
        # in A_star, i will use priority queue to choose which state have a min g(n)+h(n)
        # with g(n) is a certain cost to reach state n, h(n) is a heuristic func
        # after have solved, frontier is no use. So I will make it wiped out =))))
        frontier.add_update(problem.initState(), 0)
        self.cost[problem.initState()] = 0
        while not frontier.isEmpty():
            parentState = frontier.pop_delete()[0]
            parentCost = self.cost[parentState]
            self.totalExpand = self.totalExpand + 1
            if problem.isGoal(parentState):
                state = parentState
                self.totalCost = self.cost[state]
                while state is not problem.initState():
                    act, par = self.backPointer[state]
                    self.actions.append((act, state))
                    state = par
                self.actions.reverse()
                if display >= 2:
                    for act, reach_state in self.actions:
                        print("Do %s to reach state %s \n" % (act, reach_state))
                if (display >= 1):
                    print("Total node expand : %s \n "
                          "Total cost to reach goal : %s" % (self.totalExpand, self.totalCost))
                return
            if display >= 2:
                print(("Expanding %s with reached cost is %s" % (parentState, parentCost)))
            if display >= 1:
                problem.display_explored(parentState)
            for action, newState, addCost in problem.expandSuccessor(parentState):
                if (newState in self.cost) and (self.cost[newState] <= parentCost + addCost):
                    # tránh bị vòng lặp
                    continue
                if frontier.add_update(newState,
                                       problem.heuristic(newState)):
                    self.cost[newState] = parentCost + addCost
                    self.backPointer[newState] = (action, parentState)

        print("No path found or something happened and my alogoritm failed :(")

class DFSSolution(Solution):
    def __init__(self):
        pass

    def solve(self, problem, display=0):
        self.totalCost = 0
        # total path-cost of solution
        self.totalExpand = 0
        # count nodes have been expanded
        self.actions = []
        # a sequence of action (final solution)
        self.backPointer = {}
        # backPointer is dict state : (action,parent-state)
        frontier = deque()
        # In UCS, the priority queue is used to define the minium path-cost node (state,path-cost)
        self.backPointer[problem.initState()]=('start','None')
        frontier.append((problem.initState(),0))
        while frontier:
            parentState, parentCost = frontier.pop()
            self.totalExpand = self.totalExpand + 1
            if problem.isGoal(parentState):
                state = parentState
                self.totalCost = parentCost
                while state is not problem.initState():
                    act, par = self.backPointer[state]
                    self.actions.append((act, state))
                    state = par
                self.actions.reverse()
                if display >= 2:
                    for act, reach_state in self.actions:
                        print("Do %s to reach state %s \n" % (act, reach_state))
                if display >= 1:
                    print("Total node expand : %s \n "
                          "Total cost to reach goal : %s" % (self.totalExpand, self.totalCost))
                return
            if display >= 2:
                print(("Expanding %s with reached cost is %s" % (parentState, parentCost)))
            if display >= 1:
                problem.display_explored(parentState)
            for action, newState, addCost in problem.expandSuccessor(parentState):
                if(newState in self.backPointer): #tránh lặp
                    continue
                frontier.append((newState,parentCost+addCost))
                self.backPointer[newState] = (action, parentState)

class BFSSolution(Solution):
    def __init__(self):
        pass

    def solve(self, problem, display=0):
        self.totalCost = 0
        # total path-cost of solution
        self.totalExpand = 0
        # count nodes have been expanded
        self.actions = []
        # a sequence of action (final solution)
        self.backPointer = {}
        # backPointer is dict state : (action,parent-state)
        frontier = deque()
        # In UCS, the priority queue is used to define the minium path-cost node (state,path-cost)
        self.backPointer[problem.initState()]=('start','None')
        frontier.append((problem.initState(),0))
        while frontier:
            parentState, parentCost = frontier.popleft()
            self.totalExpand = self.totalExpand + 1
            if problem.isGoal(parentState):
                state = parentState
                self.totalCost = parentCost
                while state is not problem.initState():
                    act, par = self.backPointer[state]
                    self.actions.append((act, state))
                    state = par
                self.actions.reverse()
                if display >= 2:
                    for act, reach_state in self.actions:
                        print("Do %s to reach state %s \n" % (act, reach_state))
                if display >= 1:
                    print("Total node expand : %s \n "
                          "Total cost to reach goal : %s" % (self.totalExpand, self.totalCost))
                return
            if display >= 2:
                print(("Expanding %s with reached cost is %s" % (parentState, parentCost)))
            if display >= 1:
                problem.display_explored(parentState)
            for action, newState, addCost in problem.expandSuccessor(parentState):
                if(newState in self.backPointer): #tránh lặp
                    continue
                frontier.append((newState,parentCost+addCost))
                self.backPointer[newState] = (action, parentState)