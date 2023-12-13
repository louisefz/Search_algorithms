# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    fringe = util.Stack()
    past = []
    current_state = (problem.getStartState(),[],0)
    fringe.push(current_state)

    while not problem.isGoalState(current_state[0]):
        if current_state[0] not in past:
            past.append(current_state[0])
            for state, path, cost in problem.getSuccessors(current_state[0]):
                if state not in past:
                    fringe.push((state,current_state[1]+[path], current_state[2]+cost))  #将B存入stack中
        current_state = fringe.pop()  ##再从fringe中取B， B的triple set list

    return current_state[1]





def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # 利用queue的FIFO特性，可以优先从广度上访问节点
    fringe = util.Queue()
    start = (problem.getStartState(), [], 0)
    fringe.push(start)
    past= []

    while not fringe.isEmpty():
        node, path,cost = fringe.pop()
        if problem.isGoalState(node):
            return path
        if not node in past:
            past.append(node)
            for coord, move, n_cost in problem.getSuccessors(node):
                fringe.push((coord, path + [move],cost+n_cost))


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""

    "*** YOUR CODE HERE ***"
    past = []
    fringe = util.PriorityQueue()
    start = (problem.getStartState(), [])
    fringe.push(start, 0)

    while not fringe.isEmpty():
        state, path = fringe.pop()

        if problem.isGoalState(state):
            return path

        if state not in past:
            past.append(state)
            for node in problem.getSuccessors(state):
                now_state = node[0]
                now_path = path + [node[1]]
                now_cost= problem.getCostOfActions(path + [node[1]])
                fringe.update((now_state,now_path),now_cost)

    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    past = []
    fringe = util.PriorityQueue()
    start = (problem.getStartState(), [], 0)
    fringe.push(start,0)

    while not fringe.isEmpty():
        state, path, cost = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in past:
            past.append(state)
            for s,p,c in problem.getSuccessors(state):
                now_cost = cost + c
                now_state_path_cost = (s, path+[p], now_cost)
                fringe.push(now_state_path_cost, now_cost + heuristic(s, problem))
    util.raiseNotDefined()




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
