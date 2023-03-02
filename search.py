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

def depthFirstSearch(problem):
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

    "Copied the code from BrFS, changed to stack"
    "modified to try ID first"
    startState = problem.getStartState()
    myStack = util.Stack()
    startNode = (startState,[])
    myStack.push(startNode)
    #generated = set()
    #generated.add(startState)

    limit = 1
    found = False

    while not found:

        myStack = util.Stack()
        myStack.push(startNode)
        generated = set()
        generated.add(startState)
        
        while not myStack.isEmpty():
            node = myStack.pop()
            state, actionList = node

            if problem.isGoalState(state):
                found = True
                return actionList
            elif len(actionList) >= limit:
                continue
            else:
                for successor in problem.getSuccessors(state):
                    nextState, action, _ = successor

                    if not nextState in generated:
                        nextNode = (nextState,actionList+[action])
                        generated.add(nextState)
                        myStack.push(nextNode)

        limit = limit + 1

    
    
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    startState = problem.getStartState()
    myQ = util.Queue()
    startNode = (startState,[])
    myQ.push(startNode)
    generated = set()
    generated.add(startState)
    while not myQ.isEmpty():
        node = myQ.pop()
        state, actionList = node
        if problem.isGoalState(state):
            return actionList
        else:
            for successor in problem.getSuccessors(state):
                nextState, action, _ = successor
                if not nextState in generated:
                    nextNode = (nextState,actionList+[action])
                    generated.add(nextState)
                    myQ.push(nextNode)


    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    "attempt1: basing off A* code"
    myPQ = util.PriorityQueue()
    startState = problem.getStartState()
    startNode = (startState, '',0, [])
    "g of start = 0"
    myPQ.push(startNode, 0)
    visited = set()
    best_g = dict()
    while not myPQ.isEmpty():
        node = myPQ.pop()
        state, action, cost, path = node
        if (not state in visited) or cost < best_g.get(state):
            visited.add(state)
            best_g[state]=cost
            if problem.isGoalState(state):
                path = path + [(state, action)]
                actions = [action[1] for action in path]
                del actions[0]
                return actions
            for succ in problem.getSuccessors(state):
                succState, succAction, succCost = succ
                newNode = (succState, succAction, cost + succCost, path + [(node, action)])
                "removed the heuristic bit"
                myPQ.push(newNode,cost+succCost)
    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

# Please DO NOT change the following code, we will use it later
def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    myPQ = util.PriorityQueue()
    startState = problem.getStartState()
    startNode = (startState, '',0, [])
    myPQ.push(startNode,heuristic(startState,problem))
    visited = set()
    best_g = dict()
    while not myPQ.isEmpty():
        node = myPQ.pop()
        state, action, cost, path = node
        if (not state in visited) or cost < best_g.get(state):
            visited.add(state)
            best_g[state]=cost
            if problem.isGoalState(state):
                path = path + [(state, action)]
                actions = [action[1] for action in path]
                del actions[0]
                return actions
            for succ in problem.getSuccessors(state):
                succState, succAction, succCost = succ
                newNode = (succState, succAction, cost + succCost, path + [(node, action)])
                myPQ.push(newNode,heuristic(succState,problem)+cost+succCost)
    util.raiseNotDefined()

"attempt 2"
def enforcedHillClimbing(problem, heuristic=nullHeuristic):
    """
    Local search with heuristic function.
    You DO NOT need to implement any heuristic, but you DO have to call it.
    The heuristic function is "manhattanHeuristic" from searchAgent.py.
    It will be pass to this function as second argument (heuristic).
    """
    "*** YOUR CODE HERE FOR TASK 1 ***"
    state = problem.getStartState()
    node = (state, '', [])

    while not problem.isGoalState(state):
        node = efcImprove(node, problem, heuristic)
        state = node[0]

    state, action, path = node
    path = path + [(state,action)]
    actions = [action[1] for action in path]
    del actions[0]
    return actions

    util.raiseNotDefined()

"improve function for EHC, based off lecture slides"
def efcImprove(initialNode, problem, heuristic):
    myQ = util.Queue()
    myQ.push(initialNode)
    closed = set()
    while not myQ.isEmpty():
        node = myQ.pop()
        state, action, path = node
        if (not state in closed):
            closed.add(state)

            if heuristic(state,problem) < heuristic(initialNode[0],problem):
                return node
            for succ in problem.getSuccessors(state):
                succState, succAction, succCost = succ
                newNode = (succState, succAction, path + [(node, action)])
                myQ.push(newNode)


"attempt new 1 "        
def idaStarSearch(problem, heuristic=nullHeuristic):
    """
    Global search with heuristic function.
    You DO NOT need to implement any heuristic, but you DO have to call it.
    The heuristic function is "manhattanHeuristic" from searchAgent.py.
    It will be pass to this function as second argument (heuristic).
    """
    "*** YOUR CODE HERE FOR TASK 2 ***"

    startState = problem.getStartState()
    startNode = (startState, '', 0, [])
    limit = 0 + heuristic(startState, problem)
    
    while True:
        myStack = util.Stack()
        myStack.push(startNode)
        #wipe list; list holds f of frontier nodes to find the min after loop
        fList = []

        while not myStack.isEmpty():
            node = myStack.pop()
            state, action, cost, path = node
            if problem.isGoalState(state):
                path = path + [(state, action)]
                actions = [action[1] for action in path]
                del actions[0]
                return actions
            else:
                for succ in problem.getSuccessors(state):
                    succState, succAction, succCost = succ
                    newNode = (succState, succAction, cost + succCost, path + [(node, action)])
                    if (cost + succCost + heuristic(succState,problem)) > limit:
                        fList.append(cost + succCost + heuristic(succState, problem))
                    else:
                        myStack.push(newNode)
                        
        limit = min(fList)
    
    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ida = idaStarSearch
ehc = enforcedHillClimbing
