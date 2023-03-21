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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # Initialize an empty stack to hold nodes to be explored
    frontier = util.Stack()

    # Keep track of previously explored nodes to avoid loops
    explored = set()

    # Create the initial node with the start state and empty list of actions
    start_state = problem.getStartState()
    start_node = (start_state, [])

    # Push the initial node onto the frontier to start the search
    frontier.push(start_node)

    # Loop until the frontier is empty
    while not frontier.isEmpty():

        # Pop the next node from the frontier to explore
        current_state, actions = frontier.pop()

        # If the current state has not been explored yet
        if current_state not in explored:

            # Mark the current state as explored
            explored.add(current_state)

            # If the current state is the goal state, return the list of actions taken to reach it
            if problem.isGoalState(current_state):
                return actions

            # Otherwise, expand the current node by getting its successor states and adding them to the frontier
            else:
                successors = problem.getSuccessors(current_state)
                for successor_state, successor_action, successor_cost in successors:
                    new_actions = actions + [successor_action]
                    new_node = (successor_state, new_actions)
                    frontier.push(new_node)

    # If the frontier is empty and the goal state has not been found, return an empty list of actions
    return []
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # Initialize the frontier with the starting state and empty path and cost
    startState = problem.getStartState()
    startNode = (startState, [], 0)
    frontier = util.Queue()      # to be explored (FIFO)
    frontier.push(startNode)

    # Initialize the explored set with the starting state
    explored = set()
    explored.add(startState)

    # Perform BFS
    while not frontier.isEmpty():
        # Get the next node to explore from the frontier
        currentState, path, cost = frontier.pop()

        # Check if the current state is the goal state
        if problem.isGoalState(currentState):
            return path

        # Otherwise, expand the node and add its successors to the frontier
        for successorState, action, stepCost in problem.getSuccessors(currentState):
            if successorState not in explored:
                explored.add(successorState)
                successorPath = path + [action]
                successorCost = cost + stepCost
                successorNode = (successorState, successorPath, successorCost)
                frontier.push(successorNode)

    # If the goal state is not found, return an empty list
    return []
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    # Initialize frontier with PriorityQueue (FIFO)
    frontier = util.PriorityQueue()

    # Dictionary to hold states that have been explored (for cycle checking)
    explored_nodes = {}

    # Get start state, and initialize start node with empty path and cost 0
    start_state = problem.getStartState()
    start_node = (start_state, [], 0)

    # Push start node to frontier with priority 0
    frontier.push(start_node, 0)

    # Loop until frontier is empty
    while not frontier.isEmpty():
        # Pop the node with the least total cost from the frontier
        current_state, path, current_cost = frontier.pop()

        # Check if current state has been explored before, and skip if it has been explored with lower cost
        if (current_state not in explored_nodes) or (current_cost < explored_nodes[current_state]):
            # Add current state and its total cost to explored_nodes dictionary
            explored_nodes[current_state] = current_cost

            # Check if current state is the goal state, and return path if it is
            if problem.isGoalState(current_state):
                return path

            # Expand current state to its successors and add them to frontier
            successors = problem.getSuccessors(current_state)
            for state, action, cost in successors:
                # Create a new path by adding the current action to the existing path
                new_path = path + [action]

                # Compute the total cost of the new path
                new_cost = current_cost + cost

                # Create a new node with the new state, new path, and new cost
                new_node = (state, new_path, new_cost)

                # Add the new node to frontier with priority equal to its total cost
                frontier.push(new_node, new_cost)

    # If goal state is not found, return an empty path
    return []
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    # to be explored (FIFO): takes in item, cost+heuristic
    frontier = util.PriorityQueue()

    # Create an empty list to hold explored nodes
    explored_nodes = []

    # Get the start state and create a node to represent it
    start_state = problem.getStartState()
    start_node = (start_state, [], 0)

    # Add the start node to the frontier with a priority of 0
    frontier.push(start_node, 0)

    # Keep searching until the frontier is empty
    while not frontier.isEmpty():

        # Pop the node with the lowest combined cost and heuristic from the frontier
        current_state, actions, current_cost = frontier.pop()

        # Add the popped node to the explored list
        explored_nodes.append((current_state, current_cost))

        # Check if the current state is the goal state
        if problem.isGoalState(current_state):
            return actions

        # If the current state is not the goal state, expand it
        else:
            # Get the successors of the current state
            successors = problem.getSuccessors(current_state)

            # Iterate over each successor
            for succ_state, succ_action, succ_cost in successors:
                # Create a new node representing the successor
                new_action = actions + [succ_action]
                new_cost = problem.getCostOfActions(new_action)
                new_node = (succ_state, new_action, new_cost)

                # Check if the successor has already been explored
                already_explored = False
                for explored in explored_nodes:
                    # If the successor state has already been explored and the new cost is greater than or equal to
                    # the cost of the explored node, mark it as already explored
                    explored_state, explored_cost = explored
                    if (succ_state == explored_state) and (new_cost >= explored_cost):
                        already_explored = True

                # If the successor has not been explored, add it to the frontier and explored list
                if not already_explored:
                    frontier.push(new_node, new_cost +
                                  heuristic(succ_state, problem))
                    explored_nodes.append((succ_state, new_cost))

    # If the frontier is empty and no solution has been found, return an empty list of actions
    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
