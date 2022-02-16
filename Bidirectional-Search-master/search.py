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
    
    closed_set = []
    actions = []
    fringe = util.Stack()
    #add node and set of actions into the fringe
    fringe.push((problem.getStartState(), actions))

    #run while fringe is not empty
    while not fringe.isEmpty():
        #get node and set of actions
        state = fringe.pop()
        leaf_node = state[0]
        actions = state[1]
        #if node is goal then return set of actions
        if problem.isGoalState(leaf_node):
            return actions
        #if node not in closed set, expand children and add to fringe
        if leaf_node not in closed_set:
            closed_set.append(leaf_node)
            successors = problem.getSuccessors(leaf_node)
            for successor in successors:
                successor_node = successor[0]
                action = successor[1]
                if successor_node not in closed_set:
                    action_path = actions + [action]
                    fringe.push((successor_node, action_path))

    return actions        

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    
    closed_set = []
    actions = []
    fringe = util.Queue()
    #add node and set of actions into the fringe
    fringe.push((problem.getStartState(), actions))

    #run while fringe is not empty
    while not fringe.isEmpty():
        #get node and set of actions
        state = fringe.pop()
        leaf_node = state[0]
        actions = state[1]
        #if node is goal then return set of actions
        if problem.isGoalState(leaf_node):
            return actions
        #if node not in closed set, expand children and add to fringe
        if leaf_node not in closed_set:
            closed_set.append(leaf_node)
            successors = problem.getSuccessors(leaf_node)
            for successor in successors:
                successor_node = successor[0]
                action = successor[1]
                if successor_node not in closed_set:
                    action_path = actions + [action]
                    fringe.push((successor_node, action_path))

    return actions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    
    closedSet = []
    actions = []
    path_cost = 0
    fringe = util.PriorityQueue()
    #add node and set of actions into the fringe with priority
    fringe.push((problem.getStartState(), actions), path_cost)

    #run while fringe is not empty
    while not fringe.isEmpty():
        #get node and set of actions
        state = fringe.pop()
        leaf_node = state[0]
        actions = state[1]
        #if node is goal then return set of actions
        if problem.isGoalState(leaf_node):
            return actions
        #if node not in closed set, expand children and add to fringe
        if leaf_node not in closedSet:
            closedSet.append(leaf_node)
            successors = problem.getSuccessors(leaf_node)
            for successor in successors:
                successor_node = successor[0]
                action = successor[1]
                action_path = actions + [action]
                #get get cost of actions for path
                path_cost = problem.getCostOfActions(action_path)
                #if not in closed set insert else update priority of existing
                if successor_node not in closedSet:
                    fringe.push((successor_node, action_path), path_cost)
                else:
                    fringe.update((successor_node, action_path), path_cost)

    return actions

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    closedSet = []
    actions = []
    #get heuristic for start state
    path_cost  = heuristic(problem.getStartState(), problem)
    fringe = util.PriorityQueue()
    #add node and set of actions into the fringe with priority
    fringe.push((problem.getStartState(), actions), path_cost)

    #run while fringe is not empty
    while not fringe.isEmpty():
        #get node and set of actions
        state = fringe.pop()
        leaf_node = state[0]
        actions = state[1]
        #if node is goal then return set of actions
        if problem.isGoalState(leaf_node):
            return actions
        #if node not in closed set, expand children and add to fringe
        if leaf_node not in closedSet:
            closedSet.append(leaf_node)
            successors = problem.getSuccessors(leaf_node)
            for successor in successors:
                successor_node = successor[0]
                action = successor[1]
                action_path = actions + [action]
                #get get cost of actions for path and the heuristic value
                path_cost = problem.getCostOfActions(action_path) + heuristic(successor_node, problem)
                #if not in closed set insert else update priority of existing
                if successor_node not in closedSet:
                    fringe.push((successor_node, action_path), path_cost)
                else:
                    fringe.update((successor_node, action_path), path_cost)

    return actions

def bidirectionalSearchMM0(problem):
    closed_set_f, actions_f, closed_set_b, actions_b = {}, [], {}, [] 
    fringe_f, fringe_b = util.Queue(), util.Queue()

    #add node and set of actions into the fringe
    fringe_f.push((problem.getStartState(), actions_f))
    fringe_b.push((problem.goal, actions_b))
    
    #run while fringe is not empty
    while not fringe_f.isEmpty() and not fringe_b.isEmpty():      
        #get node and set of actions for forward direction
        leaf_node_f, actions_f = fringe_f.pop()
        #if node is in closed_set_b then return set of actions
        if problem.isBiderectionalGoalState(leaf_node_f, closed_set_b):
            return actions_f + reverseDirections(closed_set_b[leaf_node_f][:-1])

        #if node not in closed set, expand children and add to fringe
        if leaf_node_f not in closed_set_f:
            for successor_node, action, cost in problem.getSuccessors(leaf_node_f):
                if successor_node not in closed_set_f:
                    fringe_f.push((successor_node, actions_f + [action]))
                    closed_set_f[leaf_node_f] = actions_f + [action] 
    
        #get node and set of actions for backward direction
        leaf_node_b, actions_b = fringe_b.pop()
        #if node is in closed_set_f then return set of actions
        if problem.isBiderectionalGoalState(leaf_node_b, closed_set_f):
            return closed_set_f[leaf_node_b] + reverseDirections(actions_b[:-1])

        #if node not in closed set, expand children and add to fringe
        if leaf_node_b not in closed_set_b:
            for successor_node, action, cost in problem.getSuccessors(leaf_node_b):
                if successor_node not in closed_set_b:
                    fringe_b.push((successor_node, actions_b + [action]))
                    closed_set_b[leaf_node_b] = actions_b + [action]

    return []

def bidirectionalSearchMM(problem, heuristic=nullHeuristic):
    closed_set_f, actions_f, closed_set_b, actions_b = {}, [], {}, []
    fringe_f = util.PriorityQueue()
    fringe_b = util.PriorityQueue()
     
    #get heuristic for start state
    path_cost_f  = heuristic(problem.getStartState(), problem, True)
    path_cost_b  = heuristic(problem.goal, problem, False)
    #add node and set of actions into the fringe with priority
    fringe_f.push((problem.getStartState(), actions_f), path_cost_f)
    fringe_b.push((problem.goal, actions_b), path_cost_b)

    #run while fringe is not empty
    while not fringe_f.isEmpty() and not fringe_b.isEmpty():
        #get node and set of actions for forward direction
        leaf_node_f, actions_f = fringe_f.pop()
        #if node is goal then return set of actions
        if problem.isBiderectionalGoalState(leaf_node_f, closed_set_b):
            return actions_f + reverseDirections(closed_set_b[leaf_node_f][:-1])

        #if node not in closed set, expand children and add to fringe
        if leaf_node_f not in closed_set_f:
            for successor_node, action, cost in problem.getSuccessors(leaf_node_f):
                action_path = actions_f + [action]
                #if successor_node_f not in closed set insert/update priority
                if successor_node not in closed_set_f:
                    #get cost of actions for path and the heuristic value
                    path_cost_f = problem.getCostOfActions(action_path) + heuristic(successor_node, problem, True)
                    fringe_f.push((successor_node, action_path), path_cost_f)
                    closed_set_f[leaf_node_f] = action_path

        #get node and set of actions for backward direction
        leaf_node_b, actions_b = fringe_b.pop()
        #if node is goal then return set of actions
        if problem.isBiderectionalGoalState(leaf_node_b, closed_set_f):
            return closed_set_f[leaf_node_b] + reverseDirections(actions_b[:-1])
            
        #if node not in closed set, expand children and add to fringe
        if leaf_node_b not in closed_set_b:
            for successor_node, action, cost in problem.getSuccessors(leaf_node_b):
                action_path_b = actions_b + [action]
                #if successor_node_b not in closed set insert/update priority
                if successor_node not in closed_set_b:
                    #get cost of actions for path and the heuristic value
                    path_cost_b = problem.getCostOfActions(action_path_b) + heuristic(successor_node, problem, False)
                    fringe_b.push((successor_node, action_path_b), path_cost_b)
                    closed_set_b[leaf_node_b] = action_path_b

    return []

def reverseDirections(actions):
    newActions = []
    for action in actions:
        if action == 'West':
            newActions.append('East')
        elif action == 'East':
            newActions.append('West')
        elif action == 'South':
            newActions.append('North')
        elif action == 'North':
            newActions.append('South')
    newActions = newActions[::-1]
    return newActions

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
mm0 = bidirectionalSearchMM0
mm = bidirectionalSearchMM
