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

    dfs_fringe = util.Stack() #Initializing a stack for DFS fringe

    dfs_fringe.push((problem.getStartState(), [])) # Pushing the start state
    visited = set() # Tracking the visited states

    while not dfs_fringe.isEmpty(): #Loop for all the nodes in the stack
        state, path = dfs_fringe.pop() # Removing the most recently added state and path.

        if problem.isGoalState(state): # If state is our goal, then we return the path that got us there
            return path

        if state not in visited: # We explore the state if we haven't yet.
            
            for successor, action, stepCost in problem.getSuccessors(state): # Exploring all possible successors
                if successor not in visited: # Pushing successor if not yet visited
                    dfs_fringe.push((successor, path + [action]))

            visited.add(state) #Marking as visited after searching all successors.

    return [] #If no solution, we return nothing (well, an empty path).

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    bfs_fringe = util.Queue()  # Initializing a queue for BFS fringe

    bfs_fringe.push((problem.getStartState(), [])) #Pushing the start state with an empty path.

    visited = set() #Tracking the visited states
 
    while not bfs_fringe.isEmpty(): #Loop for all the nodes in the queue.
        state, path = bfs_fringe.pop() #Popping the very first state and its path.

        if problem.isGoalState(state): #If the goal is reached, then we return the path for it. Basically if the first node in the queue itself is our goal, then we return its path.
 
            return path
        
        if state not in visited: # Exploring all the other states that have not been visited yet.

            for successor, action, stepCost in problem.getSuccessors(state): 
                if successor not in visited:  #Searching all the successors that haven't been visited yet.
                    bfs_fringe.push((successor, path + [action]))

            visited.add(state) #Marking this state in the loop as visited after reaching.

    return []

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    ucs_fringe = util.PriorityQueue() # Initializing a priority queue.

    ucs_fringe.push((problem.getStartState(), [], 0), 0) # Pushing the start state with the empty path and no cost. Our format is as follows: (state, path, cost till now)

    visited = {} #Tracking the visited states, ranking them with the lowest cost found till now.

    while not ucs_fringe.isEmpty(): # Continuing till there are no more nodes left in the PQ.
 
        state, path, cost = ucs_fringe.pop() # Popping the state with lowest total cost

        if problem.isGoalState(state): # Returning the path if this state itself is te goal. Basically if the first node in the queue itself is our goal, then we return its path. Same as BFS search that we implemented.

            return path 
        
        if state not in visited or cost < visited[state]: # Expanding the state is not yet visited or if we have found a cheaper alternative (With cheaper cost)

            visited[state] = cost # Storing the cheapest cost to reach THIS state.
         
            for successor, action, step_cost in problem.getSuccessors(state): #Exploring all the successors for THIS current state.
                new_path = path + [action] # Moving forward in the path with its action.
                new_cost = cost + step_cost # Calculating totla cost till now.

                ucs_fringe.update((successor, new_path, new_cost), new_cost) # Pushing successors into the fringe with the priority as new_cost.

    return [] #returning an empty path if we dont reach a solution.



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    a_fringe = util.PriorityQueue() #Initializing the priority queue for A*

    a_fringe.push((problem.getStartState(), [], 0), 0 + heuristic(problem.getStartState(), problem)) # Pushing the start state with cost as 0, and priority as g(n) + h(n)

    visited = {} #Tracking the visited states, ranking them with the lowest cost found till now. Just like how we implemented UCS.

    while not a_fringe.isEmpty(): # Looping through all the states.

        state, path, cost = a_fringe.pop() # Popping the state with the lowest priority till now.

        if problem.isGoalState(state): # Returning the path if THIS state itself is the goal.

            return path
        
        if state not in visited or cost < visited[state]: #Searching if the current state is not yet visited or we have found a lower cost alternative.

            visited[state] = cost #Storing the cheapest alternative
            
            for successor, action, step_cost in problem.getSuccessors(state): #Seraching through all the successors.
                new_path = path + [action]  #Moving forward in the path laong with the action for it.
                new_cost = cost + step_cost # Calculating total cost till now.
                priority = new_cost + heuristic(successor, problem) # f(n) + g(n) + h(n)
                a_fringe.update((successor, new_path, new_cost), priority) #Pushing the successor in the fringe with the priority as g(n) + h(n)

    return [] #returning an empty path if we do not reach a solution.


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
