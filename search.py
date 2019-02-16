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

def generalFunction(problem, datastructure, visitedArray):
    """
        A general searchin function for the breadth-first and depth-first searches.
        We realized that the underlying root of the BFS and DFS functions are the same,
        and what really makes them different is the type of data structure they use.
        
        Therefore we created a general function which takes in the data structure as a parameter
        to reuse the same algorithm for BFS and DFS
    """
    
    #start the search with the start state
    startState = problem.getStartState()

    #adds the root, or the start state to the data structure, and the array of visited arrays to
    #avoid going back and repeating the same nodes
    visitedArray.append( startState )
    datastructure.push( (startState, []) )
    

    #this while loop keep going though all the elements in the data structure and processes them to make the
    #next move
    while not datastructure.isEmpty():
        #pops an element off of the data structure, and creates a tuple with 2 values,
        #the current node, which is the state, and the current movements, which is the actions it could take from
        #that specific position
        (currentNode, currentMovements) = datastructure.pop()
        #also, the successors, or the leaves of the node in the tree, are stored into a variable called children
        children = problem.getSuccessors(currentNode)
        
        #for each possible next move that PacMan could take, it process the potential outcomes
        for nextNode in children:
            #creates variables for the state, aka position, and then the direction that it is in
            state = nextNode[0]
            dir = nextNode[1]
            #checks if the state is in the visited array, as we dont want to go backwards in our search
            if state not in visitedArray:
                #checks if its a goal state or not
                if not problem.isGoalState(state):
                    #the state is not a goal state, so we add this next state to our data structure to proceed forward
                    datastructure.push( (state, currentMovements + [dir] ) )
                    #mark the state as visited
                    visitedArray.append( state )
                else:
                    #otherwise if its a goal state, then move in that direction
                    return currentMovements + [dir]


def generalFunctionU ( problem, startState, datastructure, visitedArray):
    """
     Similar to generalFunction(), generalFucntionU() works for Uniform cost search, and is very similar
     to the general algorithm, just has an element of cost associated with it.
    
    Uniform Cost search works in a very similar way, except the data structure is a priority queue
    We also pass in a start state, as this can be modified before executing the function in a priority queue for
    Uniform cost search
     """
    
    #adds the start state to the data structure, and the array of visited points
    visitedArray.append( startState )
    datastructure.push((startState, [], 0), 0)
    
    #checks if the data structure is empty, keeps running until it is
    while not datastructure.isEmpty():
        #create a tuple from the object that was popped, which includes the current Node, the current potential
        #movements, as well as the current cost of making that movement
        (currentNode, currentMovements,currentCost) = datastructure.pop()
        #also the successors of the current node are stored in a list called children
        children = problem.getSuccessors(currentNode)

        #for every node in children, it goes through and process each one and the decisions to get towards
        #the end goal
        for nextNode in children:
            #takes in the state, the direction that its in, as well as the cost of going in that direction
            state = nextNode[0]
            dir = nextNode[1]
            cost = nextNode[2]
            
            #checks if the state has already been visited to prevent revisiting states
            if state not in visitedArray:
                #checks if its a goal state or not
                if not problem.isGoalState(state):
                    #if its not a goal state, then update and push the tuple of the state, the potential movments
                    #and the new cost to the data structure
                    datastructure.push( (state, currentMovements + [dir] , currentCost + cost), currentCost + cost  )
                    #mark the node as visited
                    visitedArray.append( state )
                else:
                    #if its the goal state, then move towards it
                    return currentMovements + [dir]


def generalFunctionA ( problem, startState, datastructure, visitedArray, heuristic):
    """
        generalFunctionA() is almost exactly the same as generalFunctionU(), except it takes in a heuristic function
        to take into account the heuristic plus the cost to make the next decision
        
        It also takes in a data structure of a priority queue, but only works for A* searching
    """
    
    #marks the current state as visited, as well as pushes the state onto the datastructure
    visitedArray.append( startState )
    datastructure.push((startState, [], 0), 0)
    
    #checks if the data strucuture is empty, and if its not then it takes the actions to get towards the goal
    while not datastructure.isEmpty():
        #creates a tuple off of the popped node off of the datastructure, and assigns it to the currentNode,
        #currentMovements, and currentCost values
        (currentNode, currentMovements,currentCost) = datastructure.pop()
        #gets the list of children of the current node and assigns them to a variable
        children = problem.getSuccessors(currentNode)
        
        #for every potential next node, it process the decision
        for nextNode in children:
            #extracts the state, the direction that its in, and the cost of moving there
            state = nextNode[0]
            dir = nextNode[1]
            cost = nextNode[2]
            
            #checks if the state has not been visited already
            if state not in visitedArray:
                #checks if the state is a goal state
                if not problem.isGoalState(state):
                    #if its not, then add the state, the updated direction, and the updated cost
                    #as well as the heuristic added in as well to the cost
                    datastructure.push( (state, currentMovements + [dir] , currentCost + cost), currentCost + cost + heuristic(state, problem)  )
                    #mark the node as visited
                    visitedArray.append( state )
                else:
                    #if it is, then the function is complete
                    return currentMovements + [dir]

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
    

    
    #the data structure being used is a stack for Depth first search
    DFSstack = util.Stack()
    
    #have to create an array with the marked visited nodes so we visit the same nodes twice
    visitedArray = []

    
    return generalFunction(problem, DFSstack, visitedArray)
    


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    
    #the appropriate data structure for breadth-first search
    BFSqueue = util.Queue()
    #array of visited nodes to avoid visiting same one more than once
    visitedArray = []
    
    return generalFunction(problem, BFSqueue, visitedArray)
    
    

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    
    # the appropriate data structure for uniform cost search
    costPQueue = util.PriorityQueue()
    # gets the start state
    startState = problem.getStartState()
    
    #array of visited nodes to avoid visiting same one more than once
    visitedArray = []
    
    return generalFunctionU(problem, startState, costPQueue, visitedArray)
    


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    # the appropriate data structure for A* search
    aStarPQueue = util.PriorityQueue()
    # gets the start state
    startState = problem.getStartState()
    
    #array of visited nodes to avoid visiting same one more than once
    visitedArray = []
    
    return generalFunctionA(problem, startState, aStarPQueue, visitedArray, heuristic)



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
