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
    move =[]         # all the information along with action
    closed=[]        # visited place
    action = []      # final return action
    successorList=[] #inorder to avoid using getSuccessor function again due to autograder
    fringe=util.Stack()
    fringe.push((problem.getStartState(),'Stop',0))

    def isNoOtherWay(state):     # function find out if this state have no other successor
        Flag= True   # check if no way to go
        #print("----call if no way:::")
        successor= successorList[-1]
        for successor_state in successor :       # if no way to go
            if successor_state[0] not in closed:
                Flag= False
           # print(state,": ","successor:",successor_state,Flag)
           # print(closed)
        return Flag


 
    while not fringe.isEmpty():
        currentState, direction, cost = fringe.pop()
        closed.append(currentState)
        move.append((currentState,direction,cost))
        action.append(move[-1][1])
        if problem.isGoalState(currentState):         #find the goal
            action.pop(0)
            return action
        successor = problem.getSuccessors(currentState)
        
        successorList.append(successor)

        if isNoOtherWay(currentState):               #check if is the end of way and pop all the wrong state
            while 1:
                if not isNoOtherWay(move[-1][0]):
                    break
                elif isNoOtherWay(move[-1][0]):
                    move.pop()
                    action.pop()
                    successorList.pop()
        for i in range(len(successor)):
            if successor[i][0] not in closed:
                fringe.push(successor[i])
                
    return None

            




def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    fringe= util.Queue()
    path=[]
    closed=[]

    fringe.push(((problem.getStartState(),"Stop",0),path))
    while not fringe.isEmpty():
        state,newpath=fringe.pop()
        closed.append(state[0])                         #state = ((x,y),"direction",cost)    or (((x,y),[0,0,0,0]),"direction",0)
       # print("IN DFS state:",state)
       # print(newpath)
        if problem.isGoalState(state[0]):
            path=newpath
            return path
        else:
            successors=problem.getSuccessors(state[0])
            for successor in successors:
                if successor[0] not in closed:
                    successorPath=newpath[:] #****************不能直接 a=b， 指向同一个内存，会同时修改两个
                    successorPath.append(successor[1])
                    closed.append(successor[0])            #应该在这里就添加，否则有可能这个successor出队列之前又有同一个加进来
                    fringe.push((successor,successorPath))
    return None


def uniformCostSearch(problem):
    fringe= util.PriorityQueue()
    path=[]                             #加一个going to visit？
    closed=[]
    fringe.push(((problem.getStartState(),"Stop",0),path,0),0)      #((successors,path,total_cost),cost)
    while not fringe.isEmpty():
    #for i in range(10):
       # print("------------------",i)
        state,newpath,cost=fringe.pop()
        if state[0] not in closed:
            #print("This State:",state)
            #print("Current path",newpath)
            #print("&&&&&&&&&&&&&&&&&&&&&&&&&&& ")
            closed.append(state[0])                         #state = ((x,y),"direction",cost)
            if problem.isGoalState(state[0]):
                path=newpath
                return path
            else:
                successors=problem.getSuccessors(state[0])
                for successor in successors:
                    if successor[0] not in closed:
                        successorPath=newpath[:] #****************不能直接 a=b， 指向同一个内存，会同时修改两个
                        successorPath.append(successor[1])
                        fringe.push((successor,successorPath,successor[2]+cost),cost+successor[2])
                        #print("Cost",successor[2])
    return None
   
        
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    fringe= util.PriorityQueue()
    path=[]                             #加一个going to visit？
    closed=[]
    fringe.push(((problem.getStartState(),"Stop",0),path,0),0)      #((successors,path,total_cost),cost)
    while not fringe.isEmpty():
    #for i in range(10):
       # print("------------------",i)
        state,newpath,cost=fringe.pop()
        if state[0] not in closed:
            #print("This State:",state)
            #print("Current path",newpath)
            #print("&&&&&&&&&&&&&&&&&&&&&&&&&&& ")
            closed.append(state[0])                         #state = ((x,y),"direction",cost)
            if problem.isGoalState(state[0]):
                path=newpath
                return path
            else:
                successors=problem.getSuccessors(state[0])
                for successor in successors:
                    if successor[0] not in closed:
                        successorPath=newpath[:] #****************不能直接 a=b， 指向同一个内存，会同时修改两个
                        successorPath.append(successor[1])
                        fringe.push((successor,successorPath,successor[2]+cost),cost+successor[2]+heuristic(successor[0],problem))
                        #print("Cost",successor[2])
    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
