from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
from game import Actions
import game
import operator
import signal
from util import nearestPoint

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'DefensiveReflexAgent', second = 'OffensiveReflexAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class MainAgent(CaptureAgent):
  """
  A base class for reflex agents that chooses score-maximizing actions
  """

  def registerInitialState(self, gameState):

      CaptureAgent.registerInitialState(self, gameState)
      self.setTeam(gameState)

      self.start = gameState.getAgentPosition(self.index)
      self.mostlikely = [None] * 4

      self.width = gameState.data.layout.width
      self.height = gameState.data.layout.height
      self.walls = gameState.getWalls()
      self.legalPositions = self.legalPosition(gameState)
      self.goal = self.setInitGoal(gameState)

      # a global counter called distributions which update every gameState
      global distributions
      distributions = [util.Counter()] * gameState.getNumAgents()
      distributions = self.initBeliefs(gameState)

  def legalPosition(self, gameState):
      notWall = []
      for pos in gameState.getWalls().asList(False):
          if pos[0] > 0 and pos[0] < self.width:
              if pos[1] > 0 and pos[1] < self.height:
                  notWall.append(pos)
      return notWall

  def legalPositionBaseSite(self, gameState):
      notWall = []
      for pos in gameState.getWalls().asList(False):
          if self.red:
              if pos[0] > 0 and pos[0] < self.width/2:
                  if pos[1] > 0 and pos[1] < self.height:
                      notWall.append(pos)
          else:
              if pos[0] > self.width/2-1 and pos[0] < self.width:
                  if pos[1] > 0 and pos[1] < self.height:
                      notWall.append(pos)
      return notWall

  def getAdjecentFood(self, p):
      adjecentFoods = []
      xAxis = [p[0]-2,p[0]-1,p[0],p[0]+1,p[0]+2]
      yAxis = [p[1] - 2, p[1] - 1, p[1], p[1] + 1, p[1] + 2]
      for x in xAxis:
          for y in yAxis:
              if (x,y) in self.legalPositions:
                  adjecentFoods.append((x,y))
      return adjecentFoods

  def getAdjNum(self,p,gameState):
      adjNum =  0
      adjecent = self.getAdjecentFood(p)
      for q in adjecent:
         if q in self.getFood(gameState).asList():
            adjNum += 1
      return adjNum

  def getAdjDis(self,p,gameState):
      adjDis = 0
      adjecent = self.getAdjecentFood(p)
      for q in adjecent:
          if len(self.getFood(gameState).asList())!=0:
              if q in self.getFood(gameState).asList():
                  adjDis += self.getMazeDistance(p, q)

      return adjDis

  def setInitGoal(self,gameState):

    #eat the closest food to get score
    minMazeDis = float("inf")

    closetPoint = None
    foodList = self.getFood(gameState).asList()

    if self.red:
        x = int(self.width / 2 - 1)
    else:
        x = int(self.width / 2)
    for p in (self.getFood(gameState).asList()):
        for y in range(self.height):
             if (x,y) in self.legalPositions:
                 if minMazeDis > self.getMazeDistance(p,(x,y)):
                      minMazeDis = self.getMazeDistance(p,(x,y))
                      closetPoint = p
    return closetPoint

  def getGoal(self,gameState):

      ############ Closest Food ##################
      # goalFoodList = []
      # minMazeDis = float("inf")
      # for p in self.getFood(gameState).asList():
      #     if minMazeDis > self.getMazeDistance(p, myPos):
      #         maxMazeDis = self.getMazeDistance(p, myPos)
      #         goalFoodList = []
      #         goalFoodList.append(p)
      #     else:
      #         goalFoodList.append(p)
      # return random.choice(goalFoodList)

      # farthest food
      # goalFoodList = []
      # maxMazeDis = 0
      # for p in self.getFood(gameState).asList():
      #     if maxMazeDis < self.getMazeDistance(p, myPos):
      #         maxMazeDis = self.getMazeDistance(p, myPos)
      #         goalFoodList = []
      #         goalFoodList.append(p)
      #     else:
      #         goalFoodList.append(p)
      # return random.choice(goalFoodList)

      ######### value = [(dis to current agent)+(dis to adjacent food)]/(number of adjacent food)
      weightFoodList = util.PriorityQueue()
      foodList = self.getFood(gameState).asList()

      myPos = gameState.getAgentPosition(self.index)
      for p in foodList:

            dis2Agent = self.getMazeDistance(p,myPos)
            if(dis2Agent > (self.width/2)):
                break
            adjMazeDis = self.getAdjDis(p,gameState)
            adjNum = self.getAdjNum(p,gameState)
            value= (dis2Agent+adjMazeDis)/(adjNum*adjNum)
            weightFoodList.push(p,value)

      while not weightFoodList.isEmpty():
            bestFood = weightFoodList.pop()
            print str(bestFood)
            return bestFood

      return random.choice(foodList)

  def goalValid(self,gameState,goal):
      if (goal == None) |(goal not in self.getFood(gameState).asList()):
          return False
      return True

  def setTeam(self,gameState):
      if self.red:
          return CaptureAgent.registerTeam(self,gameState.getRedTeamIndices())
      else:
          return CaptureAgent.registerTeam(self, gameState.getBlueTeamIndices())

  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    actions = gameState.getLegalActions(self.index)

    # You can profile your evaluation time by uncommenting these lines
    # start = time.time()
    values = [self.evaluate(gameState, a) for a in actions]
    # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]

    foodLeft = len(self.getFood(gameState).asList())

    if foodLeft <= 2:
      bestDist = 9999
      for action in actions:
        successor = self.essor(gameState, action)
        pos2 = successor.getAgentPosition(self.index)
        dist = self.getMazeDistance(self.start,pos2)
        if dist < bestDist:
          bestAction = action
          bestDist = dist
      return bestAction

    return random.choice(bestActions)

  def findAgent(self,agent,gameState):
      posTmp = []
      pos = gameState.getAgentPosition(agent)
      if pos != None:
        posTmp.append([pos,1])
      else:
        for p in self.legalPositions:
            value = distributions[agent][p]
            if value > 0.000005:
                posTmp.append([p,value])

      return posTmp

  def initBeliefs(self,gameState):
      distributions[0][(1, 1)]=1.0
      # All beliefs begin with the agent at its inital position
      for agent in self.getOpponents(gameState):
        for p in self.legalPositions:
            distributions[agent][p] = 1.0
      return distributions

  #update distributions
  def observe(self, agent, noisyDistance, gameState):
      myPos = gameState.getAgentPosition(self.index)
      allPossible = util.Counter()
      for p in self.legalPositions:  # check each legal position
          trueDistance = self.getMazeDistance(p, myPos)  # distance between this point and Pacman
          allPossible[p] += gameState.getDistanceProb(trueDistance, noisyDistance)
      for p in self.legalPositions:
          distributions[agent][p] *= allPossible[p]

  def elapseTime(self, gameState):
        for agent, belief in enumerate(distributions):
            if agent in self.getOpponents(gameState):
                newBeliefs = util.Counter()
                # Checks to see what we can actually see
                pos = gameState.getAgentPosition(agent)
                if pos != None:
                    newBeliefs[pos] = 1.0
                else:
                    # Look at all current beliefs
                    for p in belief:
                        if p in self.legalPositions and belief[p] > 0:
                            # Check that all these values are legal positions
                            newPosDist = self.getDist(p)
                            for x, y in newPosDist:  # iterate over these probabilities
                                newBeliefs[x, y] += belief[p] * newPosDist[x, y]
                                # The new chance is old chance * prob of this location from p
                    if len(newBeliefs) == 0:
                        oldState = self.getPreviousObservation()
                        # just ate an enemy
                        if oldState != None and oldState.getAgentPosition(agent) != None:
                            newBeliefs[oldState.getInitialAgentPosition(agent)] = 1.0
                        else:
                            #initialize
                            for p in self.legalPositions: newBeliefs[p] = 1.0
                distributions[agent] = newBeliefs
                # print beliefs
                #self.displayDistributionsOverPositions(distributions)


  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def getDist(self, p):
        posActions = [(p[0] - 1, p[1]), (p[0] + 1, p[1]), (p[0], p[1] - 1), (p[0], p[1] + 1), (p[0], p[1])]
        actions = []
        for act in posActions:
            if act in self.legalPositions:
                actions.append(act)

        dist = util.Counter()
        for act in actions:
            dist[act] = 1
        return dist

  def backToBase(self,gameState,myPos):
      minDist = float("inf")
      nearestBasePoint = []
      if self.red:
        half = self.width/2 - 1
        height = list(range(self.height))
      else:
        half = self.width / 2
        height = list(range(self.height))
      for y in height:
        if (half,y) in self.legalPositions:
           dist = self.getMazeDistance(myPos,(half,y))
           if dist < minDist:
              minDist = dist
              nearestBasePoint = []
              nearestBasePoint.append((half,y))
           elif dist == minDist:
              nearestBasePoint.append((half,y))
      if len(nearestBasePoint)==1:
         return nearestBasePoint[0]
      if len(nearestBasePoint)!=0:
          return random.choice(nearestBasePoint)
      return gameState.getInitialAgentPosition(self.index)

  def getSuccessors(self, point):
      "Returns successor states, the actions they require, and a cost of 1."
      successors = []
      # self._expanded += 1  # DO NOT CHANGE
      for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          x, y = point
          dx, dy = Actions.directionToVector(direction)
          nextx, nexty = int(x + dx), int(y + dy)
          if not self.walls[nextx][nexty]:
              successors.append(((nextx, nexty), direction))
      return successors

  def goalInBase(self, goal):
        if self.red:
            if goal[0] < self.width / 2:
                return True
        else:
            if goal[0] > self.width / 2 - 1:
                return True
        return False

  def getStartPathCoord(self,gameState,path):
      coord = []
      x, y = gameState.getInitialAgentPosition(self.index)
      for d in path:
          if Directions.NORTH == d:
            y += 1
          if Directions.SOUTH == d:
            y -= 1
          if Directions.WEST == d:
            x -= 1
          if Directions.EAST == d:
            x += 1
          coord.append((x,y))
      return coord

  def getLegalDirections (self,gameState):
        nxtDirection = []
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST,Directions.STOP]:
            (dx,dy) = Actions.directionToVector(direction)
            (x,y) = gameState.getAgentPosition(self.index)
            if (dx+x,dy+y) in self.legalPositions:
                nxtDirection.append(direction)
        return nxtDirection

class OffensiveReflexAgent(MainAgent):

    def registerInitialState(self, gameState):

        CaptureAgent.registerInitialState(self, gameState)
        self.setTeam(gameState)

        self.start = gameState.getAgentPosition(self.index)
        self.mostlikely = [None] * gameState.getNumAgents()

        self.width = gameState.data.layout.width
        self.height = gameState.data.layout.height
        self.walls = gameState.getWalls()
        self.legalPositions = self.legalPosition(gameState)
        self.goal = self.setInitGoal(gameState)
        self.step = 0
        self.pwRest = 0
        self.scoreCarry = 0
        self.capsules = self.getCapsules(gameState)
        self.startPath = self.learn(gameState)
        if self.startPath != None:
            self.pathCoord = self.getStartPathCoord(gameState, self.startPath)
        else:
            self.pathCoord = None

        global distributions
        distributions = [util.Counter()] * gameState.getNumAgents()
        distributions = self.initBeliefs(gameState)

    def getCostOfActions(self,gameState,actions):
        if actions == None: return 999999
        x,y= gameState.getAgentPosition(self.index)
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            foodList = self.getFood(gameState)
            if self.walls[x][y]:
                return 9999
            if (x,y) in foodList:
                if self.scoreCarry < 3 & self.getScore(gameState)>0:
                    cost += -80/self.scoreCarry
                else:
                    cost += -10
            for agent in self.getOpponents(gameState):
                agentPos = self.findAgent(agent,gameState)
                if (len(agentPos) == 1):
                    if (self.getMazeDistance((x,y),agentPos[0][0])<3):
                        if self.pwRest <=8:
                            if not self.goalInBase(agentPos[0][0]):
                              return 999999
                        # else:
                        #     print "NOT SCARED"
                else:
                    for p in agentPos:
                        if p[0] == (x,y):
                            if self.pwRest <= 8:
                                if not self.goalInBase(agentPos[0][0]):
                                    return 1000*p[1]
                            # else:
                            #     print "NOT SCARED"
            cost += 1
        return cost

    def getCostLearn(self,gameState,actions):
        if actions == None: return 999999
        x,y= gameState.getAgentPosition(self.index)
        cost = 0
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

    def aStarSearch(self, gameState, goal):

        startNode = gameState.getAgentPosition(self.index)
        aStarOpen = util.PriorityQueue()
        aStarOpen.push((startNode, []), 0.0)
        aStarVisited = []
        depth = 0
        while not aStarOpen.isEmpty():
            tmpNode = aStarOpen.pop()
            depth += 1
            if(depth > (self.width+ self.height)/2):
                break
            if tmpNode[0] == goal:
                if len(tmpNode[1])>0:
                    return tmpNode[1][0] #direction
            if tmpNode[0] not in aStarVisited:
                aStarVisited.append(tmpNode[0])
                neighbour = self.getSuccessors(tmpNode[0])
                for x in neighbour:
                    if x[0] not in aStarVisited :
                        subPath = list(tmpNode[1])
                        subPath.append(x[1])
                        costG = self.getCostOfActions(gameState,subPath)
                        costH = self.getMazeDistance(tmpNode[0], goal)
                        totalCost = costH+costG
                        aStarOpen.push((x[0],subPath),totalCost)
        return tmpNode[1][0]

    def nullHeuristicSearch(self,gameState,goal):
        startNode = gameState.getAgentPosition(self.index)
        nodeOpen = util.PriorityQueue()
        nodeOpen.push((startNode, []), 0.0)
        nodeVisited = []

        while not nodeOpen.isEmpty():
            tmpNode = nodeOpen.pop()

            if tmpNode[0] == goal:
                if len(tmpNode[1]) > 0:
                    return tmpNode[1]  # direction
            if tmpNode[0] not in nodeVisited:
                nodeVisited.append(tmpNode[0])

                neighbour = self.getSuccessors(tmpNode[0])
                for x in neighbour:
                    if x[0] not in nodeVisited:
                        subPath = list(tmpNode[1])
                        subPath.append(x[1])
                        costG = self.getCostLearn(gameState, subPath)
                        nodeOpen.push((x[0], subPath), costG)

    def learn (self, gameState):
        myPos = gameState.getInitialAgentPosition(self.index)

        if self.red:
            destinationX = int(self.width / 5)
            destinationY = list(range(int(self.height/4),int(self.height*3/4)))
        else:
            destinationX = int(self.width * 4 / 5)
            destinationY = list(range(int(self.height/4),int(self.height*3/4)))

        #find the closest point on the boundary
        minMazeDis = float("inf")
        destination = []
        legalPosAtBase = self.legalPositionBaseSite(gameState)
        for y in destinationY:
            if (destinationX, y) in legalPosAtBase:
                tmpDis = self.getMazeDistance((destinationX, y), myPos)
                if minMazeDis > tmpDis:
                    minMazeDis = tmpDis
                    destination = []
                    destination.append((destinationX, y))
                elif minMazeDis == tmpDis:
                    destination.append((destinationX, y))
        if destination != []:
            dest = random.choice(destination)
        else:
            dest = random.choice(legalPosAtBase)

        return self.nullHeuristicSearch(gameState, dest)

    def printTime(self,start1,start2):
        interval =  time.time()-start1
        if interval > 1:
            print time.time() - start2
            print 'eval time for agent %d: %.4f' % (self.index, time.time() - start1)

    def checkStatus(self,gameState,myPos):
        # check powered
        if myPos in self.capsules:
            self.pwRest = 40

        if self.pwRest > 0:
            self.pwRest -= 1

        # check scoreCarried
        previousState = self.getPreviousObservation()
        if previousState != None:
            if myPos in self.getFood(previousState).asList():
                self.scoreCarry += 1

            if self.goalInBase(myPos):
                self.scoreCarry = 0

    def handler(self,signum,frame):
        raise Exception ()

    def oneStepToWin (self,gameState):
        myPos = gameState.getAgentPosition(self.index)
        disToBase = -1
        if not self.goalInBase(myPos):
            shortCut = self.backToBase(gameState, myPos)
            disToBase = self.getMazeDistance(myPos, shortCut)
            print disToBase == 1
            print self.getFood(gameState).asList()
            print (disToBase == 1) & (len(self.getFood(gameState).asList()) < 3)
            print ((disToBase ==1) & (len(self.getFood(gameState).asList())<3)|(self.step>299))
        return (disToBase ==1) & (len(self.getFood(gameState).asList())<3)

    def chooseAction(self, gameState):
        self.step +=1
        if ((self.oneStepToWin(gameState))|(self.step>299)):
            print "end of game"
            # signal.signal(signal.SIGALRM, self.handler)
            # signal.alarm(60)
            myPos = gameState.getAgentPosition(self.index)
            self.goal = self.backToBase(gameState, myPos)
            print self.goal
            print myPos
            dx = self.goal[0]-myPos[0]
            dy = self.goal[1]-myPos[1]
            bestAction = Actions.vectorToDirection((dx,dy))
            if bestAction in self.getLegalDirections(gameState):
                return bestAction
            else:
                return Directions.STOP
        if self.step < 300:
            # print self.step
            # signal.signal(signal.SIGALRM,self.handler)
            # signal.alarm(1)
            try:
                myPos = gameState.getAgentPosition(self.index)

                start1 = time.time()

                # update powered and score carried
                self.checkStatus(gameState,myPos)

                self.capsules = self.getCapsules(gameState)
                opponents = self.getOpponents(gameState)

                for agent in opponents:
                    distributions[agent].normalize()
                    self.mostlikely[agent] = max(distributions[agent].iteritems(), key=operator.itemgetter(1))[0]
                    threshold = (self.width+self.height)/4
                    if (gameState.getAgentDistances()[agent] < 20) & \
                            (self.getMazeDistance(gameState.getAgentPosition(self.index),gameState.getInitialAgentPosition(self.index))>threshold):
                        self.observe(agent, gameState.getAgentDistances()[agent], gameState)

                self.elapseTime(gameState)
                foodLeft = len(self.getFood(gameState).asList())

                start2 = time.time()

                if myPos == gameState.getInitialAgentPosition(self.index):
                    self.pathLength = len(self.startPath)

                if self.pathLength > 0:
                    index =  len(self.startPath)-self.pathLength
                    coord = self.pathCoord
                    # print str(coord)
                    i = len(coord)-1
                    enemyPos = []
                    # avoid powered agent on the start path
                    for agent in self.getOpponents(gameState):
                        tmpPos = gameState.getAgentPosition(agent)
                        if tmpPos != None:
                            enemyPos.append(tmpPos)
                    while i > index:
                        # if ((coord[i] in enemyPos) & (not self.goalInBase(coord[i]))):
                        if (coord[i] in enemyPos) & (not self.goalInBase(coord[i])):
                            # print coord[i]
                            # print enemyPos
                            # print coord[i] in enemyPos
                            self.pathLength += 1
                            if (index == 0):
                                print "stop"
                                return Directions.STOP
                            print "avoid agent on the start path"
                            return Actions.reverseDirection(self.startPath[index-1])
                        i -= 1
                    self.pathLength -= 1
                    return self.startPath[index]

                #back to base as far as possible when only two foodLeft
                if foodLeft <= 2:
                    bestAction = self.aStarSearch(gameState, self.backToBase(gameState, gameState.getAgentPosition(self.index)))
                    if bestAction != None:
                        self.printTime(start1,start2)
                        return bestAction

                # shift to the next goal when agent arrives goal
                if self.goal == None:
                    self.goal = self.getGoal(gameState)
                if gameState.getAgentPosition(self.index) == self.goal:
                    if self.goalInBase(self.goal):
                        print "attack"
                        self.goal = self.getGoal(gameState)
                    else:
                        print "back"
                        self.goal = self.backToBase(gameState,gameState.getAgentPosition(self.index))
                #we are at their base
                if (gameState.getAgentState(self.index).isPacman==True):
                    # our goal is back
                    if self.goalInBase(self.goal):
                        bestAction = self.aStarSearch(gameState, self.goal)
                        if bestAction != None:
                            print "1"+str(bestAction)
                            return bestAction
                    # out goal is attack
                    else:
                        # goal place still has food
                        if self.goalValid(gameState, self.goal):
                            bestAction = self.aStarSearch(gameState, self.goal)
                            if bestAction != None:
                                self.printTime(start1, start2)
                                print "2"+str(bestAction)
                                return bestAction
                        # goal place doesn't has food
                        else:
                            if self.scoreCarry > 0:
                                self.goal = self.getGoal(gameState)
                            else:
                                self.goal = self.backToBase(gameState,myPos)
                            bestAction = self.aStarSearch(gameState, self.goal)
                            if bestAction != None:
                                self.printTime(start1, start2)
                                print "3"+str(bestAction)
                                return bestAction

                # we are at our base
                else:
                    # attack
                    if self.goalInBase(self.goal):
                        if self.getScore(gameState) == 0:
                            self.goal = self.setInitGoal(gameState)
                        else:
                            self.goal = self.getGoal(gameState)
                    # if goal valid
                    if self.goalValid(gameState, self.goal):
                            bestAction = self.aStarSearch(gameState, self.goal)
                            if bestAction != None:
                                self.printTime(start1, start2)
                                print "4"+str(bestAction)
                                return bestAction
                    #get new goal
                    else:
                            if self.getScore(gameState) == 0:
                                self.goal = self.setInitGoal(gameState)
                            else:
                                self.goal = self.getGoal(gameState)
                            bestAction = self.aStarSearch(gameState, self.goal)
                            if bestAction != None:
                                self.printTime(start1, start2)
                                print "5" + str(bestAction)
                                return bestAction

                print "stop" + str(self.goal)
                return random.choice(self.getLegalDirections(gameState))
            except Exception:
                if self.step<301:
                    #print "time out"
                    return random.choice(self.getLegalDirections(gameState))
        return random.choice(self.getLegalDirections(gameState))

class DefensiveReflexAgent(MainAgent):

  #A reflex agent that keeps its side Pacman-free. Again,
  #this is to give you an idea of what a defensive agent
  #could be like.  It is not the best or only way to make
  #such an agent.


  def registerInitialState(self, gameState):

      CaptureAgent.registerInitialState(self, gameState)
      self.setTeam(gameState)

      self.start = gameState.getAgentPosition(self.index)
      self.mostlikely = [None] * gameState.getNumAgents()
      self.width = gameState.data.layout.width
      self.height = gameState.data.layout.height
      self.walls = gameState.getWalls()
      self.legalPositions = self.legalPosition(gameState)
      self.goal = self.setInitGoal(gameState)
      self.pwRest = 0
      self.capsules = self.getCapsules(gameState)
      #self.startPath = self.learn(gameState)
      #self.pathCoord = self.getStartPathCoord(gameState, self.startPath)

      global distributions
      distributions = [util.Counter()] * gameState.getNumAgents()
      distributions = self.initBeliefs(gameState)

  def getSuccessors(self,gameState, point):
      "Returns successor states, the actions they require, and a cost of 1."
      successors = []
      # self._expanded += 1  # DO NOT CHANGE
      for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          x, y = point
          dx, dy = Actions.directionToVector(direction)
          nextx, nexty = int(x + dx), int(y + dy)
          if (nextx,nexty) in self.legalPositionBaseSite(gameState):
              successors.append(((nextx, nexty), direction))
      return successors

  def getRandom(self, gameState):
      pos = []
      width = self.width/2
      height = self.height/2
      if self.red:
        width = width - 1
        if width-2 > 1:
            for x in range(width-2,width-1):
              for y in range(height):
                if (x,y) in self.legalPositionBaseSite(gameState):
                  if (x,y) != gameState.getAgentPosition(self.index):
                   pos.append((x,y))
      else:
        for x in range(width+1,width+2):
            for y in range(height):
                if (x, y) in self.legalPositionBaseSite(gameState):
                  if (x, y) != gameState.getAgentPosition(self.index):
                    pos.append((x, y))
      if len(pos) == 0:
            return random.choice(self.legalPositionBaseSite(gameState))
      bestAction = random.choice(pos)
      return bestAction

  def aStar(self, gameState, goal):

      startNode = gameState.getAgentPosition(self.index)
      aStarOpen = util.PriorityQueue()
      aStarOpen.push((startNode, []), 0.0)
      aStarVisited = []
      depth = 0
      while not aStarOpen.isEmpty():
          depth += 1
          if(depth > (self.width+ self.height)/2):
              break
          tmpNode = aStarOpen.pop()

          if tmpNode[0] == goal:
              if len(tmpNode[1]) > 0:
                  return tmpNode[1][0]  # direction
          if tmpNode[0] not in aStarVisited:
              aStarVisited.append(tmpNode[0])

              neighbour = self.getSuccessors(gameState,tmpNode[0])
              for x in neighbour:
                  if x[0] not in aStarVisited:
                      subPath = list(tmpNode[1])
                      subPath.append(x[1])
                      costG = len(subPath)
                      costH = self.getMazeDistance(tmpNode[0], goal)
                      totalCost = costH + costG
                      aStarOpen.push((x[0], subPath), totalCost)
      return tmpNode[1][0]

  def chooseAction(self, gameState):

    myPos = gameState.getAgentPosition(self.index)
    nonEnemyAtBas = True
    farDists= []

    enemies = [self.getCurrentObservation().getAgentState(i) for i in self.getOpponents(self.getCurrentObservation())]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    farInvaders = [b for b in enemies if b.isPacman and b.getPosition() == None]
    farPacman = []

    if len(invaders) > 0:
        dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
        for i in range(0,len(invaders)) :
            if self.getMazeDistance(myPos,invaders[i].getPosition())  == min(dists) :
                self.goal = invaders[i].getPosition()
        bestAction = self.aStar(gameState, self.goal)
        print "see"
        if bestAction != None:
            return bestAction

    if len(farInvaders) > 0:
        print "not see"
        opponents = self.getOpponents(gameState)
        for agent in opponents:
            if gameState.getAgentState(agent).isPacman:
                distributions[agent].normalize()
                self.mostlikely[agent] = max(distributions[agent].iteritems(), key=operator.itemgetter(1))[0]
                print self.mostlikely[agent]
                farDists.append(self.getMazeDistance(myPos,self.mostlikely[agent]))
                farPacman.append(self.mostlikely[agent])
        for item in farPacman:
            if self.getMazeDistance(myPos,item) == min(farDists):
                if self.red:
                    if item[0]<self.width/2:
                        self.goal = item
                    else :
                        self.goal = self.getRandom(gameState)
                else:
                    if item[0]>self.width/2-1:
                        self.goal = item
                    else :
                        self.goal = self.getRandom(gameState)

        bestAction = self.aStar(gameState,self.goal)
        if bestAction != None:
            return bestAction

    if  nonEnemyAtBas == True :
        self.goal = self.getRandom(gameState)
        bestAction = self.aStar(gameState,self.goal)
        print "random"
        if bestAction != None:
            return bestAction

    return random.choice(self.getLegalDirections(gameState))