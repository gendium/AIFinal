# myTeam.py
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


from capture_agents import CaptureAgent
import random, time, util
from game import Directions
import game
import util
from util import Queue

#################
# Team creation #
#################

def create_team(firstIndex, secondIndex, isRed,
               first = 'Top', second = 'Bottom'):
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

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def register_initial_state(self, game_state):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).
    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)
    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.register_initial_state in captureAgents.py.
    '''
    
    CaptureAgent.register_initial_state(self, game_state)
    
    '''
    Your initialization code goes here, if you need any.
    '''
    self.behaviour_state = 'Guard'
    self.setCenter(game_state)
    self.eatenFood = 0
    self.prevFoodState = self.get_food_you_are_defending(game_state)
    self.opponentIndices = self.get_opponents(game_state)
    self.teamIndices = self.get_team(game_state)

    self.teammateIndex = self.get_team(game_state)[:]
    self.teammateIndex.remove(self.index)

    self.defenceDestination = None
    self.attackDestination = None
    self.opponent_positions = {}
    self.opponentPrevPositions = {}
    self.opponentDetected = None
    for opponent_index in self.opponentIndices:
      self.opponent_positions[opponent_index] = None
      self.opponentPrevPositions[opponent_index] = None

  def updateDefenceDestination(self,game_state):
    if self.destination_reached(game_state, self.defenceDestination):
      self.defenceDestination = self.opponentDetected
    else:
      if self.opponentDetected != None:
        self.defenceDestination = self.opponentDetected
      else:
        if self.defenceDestination == None:
          return 
        elif not self.inHomeTerritory(game_state,self.defenceDestination,0):
          self.defenceDestination = None
        else: 
          return 

  def updateOpponentDetected(self,game_state):
    opponent_positions = self.get_opponent_positions_list(game_state)
    food_eaten_by_opponent = self.food_eaten_by_opponent(game_state)
    if len(opponent_positions)<2 and len(food_eaten_by_opponent)>0:
      if len(opponent_positions) == 0:
        opponent_positions = food_eaten_by_opponent
      else:
        for opEatFood in food_eaten_by_opponent:
          for opponentPosition in opponent_positions:
            if self.get_maze_distance(opEatFood,opponentPosition) > 1:
              opponent_positions = opponent_positions + [opEatFood]
    if len(opponent_positions) == 1:
      if self.closestTeammember(game_state, opponent_positions[0])[0] == self.index:
        self.opponentDetected = opponent_positions[0]
        return
      else:
        self.opponentDetected = None
        return
    elif len(opponent_positions) > 1:
      min_distance = 99999999
      for position in opponent_positions:
        index, distance = self.closestTeammember(game_state,position)
        if distance < min_distance:
            min_distance = distance
            minPosition = position
            min_index = index
      if min_index == self.index:
        self.opponentDetected = minPosition
        return
      else:
        for position in opponent_positions:
          if not position == minPosition:
            self.opponentDetected =  position
            return
    else:
      self.opponentDetected =  None

  def update_opponent_positions(self, game_state):
    self.opponentPrevPositions = self.opponent_positions.copy()
    self.opponent_positions = self.get_opponent_dicts(game_state)

  def get_opponent_dicts(self, game_state):
    opponent_positionsDict = {}
    for index in self.opponentIndices:
      opponent_positionsDict[index] = game_state.get_agent_position(index)
    return opponent_positionsDict

  def get_opponent_positions_list(self, game_state):
    opponent_positionsList = []
    for index in self.opponentIndices:
      if not game_state.get_agent_position(index) == None:
        opponent_positionsList = opponent_positionsList + [game_state.get_agent_position(index)]
    return opponent_positionsList

  def destination_reached(self,game_state,destination):
    if destination == None:
      return False
    return self.get_maze_distance(game_state.get_agent_position(self.index),destination) == 0

  def killed_opponent(self, game_state,index):
    for opponent_index in self.opponentIndices:
      if self.opponent_positions[opponent_index] == game_state.get_initial_agent_position(opponent_index):
        return True
      elif not self.opponentPrevPositions[opponent_index] == None:
        if self.opponent_positions[opponent_index] == None and util.manhattan_distance(game_state.get_agent_position(index), self.opponentPrevPositions[opponent_index])<2:
          return True
    return False
    
  def opponent_is_dead(self, game_state):
    for team_index in self.teamIndices:
      if self.killed_opponent(game_state,team_index):
        return True
    return False

  def should_i_attack(self,game_state):
    min_distance = 99999999
    min_team_index = None
    if self.opponent_is_dead(game_state):
      for opponent_index in self.opponentIndices:
        opponentPosition = game_state.get_agent_position(opponent_index)
        if opponentPosition != None:
          team_index,distance = self.closestTeammember(game_state, opponentPosition)
          if distance < min_distance:
            min_distance = distance
            min_team_index = team_index
    else:
      return False

    if min_team_index == None:
      return self.index == min(self.teamIndices)
    elif(self.index) != min_team_index:
      return True
    return False

  def isDead(self, game_state):
    if self.get_maze_distance(game_state.get_agent_position(self.index),game_state.get_initial_agent_position(self.index)) <= 2:
      return True 
    return False

  def tooMuchFood(self):
    if self.eatenFood > 3:
      return True
    return False

  def resetFoodCount(self):
    self.eatenFood = 0

  def nextbehaviour_state(self,game_state):
    self.update_opponent_positions(game_state)
    self.updateOpponentDetected(game_state)
    self.updateDefenceDestination(game_state)
   
    if game_state.get_agent_state(self.index).scared_timer>10:
      self.behaviour_state = 'Offence'

    elif self.behaviour_state == 'Guard':
      if not self.defenceDestination == None:
        self.behaviour_state = 'Defence'
      elif self.should_i_attack(game_state):
         self.behaviour_state = 'Offence'
      return
     
    elif self.behaviour_state == 'Defence':
      if game_state.get_agent_state(self.index).is_pacman:
        self.behaviour_state = 'Flee'
      elif self.should_i_attack(game_state):
        self.behaviour_state = 'Offence'
      elif self.defenceDestination == None:
        self.behaviour_state = 'Guard'
      return

    elif self.behaviour_state == 'Offence':
      if self.tooMuchFood() or (self.nearestGhostDistance(game_state) <= 3 and game_state.get_agent_state(self.index).is_pacman):
        self.behaviour_state = 'Flee'
      elif not self.defenceDestination == None:# and self.inHomeTerritory(game_state,game_state.get_agent_position(self.index),0):
        self.behaviour_state = 'Defence'
      elif self.isDead(game_state):
        self.resetFoodCount()
        self.behaviour_state = 'Guard'
      return

    elif self.behaviour_state == 'Flee':
      if self.inHomeTerritory(game_state, game_state.get_agent_position(self.index),0) or self.isDead(game_state):
        self.resetFoodCount()
        self.behaviour_state = 'Guard'
      return

    else:
      self.update_opponent_positions(game_state)
      self.behaviour_state = 'Guard'
    
  def choose_action(self, game_state):
    self.nextbehaviour_state(game_state)
    if self.behaviour_state == 'Guard':
      return self.chooseGuardAction(game_state)
    elif self.behaviour_state == 'Defence':
      return self.chooseDefensiveAction(game_state)
    elif self.behaviour_state == 'Offence':
      return self.chooseOffensiveAction(game_state)
    elif self.behaviour_state == 'Flee':
      return self.chooseFleeAction(game_state)
    else:
      return Directions.STOP
    
  def getSuccessor(self, game_state, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = game_state.generate_successor(self.index, action)
    pos = successor.get_agent_state(self.index).get_position()
    if pos != util.nearest_point(pos):
      # Only half a grid position was covered
      return successor.generate_successor(self.index, action)
    return successor

  def closestTeammember(self, game_state, position):
    min_distance = 99999
    for index in self.teamIndices:
      distance = self.get_maze_distance(game_state.get_agent_position(index), position)
      if distance < min_distance:
        min_distance = distance
        min_index = index
    return min_index,min_distance
      
  ###### 'GUARD' BEHAVIOUR CODE ######
  
  def chooseGuardAction(self, game_state):
    # use greedyBFS to get to middle
      # one goes top, one goes bottom (see 'Top' and 'Bottom' classes) 
    actions = game_state.get_legal_actions(self.index)
    values = [self.evaluateGuard(game_state, a) for a in actions]
    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    return random.choice(bestActions)
      
  def evaluateGuard(self, game_state, action):
    # same as base evaluate function really (see baselineTeam.py)
    features = self.getGuardFeatures(game_state, action)
    weights = self.getGuardWeights(game_state, action)
    return features * weights

  def getGuardFeatures(self, game_state, action):
    # distanceToCenter
    features = util.Counter()
    successor = self.getSuccessor(game_state, action)
    successorState = successor.get_agent_state(self.index)
    successorPos = successorState.get_position()
    min_distance = 99999999
    if self.get_maze_distance(successorPos,self.center) < min_distance:
      #bestAction = action
      min_distance = self.get_maze_distance(successorPos,self.center)
    features['distanceToCenter'] = min_distance
    return features

  def getGuardWeights(self, game_state, action):
    return {'distanceToCenter': -1}
    
  ###### 'OFFENCE' BEHAVIOUR CODE ######
  
  def chooseOffensiveAction(self, game_state):
    # get a list of actions for MonteCarloSearch()
    actions = game_state.get_legal_actions(self.index)
    actions.remove(Directions.STOP)
    minAll = 99999999999999
    maxAll = -99999999999999
    values = []
    for a in actions:
      successor = self.getSuccessor(game_state, a)
      #monValues = self.MonteCarloSearch(8, successor, 40)
      monValues = self.MonteCarloSearch(5, successor, 50)
      value = sum(monValues)
      values.append(value)
    if not self.FoodInProximity(game_state):
      min_distance = 999999999
      foodList = self.get_food(game_state).as_list()
      for food in foodList:
        distance = self.get_maze_distance(game_state.get_agent_position(self.index),food)
        if distance<min_distance:
          min_distance = distance
          minFood = food
      min_distance = 999999999
      for action in actions:
        position = self.getSuccessor(game_state, action).get_agent_state(self.index).get_position()
        distance = self.get_maze_distance(position,minFood)
        if distance<min_distance:
          min_distance = distance
          minAction = action
      successor = self.getSuccessor(game_state, minAction)
      foodList = self.get_food(game_state).as_list()
      successorFoodList = self.get_food(successor).as_list()
      if len(successorFoodList) < len(foodList):
        self.eatenFood += 1
      return minAction
    else:
    # choose action with best value
      maxValue = max(values)
      bestActions = [a for a, v in zip(actions, values) if v == maxValue]
      #print 'bestActions:', bestActions
      choice = random.choice(bestActions)
      successor = self.getSuccessor(game_state, choice)
      foodList = self.get_food(game_state).as_list()
      successorFoodList = self.get_food(successor).as_list()
      if len(successorFoodList) < len(foodList):
        self.eatenFood += 1
      return choice
    
  def FoodInProximity(self,game_state):
    foodList = self.get_food(game_state).as_list()
    if len(foodList) > 0:
      min_distance = min([self.get_maze_distance(game_state.get_agent_state(self.index).get_position(), food)
                         for food in foodList])
      if min_distance>8:
        return False
      return True
    return False

  def MonteCarloSearch(self, depth, game_state, iterations):
    # define a game_state that we will iteratively search through
    searchState = None

    # get the distance to the nearest food
    foodList = self.get_food(game_state).as_list()
    if len(foodList) > 0:
      min_distance = min([self.get_maze_distance(game_state.get_agent_state(self.index).get_position(), food)
                         for food in foodList])

    # keep track of discovered endstates
    endStates = []
    # do random searches for the number of iterations defined
    while iterations > 0:
      searchState = game_state.deep_copy()
      #print searchState.get_agent_state(self.index).get_position()
      # if min_distance = 0, we want the action that called MonteCarlo
      if min_distance == 0:
        endStates.append(game_state)
      # otherwise commit to random searches for depth specified
      else:
        tree = depth
        while tree > 0:
          actions = searchState.get_legal_actions(self.index)
          # stopping is a waste of time
          actions.remove(Directions.STOP)
          
          # reversing direction is also a waste of time
          rev = Directions.REVERSE[searchState.get_agent_state(self.index).configuration.direction]
          if rev in actions and len(actions) > 1:
            actions.remove(rev)
          
          action = random.choice(actions)
          #print(action)
          searchState = self.getSuccessor(searchState, action)

          tree -= 1
        endStates.append(searchState)
        #print searchState.get_agent_state(self.index).get_position()
      iterations -= 1
    # return values (to chooseOffensiveAction)
    maxval = -100000
    pls = None
    for endState in endStates:
      if self.evaluateOffensive(endState) > maxval:
          maxval = self.evaluateOffensive(endState)
          pls = self.getOffensiveFeatures(endState)
    return [self.evaluateOffensive(endState) for endState in endStates]

  def nearestGhostDistance(self,game_state):
    min_distance = 999999
    for index in self.opponentIndices:
      if self.opponent_positions[index] != None:
        oppState = game_state.get_agent_state(index)
        distance = self.get_maze_distance(self.opponent_positions[index],game_state.get_agent_position(self.index))
        
        if game_state.get_agent_state(index).scared_timer > 0:
          distance = distance*1000
        if oppState.is_pacman:
          distance = distance*1000
        if distance < min_distance:
          min_distance = distance
    return min_distance
  
  def evaluateOffensive(self, game_state):
    # same as base evaluate function really (see baselineTeam.py)
    features = self.getOffensiveFeatures(game_state)
    weights = self.getOffensiveWeights(game_state)
    #print features * weights
    return features * weights
  
  def getOffensiveFeatures(self, game_state):
    # distancetofood, foodremaining?, ghost?, capsule?/distancetocapsule?
    features = util.Counter()

    foodList = self.get_food(game_state).as_list()
    features['stateScore'] = -len(foodList)

    myPos = game_state.get_agent_state(self.index).get_position()
    betterFoodList = [f for f in foodList if self.get_maze_distance(myPos, f) <= 8]
    #print betterFoodList
    sumFoods = 0
    sumDistance = 0
    for food in betterFoodList:
      sumFoods += 1
      sumDistance += self.get_maze_distance(myPos, food)
    features['numFoods'] = sumFoods
    features['sumDistanceToFood'] = sumDistance

    #Calculate Distance to nearest ghost
    min_distance = 999999
    for index in self.opponentIndices:
      if self.opponent_positions[index] != None:
        oppState = game_state.get_agent_state(index)
        distance = self.get_maze_distance(self.opponent_positions[index],game_state.get_agent_position(self.index))
        
        if game_state.get_agent_state(index).scared_timer > 0:
          distance = distance*1000
        if oppState.is_pacman:
          distance = distance*1000
        if distance < min_distance:
          min_distance = distance
    if min_distance == 0:
      min_distance = 0.01
    if min_distance < 6:
      features['closestEnemy'] = 5 - min_distance #float(1)/(5-min_distance**0.5)
    else:
      features['closestEnemy'] = 0 #float(1)/(5**0.5)

    distance = self.get_maze_distance(game_state.get_agent_position(self.teammateIndex[0]),game_state.get_agent_position(self.index))
    if distance > 0:
      features['teammateDistance'] = float(1)/distance
    else:
      features['teammateDistance'] = 5

    capsules = self.get_capsules(game_state)
    min_distance = 9999999
    for capsule in capsules:
      distance =self.get_maze_distance(game_state.get_agent_position(self.index), capsule)
      if distance < min_distance:
        min_distance = distance
    if min_distance == 0:
      min_distance = 0.01
    if min_distance > 1000:
      features['closestCapsuleDistance'] = 1
    else: 
      features['closestCapsuleDistance'] = float(1)/min_distance

    # distance = self.get_maze_distance(game_state.get_agent_position(self.teammateIndex[0]),game_state.get_agent_position(self.index))
    # if distance > 0:
    #   features['teammateDistance'] = float(1)/distance
    # else:
    #   features['teammateDistance'] = 5

    return features

  def getOffensiveWeights(self, game_state):
    # what weights? check other implementations for a rough idea
    #return {'stateScore': 60, 'numFoods': 60, 'sumDistanceToFood': -5, 'closestEnemy': -400, 'teammateDistance': -30,'closestCapsuleDistance': 40}
    return {'stateScore': 60, 'numFoods': 60, 'sumDistanceToFood': -5, 'closestEnemy': -10, 'teammateDistance': -90,'closestCapsuleDistance': 80}
 
  ###### 'DEFENCE' BEHAVIOUR CODE ######

  def inHomeTerritory(self,game_state,position,offset):
    homeX = game_state.get_walls().width/2
    if self.red:
      homeX = homeX - (1+offset)
    else:
      homeX = homeX + offset

    if self.red and position[0] > homeX:
      return False
    elif not self.red and position[0] < homeX:
      return False
    else:
      return True

  def chooseDefensiveAction(self, game_state):
    # get a list of actions
    actions = game_state.get_legal_actions(self.index)
    actions.remove(Directions.STOP)
    for action in actions:
      successor = self.getSuccessor(game_state,action)
      successorState = successor.get_agent_state(self.index)
      successorPos = successorState.get_position()
      if not self.inHomeTerritory(game_state,successorPos,0) and not game_state.get_agent_state(self.index).is_pacman:
        actions.remove(action)
    # get a list of values (call evaluate?) OR call evaluateDefensive
      # evaluate defensive features/weights
    values = [self.evaluateDefensive(game_state, a) for a in actions]
    # choose action with best value
    if len(values) > 0:
      maxValue = max(values)
    else:
      return Directions.STOP
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    return random.choice(bestActions)

  def evaluateDefensive(self, game_state, action):
    # same as base evaluate function really (see baselineTeam.py)
    features = self.getDefensiveFeatures(game_state, action)
    weights = self.getDefensiveWeights(game_state, action)
    return features * weights

  def getDefensiveFeatures(self, game_state, action):
    # distanceToCenter
    features = util.Counter()
    successor = self.getSuccessor(game_state, action)
    successorState = successor.get_agent_state(self.index)
    successorPos = successorState.get_position()
    min_distance = 99999999
    if (not self.defenceDestination == None) and self.get_maze_distance(successorPos,self.defenceDestination) < min_distance:
      min_distance = self.get_maze_distance(successorPos,self.defenceDestination)
    features['distanceToCenter'] = min_distance
    return features

  def getDefensiveWeights(self, game_state, action):
    #
    return {'distanceToCenter': -1}

  def food_eaten_by_opponent(self, game_state):
    food_eaten_by_opponent = []
    for x in range(game_state.get_walls().width):
      for y in range(game_state.get_walls().height):
        if self.prevFoodState[x][y] == True and self.get_food_you_are_defending(game_state)[x][y] == False and self.inHomeTerritory(game_state, (x,y), 0):
          food_eaten_by_opponent = food_eaten_by_opponent + [(x,y)]
    self.prevFoodState = self.get_food_you_are_defending(game_state)
    return food_eaten_by_opponent

  ###### 'FLEE' BEHAVIOUR CODE ######

  def chooseFleeAction(self, game_state):
    #
    q = Queue()
    q.push((game_state, []))
    visited = []
    i = 0
    while not q.is_empty():
      i = i+1
      state, route = q.pop()
      if self.nearestGhostDistance(state) <= 1 and state != game_state:
        continue
      elif state.get_agent_position(self.index) in visited:
        continue
      elif self.inHomeTerritory(state,state.get_agent_position(self.index),0):
        if len(route) == 0:
          return Directions.STOP
        else:
          return route[0]
      visited = visited + [state.get_agent_position(self.index)]
      actions = state.get_legal_actions(self.index)
      rev = Directions.REVERSE[state.get_agent_state(self.index).configuration.direction]
      if rev in actions and len(actions) > 1 and i!=1:
        actions.remove(rev)
      for action in actions:
        q.push((self.getSuccessor(state,action),route+[action]))
    return random.choice(game_state.get_legal_actions(self.index))

#########################################################################33


class Top(DummyAgent):
# go top somehow
  def setCenter(self,game_state):
    #get center of map and maxHeight

    x = int(game_state.get_walls().width/2)
    offset = 1
    if self.red:
      x = x - (1+offset)
    else:
      x = x + offset
    y = game_state.get_walls().height/2
    y_max = game_state.get_walls().height
    yCenter = int(round(y_max/4*3))
    for i in range(0,y_max):
      y_candidate = yCenter+i
      if y_candidate <= y_max and y_candidate > 0:
        if not game_state.has_wall(x,y_candidate):
          break
      y_candidate = yCenter-i
      if y_candidate <= y_max and y_candidate > 0:
        if not  game_state.has_wall(x,y_candidate):
          break
    self.center = (x,y_candidate)

class Bottom(DummyAgent):
# go bottom somehow
  def setCenter(self,game_state):
    #get center of map and maxHeight
    x = int(game_state.get_walls().width/2)
    offset = 1
    if self.red:
      x = x - (1+offset)
    else:
      x = x + offset
    y = game_state.get_walls().height/2
    y_max = game_state.get_walls().height
    yCenter = int(round(y_max/4))
    for i in range(0,y_max):
      y_candidate = yCenter+i
      if y_candidate <= y_max and y_candidate > 0:
        if not  game_state.has_wall(x,y_candidate):
          break
      y_candidate = yCenter-i
      if y_candidate <= y_max and y_candidate > 0:
        if not  game_state.has_wall(x,y_candidate):
          break
    self.center = (x,y_candidate)