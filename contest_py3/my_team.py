# myTeam.py
'''
Author: Sean Egger, Alec Rulev
Class: CSI-480-01
Assignment: Final Project
Due Date: 12/14/2017
 
Description:
A pacman ai capture the flag program
 
Certification of Authenticity: 
I certify that this is entirely my own work, except where I have given 
fully-documented references to the work of others. I understand the definition 
and consequences of plagiarism and acknowledge that the assessor of this 
assignment may, for the purpose of assessing this assignment:
- Reproduce this assignment and provide a copy to another member of academic
- staff; and/or Communicate a copy of this assignment to a plagiarism checking
- service (which may then retain a copy of this assignment on its database for
- the purpose of future plagiarism checking)
'''
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

def create_team(first_index, second_index, is_red,
               first = 'Top', second = 'Bottom'):
  """
  This function should return a list of two agents that will form the
  team, initialized using first_index and second_index as their agent
  index numbers.  is_red is True if the red team is being created, and
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
  return [eval(first)(first_index), eval(second)(second_index)]

##########
# Agents #
##########

class Dummy_Agent(CaptureAgent):
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
    self.set_center(game_state)
    self.eaten_food = 0
    self.prev_food_state = self.get_food_you_are_defending(game_state)
    self.opponent_indices = self.get_opponents(game_state)
    self.team_indices = self.get_team(game_state)

    self.teammate_index = self.get_team(game_state)[:]
    self.teammate_index.remove(self.index)

    self.defence_destination = None
    self.attack_destination = None
    self.opponent_positions = {}
    self.opponent_previous_positions = {}
    self.opponent_detected = None
    for opponent_index in self.opponent_indices:
      self.opponent_positions[opponent_index] = None
      self.opponent_previous_positions[opponent_index] = None
    self.set_guard_mode()

  def update_defence_destination(self,game_state):
    if self.destination_reached(game_state, self.defence_destination):
      self.defence_destination = self.opponent_detected
    else:
      if self.opponent_detected != None:
        self.defence_destination = self.opponent_detected
      else:
        if self.defence_destination == None:
          return 
        elif not self.in_home_territory(game_state,self.defence_destination,0):
          self.defence_destination = None
        else: 
          return 

  def update_opponent_detected(self,game_state):
    opponent_positions = self.get_opponent_positions_list(game_state)
    food_eaten_by_opponent = self.food_eaten_by_opponent(game_state)
    if len(opponent_positions)<2 and len(food_eaten_by_opponent)>0:
      if len(opponent_positions) == 0:
        opponent_positions = food_eaten_by_opponent
      else:
        for opponent_eat_food in food_eaten_by_opponent:
          for opponent_position in opponent_positions:
            if self.get_maze_distance(opponent_eat_food,opponent_position) > 1:
              opponent_positions = opponent_positions + [opponent_eat_food]
    if len(opponent_positions) == 1:
      if self.closest_teammember(game_state, opponent_positions[0])[0] == self.index:
        self.opponent_detected = opponent_positions[0]
        return
      else:
        self.opponent_detected = None
        return
    elif len(opponent_positions) > 1:
      min_distance = 99999999
      for position in opponent_positions:
        index, distance = self.closest_teammember(game_state,position)
        if distance < min_distance:
            min_distance = distance
            min_position = position
            min_index = index
      if min_index == self.index:
        self.opponent_detected = min_position
        return
      else:
        for position in opponent_positions:
          if not position == min_position:
            self.opponent_detected =  position
            return
    else:
      self.opponent_detected =  None

  def update_opponent_positions(self, game_state):
    self.opponent_previous_positions = self.opponent_positions.copy()
    self.opponent_positions = self.get_opponent_dicts(game_state)

  def get_opponent_dicts(self, game_state):
    opponent_position_dict = {}
    for index in self.opponent_indices:
      opponent_position_dict[index] = game_state.get_agent_position(index)
    return opponent_position_dict

  def get_opponent_positions_list(self, game_state):
    opponent_positionsList = []
    for index in self.opponent_indices:
      if not game_state.get_agent_position(index) == None:
        opponent_positionsList = opponent_positionsList + [game_state.get_agent_position(index)]
    return opponent_positionsList

  def destination_reached(self,game_state,destination):
    if destination == None:
      return False
    return self.get_maze_distance(game_state.get_agent_position(self.index),destination) == 0

  def killed_opponent(self, game_state,index):
    for opponent_index in self.opponent_indices:
      if self.opponent_positions[opponent_index] == game_state.get_initial_agent_position(opponent_index):
        return True
      elif not self.opponent_previous_positions[opponent_index] == None:
        if self.opponent_positions[opponent_index] == None and util.manhattan_distance(game_state.get_agent_position(index), self.opponent_previous_positions[opponent_index])<2:
          return True
    return False
    
  def opponent_is_dead(self, game_state):
    for team_index in self.team_indices:
      if self.killed_opponent(game_state,team_index):
        return True
    return False

  def should_i_attack(self,game_state):
    min_distance = 99999999
    min_team_index = None
    if self.opponent_is_dead(game_state):
      for opponent_index in self.opponent_indices:
        opponent_position = game_state.get_agent_position(opponent_index)
        if opponent_position != None:
          team_index,distance = self.closest_teammember(game_state, opponent_position)
          if distance < min_distance:
            min_distance = distance
            min_team_index = team_index
    else:
      return False

    if min_team_index == None:
      return self.index == min(self.team_indices)
    elif(self.index) != min_team_index:
      return True
    return False

  def is_dead(self, game_state):
    if self.get_maze_distance(game_state.get_agent_position(self.index),game_state.get_initial_agent_position(self.index)) <= 2:
      return True 
    return False

  def too_much_food(self):
    if self.eaten_food > 3:
      return True
    return False

  def reset_food_count(self):
    self.eaten_food = 0

  def next_behaviour_state(self,game_state):
    self.update_opponent_positions(game_state)
    self.update_opponent_detected(game_state)
    self.update_defence_destination(game_state)
   
    if game_state.get_agent_state(self.index).scared_timer>10:
      self.behaviour_state = 'Offence'

    elif self.behaviour_state == 'Guard':
      if not self.defence_destination == None:
        self.behaviour_state = 'Defence'
      elif self.should_i_attack(game_state):
         self.behaviour_state = 'Offence'
      return
     
    elif self.behaviour_state == 'Defence':
      if game_state.get_agent_state(self.index).is_pacman:
        self.behaviour_state = 'Flee'
      elif self.should_i_attack(game_state):
        self.behaviour_state = 'Offence'
      elif self.defence_destination == None:
        self.behaviour_state = 'Guard'
      return

    elif self.behaviour_state == 'Offence':
      if self.too_much_food() or (self.nearest_ghost_distance(game_state) <= 3 and game_state.get_agent_state(self.index).is_pacman):
        self.behaviour_state = 'Flee'
      elif not self.defence_destination == None:# and self.in_home_territory(game_state,game_state.get_agent_position(self.index),0):
        self.behaviour_state = 'Defence'
      elif self.is_dead(game_state):
        self.reset_food_count()
        self.behaviour_state = 'Guard'
      return

    elif self.behaviour_state == 'Flee':
      if self.in_home_territory(game_state, game_state.get_agent_position(self.index),0) or self.is_dead(game_state):
        self.reset_food_count()
        self.behaviour_state = 'Guard'
      return

    else:
      self.update_opponent_positions(game_state)
      self.behaviour_state = 'Guard'
    
  def choose_action(self, game_state):
    self.next_behaviour_state(game_state)
    if self.behaviour_state == 'Guard':
      if self.can_guard:
        return self.choose_guard_action(game_state)
      else:
        return self.choose_offensive_action(game_state)
    elif self.behaviour_state == 'Defence':
      return self.choose_defensive_action(game_state)
    elif self.behaviour_state == 'Offence':
      return self.choose_offensive_action(game_state)
    elif self.behaviour_state == 'Flee':
      return self.choose_flee_action(game_state)
    else:
      return Directions.STOP
    
  def get_successor(self, game_state, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = game_state.generate_successor(self.index, action)
    pos = successor.get_agent_state(self.index).get_position()
    if pos != util.nearest_point(pos):
      # Only half a grid position was covered
      return successor.generate_successor(self.index, action)
    return successor

  def closest_teammember(self, game_state, position):
    min_distance = 99999
    for index in self.team_indices:
      distance = self.get_maze_distance(game_state.get_agent_position(index), position)
      if distance < min_distance:
        min_distance = distance
        min_index = index
    return min_index,min_distance
      
  ###### 'GUARD' BEHAVIOUR CODE ######
  
  def choose_guard_action(self, game_state):
    # use greedyBFS to get to middle
      # one goes top, one goes bottom (see 'Top' and 'Bottom' classes) 
    actions = game_state.get_legal_actions(self.index)
    values = [self.evaluate_guard(game_state, a) for a in actions]
    max_value = max(values)
    best_actions = [a for a, v in zip(actions, values) if v == max_value]
    return random.choice(best_actions)
      
  def evaluate_guard(self, game_state, action):
    # same as base evaluate function really (see baselineTeam.py)
    features = self.get_guard_features(game_state, action)
    weights = self.get_guards_weight(game_state, action)
    return features * weights

  def get_guard_features(self, game_state, action):
    # distance_to_center
    features = util.Counter()
    successor = self.get_successor(game_state, action)
    successor_state = successor.get_agent_state(self.index)
    successor_pos = successor_state.get_position()
    min_distance = 99999999
    if self.get_maze_distance(successor_pos,self.center) < min_distance:
      #bestAction = action
      min_distance = self.get_maze_distance(successor_pos,self.center)
    features['distance_to_center'] = min_distance
    return features

  def get_guards_weight(self, game_state, action):
    return {'distance_to_center': -1}
    
  ###### 'OFFENCE' BEHAVIOUR CODE ######
  
  def choose_offensive_action(self, game_state):
    # get a list of actions for montecarlo_search()
    actions = game_state.get_legal_actions(self.index)
    actions.remove(Directions.STOP)
    minAll = 99999999999999
    maxAll = -99999999999999
    values = []
    for a in actions:
      successor = self.get_successor(game_state, a)
      #mon_values = self.montecarlo_search(8, successor, 40)
      mon_values = self.montecarlo_search(5, successor, 50)
      value = sum(mon_values)
      values.append(value)
    if not self.food_in_proximity(game_state):
      min_distance = 999999999
      food_list = self.get_food(game_state).as_list()
      for food in food_list:
        distance = self.get_maze_distance(game_state.get_agent_position(self.index),food)
        if distance<min_distance:
          min_distance = distance
          minFood = food
      min_distance = 999999999
      for action in actions:
        position = self.get_successor(game_state, action).get_agent_state(self.index).get_position()
        distance = self.get_maze_distance(position,minFood)
        if distance<min_distance:
          min_distance = distance
          min_action = action
      successor = self.get_successor(game_state, min_action)
      food_list = self.get_food(game_state).as_list()
      successor_food_list = self.get_food(successor).as_list()
      if len(successor_food_list) < len(food_list):
        self.eaten_food += 1
      return min_action
    else:
    # choose action with best value
      max_value = max(values)
      best_actions = [a for a, v in zip(actions, values) if v == max_value]
      #print 'best_actions:', best_actions
      choice = random.choice(best_actions)
      successor = self.get_successor(game_state, choice)
      food_list = self.get_food(game_state).as_list()
      successor_food_list = self.get_food(successor).as_list()
      if len(successor_food_list) < len(food_list):
        self.eaten_food += 1
      return choice
    
  def food_in_proximity(self,game_state):
    food_list = self.get_food(game_state).as_list()
    if len(food_list) > 0:
      min_distance = min([self.get_maze_distance(game_state.get_agent_state(self.index).get_position(), food)
                         for food in food_list])
      if min_distance>8:
        return False
      return True
    return False

  def montecarlo_search(self, depth, game_state, iterations):
    # define a game_state that we will iteratively search through
    search_state = None

    # get the distance to the nearest food
    food_list = self.get_food(game_state).as_list()
    if len(food_list) > 0:
      min_distance = min([self.get_maze_distance(game_state.get_agent_state(self.index).get_position(), food)
                         for food in food_list])

    # keep track of discovered end_states
    end_states = []
    # do random searches for the number of iterations defined
    while iterations > 0:
      search_state = game_state.deep_copy()
      #print search_state.get_agent_state(self.index).get_position()
      # if min_distance = 0, we want the action that called MonteCarlo
      if min_distance == 0:
        end_states.append(game_state)
      # otherwise commit to random searches for depth specified
      else:
        tree = depth
        while tree > 0:
          actions = search_state.get_legal_actions(self.index)
          # stopping is a waste of time
          actions.remove(Directions.STOP)
          
          # reversing direction is also a waste of time
          rev = Directions.REVERSE[search_state.get_agent_state(self.index).configuration.direction]
          if rev in actions and len(actions) > 1:
            actions.remove(rev)
          
          action = random.choice(actions)
          #print(action)
          search_state = self.get_successor(search_state, action)

          tree -= 1
        end_states.append(search_state)
        #print search_state.get_agent_state(self.index).get_position()
      iterations -= 1
    # return values (to choose_offensive_action)
    max_val = -100000
    pls = None
    for end_state in end_states:
      if self.evaluate_offensive(end_state) > max_val:
          max_val = self.evaluate_offensive(end_state)
          pls = self.get_offensive_features(end_state)
    return [self.evaluate_offensive(end_state) for end_state in end_states]

  def nearest_ghost_distance(self,game_state):
    min_distance = 999999
    for index in self.opponent_indices:
      if self.opponent_positions[index] != None:
        opponent_state = game_state.get_agent_state(index)
        distance = self.get_maze_distance(self.opponent_positions[index],game_state.get_agent_position(self.index))
        
        if game_state.get_agent_state(index).scared_timer > 0:
          distance = distance*1000
        if opponent_state.is_pacman:
          distance = distance*1000
        if distance < min_distance:
          min_distance = distance
    return min_distance
  
  def evaluate_offensive(self, game_state):
    # same as base evaluate function really (see baselineTeam.py)
    features = self.get_offensive_features(game_state)
    weights = self.get_offensive_weights(game_state)
    #print features * weights
    return features * weights
  
  def get_offensive_features(self, game_state):
    # distancetofood, foodremaining?, ghost?, capsule?/distancetocapsule?
    features = util.Counter()

    food_list = self.get_food(game_state).as_list()
    features['state_score'] = -len(food_list)

    my_pos = game_state.get_agent_state(self.index).get_position()
    better_food_list = [f for f in food_list if self.get_maze_distance(my_pos, f) <= 8]
    #print better_food_list
    sum_foods = 0
    sum_distance = 0
    for food in better_food_list:
      sum_foods += 1
      sum_distance += self.get_maze_distance(my_pos, food)
    features['num_foods'] = sum_foods
    features['sum_distanceToFood'] = sum_distance

    #Calculate Distance to nearest ghost
    min_distance = 999999
    for index in self.opponent_indices:
      if self.opponent_positions[index] != None:
        opponent_state = game_state.get_agent_state(index)
        distance = self.get_maze_distance(self.opponent_positions[index],game_state.get_agent_position(self.index))
        
        if game_state.get_agent_state(index).scared_timer > 0:
          distance = distance*1000
        if opponent_state.is_pacman:
          distance = distance*1000
        if distance < min_distance:
          min_distance = distance
    if min_distance == 0:
      min_distance = 0.01
    if min_distance < 6:
      features['closest_enemy'] = 5 - min_distance #float(1)/(5-min_distance**0.5)
    else:
      features['closest_enemy'] = 0 #float(1)/(5**0.5)

    distance = self.get_maze_distance(game_state.get_agent_position(self.teammate_index[0]),game_state.get_agent_position(self.index))
    if distance > 0:
      features['teammate_distance'] = float(1)/distance
    else:
      features['teammate_distance'] = 5

    capsules = self.get_capsules(game_state)
    min_distance = 9999999
    for capsule in capsules:
      distance =self.get_maze_distance(game_state.get_agent_position(self.index), capsule)
      if distance < min_distance:
        min_distance = distance
    if min_distance == 0:
      min_distance = 0.01
    if min_distance > 1000:
      features['closest_capsule_distance'] = 1
    else: 
      features['closest_capsule_distance'] = float(1)/min_distance

    # distance = self.get_maze_distance(game_state.get_agent_position(self.teammate_index[0]),game_state.get_agent_position(self.index))
    # if distance > 0:
    #   features['teammate_distance'] = float(1)/distance
    # else:
    #   features['teammate_distance'] = 5

    return features

  def get_offensive_weights(self, game_state):
    # what weights? check other implementations for a rough idea
    #return {'state_score': 60, 'num_foods': 60, 'sum_distanceToFood': -5, 'closest_enemy': -400, 'teammate_distance': -30,'closest_capsule_distance': 40}
    return {'state_score': 60, 'num_foods': 60, 'sum_distanceToFood': -5, 'closest_enemy': -10, 'teammate_distance': -90,'closest_capsule_distance': 80}
 
  ###### 'DEFENCE' BEHAVIOUR CODE ######

  def in_home_territory(self,game_state,position,offset):
    home_x = game_state.get_walls().width/2
    if self.red:
      home_x = home_x - (1+offset)
    else:
      home_x = home_x + offset

    if self.red and position[0] > home_x:
      return False
    elif not self.red and position[0] < home_x:
      return False
    else:
      return True

  def choose_defensive_action(self, game_state):
    # get a list of actions
    actions = game_state.get_legal_actions(self.index)
    actions.remove(Directions.STOP)
    for action in actions:
      successor = self.get_successor(game_state,action)
      successor_state = successor.get_agent_state(self.index)
      successor_pos = successor_state.get_position()
      if not self.in_home_territory(game_state,successor_pos,0) and not game_state.get_agent_state(self.index).is_pacman:
        actions.remove(action)
    # get a list of values (call evaluate?) OR call evaluate_defensive
      # evaluate defensive features/weights
    values = [self.evaluate_defensive(game_state, a) for a in actions]
    # choose action with best value
    if len(values) > 0:
      max_value = max(values)
    else:
      return Directions.STOP
    best_actions = [a for a, v in zip(actions, values) if v == max_value]
    return random.choice(best_actions)

  def evaluate_defensive(self, game_state, action):
    # same as base evaluate function really (see baselineTeam.py)
    features = self.get_defensive_features(game_state, action)
    weights = self.get_defensive_weights(game_state, action)
    return features * weights

  def get_defensive_features(self, game_state, action):
    # distance_to_center
    features = util.Counter()
    successor = self.get_successor(game_state, action)
    successor_state = successor.get_agent_state(self.index)
    successor_pos = successor_state.get_position()
    min_distance = 99999999
    if (not self.defence_destination == None) and self.get_maze_distance(successor_pos,self.defence_destination) < min_distance:
      min_distance = self.get_maze_distance(successor_pos,self.defence_destination)
    features['distance_to_center'] = min_distance
    return features

  def get_defensive_weights(self, game_state, action):
    #
    return {'distance_to_center': -1}

  def food_eaten_by_opponent(self, game_state):
    food_eaten_by_opponent = []
    for x in range(game_state.get_walls().width):
      for y in range(game_state.get_walls().height):
        if self.prev_food_state[x][y] == True and self.get_food_you_are_defending(game_state)[x][y] == False and self.in_home_territory(game_state, (x,y), 0):
          food_eaten_by_opponent = food_eaten_by_opponent + [(x,y)]
    self.prev_food_state = self.get_food_you_are_defending(game_state)
    return food_eaten_by_opponent

  ###### 'FLEE' BEHAVIOUR CODE ######

  def choose_flee_action(self, game_state):
    #
    q = Queue()
    q.push((game_state, []))
    visited = []
    i = 0
    while not q.is_empty():
      i = i+1
      state, route = q.pop()
      if self.nearest_ghost_distance(state) <= 1 and state != game_state:
        continue
      elif state.get_agent_position(self.index) in visited:
        continue
      elif self.in_home_territory(state,state.get_agent_position(self.index),0):
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
        q.push((self.get_successor(state,action),route+[action]))
    return random.choice(game_state.get_legal_actions(self.index))

#########################################################################33


class Top(Dummy_Agent):
# defender agent

  def set_guard_mode(self):
      self.can_guard = True

  def set_center(self,game_state):
    #get center of map and maxHeight

    x = int(game_state.get_walls().width/2)
    offset = 1
    if self.red:
      x = x - (1+offset)
    else:
      x = x + offset
    y = game_state.get_walls().height/2
    y_max = game_state.get_walls().height
    y_center = int(round(y_max/4*3))
    for i in range(0,y_max):
      y_candidate = y_center+i
      if y_candidate <= y_max and y_candidate > 0:
        if not game_state.has_wall(x,y_candidate):
          break
      y_candidate = y_center-i
      if y_candidate <= y_max and y_candidate > 0:
        if not  game_state.has_wall(x,y_candidate):
          break
    self.center = (x,y_candidate)

class Bottom(Dummy_Agent):
# attacker agent

  def set_guard_mode(self):
      self.can_guard = False

  def set_center(self,game_state):
    #get center of map and maxHeight
    x = int(game_state.get_walls().width/2)
    offset = 1
    if self.red:
      x = x - (1+offset)
    else:
      x = x + offset
    y = game_state.get_walls().height/2
    y_max = game_state.get_walls().height
    y_center = int(round(y_max/4))
    for i in range(0,y_max):
      y_candidate = y_center+i
      if y_candidate <= y_max and y_candidate > 0:
        if not  game_state.has_wall(x,y_candidate):
          break
      y_candidate = y_center-i
      if y_candidate <= y_max and y_candidate > 0:
        if not  game_state.has_wall(x,y_candidate):
          break
    self.center = (x,y_candidate)