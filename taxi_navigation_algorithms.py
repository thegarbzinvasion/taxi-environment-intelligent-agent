import gymnasium as gym ## <-- To use to Taxi-v4 environment
from typing import Any, cast
import heapq ## <-- To use a priority queue for A* Search
from collections import deque ## <-- To use a queue for BFS
class TaxiNavigation:
    def __init__(self):
     ## ~~~--------- !! Environmental Interface Definition !! --------~~~ ##

        ## Creates the Taxi-v4 environment from Python package "gymnasium"
        self.gym_environment = gym.make("Taxi-v4", render_mode="ansi")
        self.P = cast(Any, self.gym_environment.unwrapped).P ## Accessing the transition probabilities and rewards from the environment's unwrapped version
    
    ## Resets the environment to the initial state and returns the initial observation
    def reset_environment(self):
        initial_state, info = self.gym_environment.reset()
        return initial_state
    
    ## Renders the current state of the environment and returns it as a string (since render_mode is set to "ansi")
    def render_environment(self):
        return self.gym_environment.render()
    
    ## Executes the given action in the environment aka stepping in a direction and returns the new observation
    def execute_action(self, action):
        return self.gym_environment.step(action)
    
    ## ~~~-------- !! Environmental Interface Definition !! --------~~~ ##
    
    ## ~~~-------- !! Helper method to decode the state of Taxi-v3 using KnowledgeBase !! --------~~~ ##

    def decode_state(self, state):
        ## Decodes the Taxi-v3 state integer into taxi_row, taxi_col, passenger_location, destination
        destination = state % 4
        state //= 4
        passenger_location = state % 5
        state //= 5
        taxi_col = state % 5
        state //= 5
        taxi_row = state    
        return taxi_row, taxi_col, passenger_location, destination
    
     ## ~~~-------- !! Helper method to decode the state of Taxi-v3 using KnowledgeBase !! --------~~~ ##

     ## ~~~--------- !! Search Algorithm #1 (BFS) !! --------~~~ ##

    def bfs_search(self, kb):
        start_state = self.reset_environment() ## Define the start_state by calling reset_environment() method of TaxiNavigation
        queue = deque()
        queue.append((start_state, []))  ## (current_state, path_to_current_state)
        visited = set()  ## To keep track of visited states
        while queue:
            state, path = queue.popleft()  ## Get the next state and path (left node first in regards to a tree)
            if state in visited:
                continue  ## Skip if the state has already been visited
            visited.add(state)  ## Mark the state as visited
            taxi_row, taxi_col, passenger_location, destination = self.decode_state(state)  ## Decode the state using the helper method
            passenger_state = kb.passenger_states[passenger_location]  ## Get the passenger state from the knowledge base
            destination_state = kb.drop_off_locations[destination]  ## Get the destination state from the knowledge base
            if kb.is_at_goal_state(passenger_state, destination_state):  ## Check if the current state is the goal state using the knowledge base
                return path  ## Return the path to the goal state if found    
            for action in range(6):  ## Loop through all possible actions (0 to 5)
                transitions = self.P[state][action]  ## Get the possible transitions for the current state and action
                for prob, next_state, reward, done in transitions:  ## Loop through the possible transitions
                    if next_state in visited:
                        continue
                    new_path = path.copy()  ## Create a copy of the current path
                    new_path.append(action)  ## Add the current action to the new path

                    if done:
                        return new_path  ## If the action leads to a terminal state, return the path immediately
                    queue.append((next_state, new_path))  ## Add the new state and path 
        return None  ## Return None if no solution is found after exploring all states

     ## ~~~--------- !! Search Algorithm #1 (BFS) !! --------~~~ ##

    def heuristic(self, state, kb):
        taxi_row, taxi_col, passenger_location, destination = self.decode_state(state)  ## Decode the state using the helper method
        passenger_state = kb.passenger_states[passenger_location]  ## Get the passenger state from the knowledge base
        destination_state = kb.drop_off_locations[destination]  ## Get the destination state from the knowledge base
        coordinates = {
            'Red': (0, 0),
            'Green': (0, 4),
            'Yellow': (4, 0),
            'Blue': (4, 3),
        }
        if passenger_state == "In taxi":
            destination_coordinate = coordinates[destination_state]
            return abs(taxi_row - destination_coordinate[0]) + abs(taxi_col - destination_coordinate[1])  ## Manhattan distance to the destination
        else:
            # If the passenger is not in the taxi, calculate the distance to the passenger's location to pick them up first
            passenger_coordinate = coordinates[passenger_state]
            destination_coordinate = coordinates[destination_state]
            to_passenger = abs(taxi_row - passenger_coordinate[0]) + abs(taxi_col - passenger_coordinate[1])  ## Manhattan distance to the passenger
            from_passenger_to_destination = abs(passenger_coordinate[0] - destination_coordinate[0]) + abs(passenger_coordinate[1] - destination_coordinate[1])  ## Manhattan distance from the passenger to the destination
            return to_passenger + from_passenger_to_destination  ## Total heuristic is the distance to the passenger plus the distance from the passenger to the destination
        
     ## ~~~--------- !! Search Algorithm #2 (A* search) !! --------~~~ ##

    def a_star_search(self, kb):
        start_state = self.reset_environment() ## Define the start_state by calling reset_environment() method of TaxiNavigation
        priority_queue = []  ## To keep track of states to explore based on their f(n) value **(f(n) = g(n) + h(n))**
        start_h = self.heuristic(start_state, kb)  ## Calculate the heuristic value for the start state
        heapq.heappush(priority_queue, (start_h, 0, start_state, []))  ## (f(n), g(n), current_state, path_to_current_state) **(f(n) = g(n) + h(n))**
        visited = set()  ## To keep track of visited states
        while priority_queue:
            f, g, state, path = heapq.heappop(priority_queue)  ## Get the state with the lowest f(n) value **(f(n) = g(n) + h(n))**
            if state in visited:
                continue  ## Skip if the state has already been visited
            visited.add(state)  ## Mark the state as visited
            taxi_row, taxi_col, passenger_location, destination = self.decode_state(state)  ## Decode the state using the helper method
            passenger_state = kb.passenger_states[passenger_location]  ## Get the passenger state from the knowledge base
            destination_state = kb.drop_off_locations[destination]  ## Get the destination state from the knowledge base
            if kb.is_at_goal_state(passenger_state, destination_state):  ## Check if the current state is the goal state using the knowledge base
                return path  ## Return the path to the goal state if found    
            for action in range(6):  ## Loop through all possible actions (0 to 5)
                for prob, next_state, reward, done in self.P[state][action]:  ## Loop through the possible transitions for the current state and action
                    if next_state in visited:
                        continue
                    new_path = path.copy()  ## Create a copy of the current path
                    new_path.append(action)  ## Add the current action to the new path
                    if done:
                        return new_path  ## If the action leads to a terminal state, return the path immediately
                    g_n = g + 1  ## Increment g(n) by 1 for the new path (assuming each action has a cost of 1)
                    h_n = self.heuristic(next_state, kb)  ## Calculate h(n) using the heuristic function defined above
                    f_n = g_n + h_n  ## Calculate f(n) as g(n) + h(n)
                    heapq.heappush(priority_queue, (f_n, g_n, next_state, new_path))  ## Add the new state and path to the priority queue with its f(n) value
        return None  ## Return None if no solution is found after exploring all states
    
     ## ~~~--------- !! Search Algorithm #2 (A* search) !! --------~~~ ##

     ## ~~~--------- !! Execute Path and Calculate Rewards !! --------~~~ ##
    
    def execute_path(self, path, kb):
        state = self.reset_environment()
        total_reward = 0.0
        print("\nInitial Environmental State:")
        print("\n")
        print(self.render_environment())  ## Print the initial state of the environment before executing the path
        print("EXECUTING PATH AND CALCULATING REWARDS...")
        done = False
        for i, action in enumerate(path):
            next_state, reward, done, _, _ = self.execute_action(action)
            total_reward += float(reward) 
            ## Print the action taken, reward received, and total reward accumulated so far
            action_name = kb.available_actions[action]
            state = next_state 
            ## After executing each action, check if the environment has reached a terminal state (done == True) and print a message if it has
            if done:
                print("\nReached a terminal state.")
                break
         ## After executing the entire path, check if the goal state is achieved and print the final results
        taxi_row, taxi_col, passenger_location, destination = self.decode_state(state)
        passenger_state = kb.passenger_states[passenger_location]
        destination_state = kb.drop_off_locations[destination]
        ## Check if the goal state is achieved using the knowledge base and print the final results
        if done:
            print(f"\n{kb.goal}")
        else:
            print(f"\nGoal not achieved")
        print("-"*50)
        print(f"\nTotal Reward: {total_reward}")
        print("-"*50)
        return total_reward

     ## ~~~--------- !! Execute Path and Calculate Rewards !! --------~~~ ##