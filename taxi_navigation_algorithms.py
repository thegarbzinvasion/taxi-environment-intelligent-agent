import gymnasium as gym ## <-- To use to Taxi-v4 environment
from typing import Any, cast
import heapq ## <-- To use a priority queue for A* Search
from collections import deque ## <-- To use a queue for BFS
import time
class TaxiNavigation:
    def __init__(self):
        ## ~~~--------- !! Environmental Interface Definition !! --------~~~ ##
        self.gym_environment = gym.make("Taxi-v4", render_mode="ansi")
        self.P = cast(Any, self.gym_environment.unwrapped).P 
        self.initial_state = None  
        ## ~~~--------- !! Environmental Interface Definition !! --------~~~ ##

    def reset_environment(self):
        ## Reset environment once per run to preserve the map layout state across search and execution
        initial_state, info = self.gym_environment.reset()
        self.initial_state = initial_state  
        return initial_state
    
    def render_environment(self):
        return self.gym_environment.render()
    
    def execute_action(self, action):
        return self.gym_environment.step(action)
    
    def decode_state(self, state):
        ## Decodes the discrete single state integer back into its 4 layout variables
        destination = state % 4
        state //= 4
        passenger_location = state % 5
        state //= 5
        taxi_col = state % 5
        state //= 5
        taxi_row = state    
        return taxi_row, taxi_col, passenger_location, destination

    ## ~~~--------- !! Search Algorithm #1 (BFS) !! --------~~~ ##

    def bfs_search(self, start_state, kb):
        queue = deque()
        queue.append((start_state, []))  
        visited = set([start_state])  
        ## --- METRICS COLLECTORS ---
        nodes_explored = 0
        max_frontier_size = 1
        start_time = time.perf_counter()
        while queue:
            ## Track maximum size the queue reached (Space Complexity proxy)
            max_frontier_size = max(max_frontier_size, len(queue))
            state, path = queue.popleft()  
            nodes_explored += 1 ## Track popped nodes (Time Complexity proxy)
            taxi_row, taxi_col, passenger_location, destination = self.decode_state(state)
            for action in range(6):  
                ## Logic Filter: Prune obviously impossible pickups/dropoffs to avoid sync errors
                if action == 4 and passenger_location == 4:  
                    continue
                if action == 5 and passenger_location != 4:  
                    continue
                for prob, next_state, reward, done in self.P[state][action]:
                    if reward == -10:
                        continue
                    if next_state in visited:
                        continue
                    new_path = path + [action]  
                    ## Goal verification: explicitly checking for the terminal +20 success reward flag
                    if done and reward == 20:
                        metrics = {
                            "time_complexity_nodes": nodes_explored,
                            "space_complexity_nodes": max_frontier_size,
                            "execution_time_ms": (time.perf_counter() - start_time) * 1000,
                            "completeness": "Yes (BFS is complete for finite state spaces)",
                            "optimality": "Yes (BFS guarantees fewest actions/shallowest path)"
                        }
                        return new_path, metrics  
                    queue.append((next_state, new_path))  
                    visited.add(next_state)  
        metrics = {
            "time_complexity_nodes": nodes_explored,
            "space_complexity_nodes": max_frontier_size,
            "execution_time_ms": (time.perf_counter() - start_time) * 1000,
            "completeness": "Yes",
            "optimality": "N/A (No path found)"
        }
        return None, metrics  
    
    ## ~~~--------- !! Search Algorithm #1 (BFS) !! --------~~~ ##

    def heuristic(self, state, kb):
        taxi_row, taxi_col, passenger_location, destination = self.decode_state(state)  
        passenger_state = kb.passenger_states[passenger_location]  
        destination_state = kb.drop_off_locations[destination]  
        coordinates = {
            'Red': (0, 0),
            'Green': (0, 4),
            'Yellow': (4, 0),
            'Blue': (4, 3),
        }
        ## If passenger is inside the cab, track Manhattan distance straight to destination
        if passenger_state == "In taxi":
            destination_coordinate = coordinates[destination_state]
            return abs(taxi_row - destination_coordinate[0]) + abs(taxi_col - destination_coordinate[1])  
        ## If passenger is waiting, add distance to passenger + distance from passenger to destination
        p = coordinates[passenger_state]
        d = coordinates[destination_state]
        return (
            abs(taxi_row - p[0]) + abs(taxi_col - p[1]) +  
            abs(p[0] - d[0]) + abs(p[1] - d[1])  
        )
        
    ## ~~~--------- !! Search Algorithm #2 (A* search) !! --------~~~ ##

    def a_star_search(self, start_state, kb):
        priority_queue = []  
        start_h = self.heuristic(start_state, kb)  
        heapq.heappush(priority_queue, (start_h, 0, start_state, [])) 
        ## Map to track the lowest step-cost (g) spent to discover any given state
        best_g = {start_state: 0} 
        ## --- METRICS COLLECTORS ---
        nodes_explored = 0
        max_frontier_size = 1
        start_time = time.perf_counter()
        while priority_queue:
            max_frontier_size = max(max_frontier_size, len(priority_queue))
            f, g, state, path = heapq.heappop(priority_queue)  
            nodes_explored += 1
            ## Graph Search Check: Skip processing if a faster path to this state was popped first
            if g > best_g.get(state, float('inf')):
                continue
            taxi_row, taxi_col, passenger_location, destination = self.decode_state(state)
            for action in range(6):  
                if action == 4 and passenger_location == 4:  
                    continue
                if action == 5 and passenger_location != 4:  
                    continue
                for prob, next_state, reward, done in self.P[state][action]:  
                    if reward == -10:
                        continue
                    g_n = g + 1  
                    ## Optimization: Check cost at state discovery time to prevent heap bloat
                    if g_n < best_g.get(next_state, float('inf')):
                        best_g[next_state] = g_n
                        new_path = path + [action]
                        if done and reward == 20:
                            metrics = {
                                "time_complexity_nodes": nodes_explored,
                                "space_complexity_nodes": max_frontier_size,
                                "execution_time_ms": (time.perf_counter() - start_time) * 1000,
                                "completeness": "Yes",
                                "optimality": "Yes (Admissible Manhattan heuristic guarantees path cost optimality)"
                            }
                            return new_path, metrics
                        h_n = self.heuristic(next_state, kb)  
                        f_n = g_n + h_n  
                        heapq.heappush(priority_queue, (f_n, g_n, next_state, new_path))  
                        
        metrics = {
            "time_complexity_nodes": nodes_explored,
            "space_complexity_nodes": max_frontier_size,
            "execution_time_ms": (time.perf_counter() - start_time) * 1000,
            "completeness": "Yes",
            "optimality": "N/A (No path found)"
        }
        return None, metrics  
    
    ## ~~~--------- !! Search Algorithm #2 (A* search) !! --------~~~ ##

    ## ~~~--------- !! Execute Path and Calculate Rewards !! --------~~~ ##
    
    def execute_path(self, path, kb, metrics=None):
        total_reward = 0.0
        print("\nInitial State:")
        print(self.render_environment())  
        print("\n==============================")
        print("EXECUTION")
        print("==============================")
        done = False
        for i, action in enumerate(path):
            state, reward, done, _, _ = self.execute_action(action)
            total_reward += float(reward)
            print(
                f"Step {i+1}: {kb.available_actions[action]} "
                f"| Reward: {reward} | Total: {total_reward}"
            )
            if done:
                print("\nTerminal state reached.")
                break
        ## Outcome evaluation and post-execution visual environment renders
        if done and total_reward > 0: 
            print("\nSUCCESS: Passenger delivered!")
            print("\nFinal Environment State:")
            print(self.render_environment()) 
        else:
            print("\nFAILED: Did not reach goal.")
            print("\nFinal Environment State:")
            print(self.render_environment())
        print(f"\nFinal reward: {total_reward}")
        if metrics:
            print("\n==============================================")
            print("         ALGORITHM PERFORMANCE METRICS        ")
            print("==============================================")
            print(f" * Completeness:      {metrics['completeness']}")
            print(f" * Cost Optimality:   {metrics['optimality']}")
            print(f" * Time Complexity:   {metrics['time_complexity_nodes']} nodes popped from frontier")
            print(f" * Space Complexity:  {metrics['space_complexity_nodes']} maximum nodes held in frontier concurrently")
            print(f" * Search Computation: {metrics['execution_time_ms']:.2f} ms")
            print("==============================================\n")
        return total_reward

    ## ~~~--------- !! Execute Path and Calculate Rewards !! --------~~~ ##