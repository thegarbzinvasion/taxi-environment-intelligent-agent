from taxi_navigation_algorithms import TaxiNavigation
from knowledge_base import KnowledgeBase

def run_single_algorithm(algo_choice, start_state, kb, taxi):
    """Helper function to execute a single search algorithm and run its path."""
    if algo_choice == '0':
        print("\nRunning BFS...")
        ## Unpack both the solution path and the metric tracking dictionary from BFS
        path, metrics = taxi.bfs_search(start_state, kb)
    else:
        print("\nRunning A*...")
        ## Unpack both the solution path and the metric tracking dictionary from A*
        path, metrics = taxi.a_star_search(start_state, kb)    
        
    if path:
        print(f"Path found!")
        ## Execute the action sequence and print the performance metrics summary
        taxi.execute_path(path, kb, metrics)
    else:
        print("No path found!")

def main():
    kb = KnowledgeBase()
    print("\nSelect Mode:")
    print("0 - BFS Only")
    print("1 - A* Only")
    print("2 - Both: (BFS vs A*): ")
    ## Capture the user's evaluation choice to dictate script behavior
    mode = input("Enter choice (0, 1, or 2): ").strip()
    try:
        ## Convert string input into an integer to control the loop boundary length
        num_loops = int(input("Number of Loops: ").strip())
    except ValueError:
        ## Gracefully catch non-integer entries and fall back to a single baseline run
        print("Invalid number. Defaulting to 1 loop.")
        num_loops = 1
    ## Iterate through the total count of randomized map cycles requested by the user
    for loop_idx in range(1, num_loops + 1):
        print("\n" + "="*60)
        print(f"               Loop {loop_idx} of {num_loops}               ")
        print("="*60) 
        ## Initialize a fresh instance of the Taxi navigation environment class object
        taxi_base = TaxiNavigation() 
        ## Reset environment to generate a unique random map layout and cache its state ID
        start_state = taxi_base.reset_environment()
        if mode in ['0', '1']:
            ## Delegate standalone mode calls directly down to our single runner helper
            run_single_algorithm(mode, start_state, kb, taxi_base)    
        elif mode == '2':
            ## ~~~--------- Head-to-Head Evaluation Execution ---------~~~ ##
            print("\n---BFS---")
            bfs_path, bfs_metrics = taxi_base.bfs_search(start_state, kb)
            if bfs_path:
                taxi_base.execute_path(bfs_path, kb, bfs_metrics)
            else:
                print("BFS failed to find a path.")  
            print("\n--- A* (With same map) ---")
            ## Spin up a completely separate environment tracker object for the A* segment
            taxi_astar = TaxiNavigation()
            ## Clear internal gym library state properties to ensure a clean simulation canvas
            taxi_astar.gym_environment.reset(seed=None)    
            ## CRITICAL: Force inject the exact state ID integer to perfectly sync the A* map to BFS
            taxi_astar.gym_environment.unwrapped.s = start_state # type: ignore   
            ## Execute A* on the locked state to ensure a 100% fair benchmark comparison
            astar_path, astar_metrics = taxi_astar.a_star_search(start_state, kb)
            if astar_path:
                taxi_astar.execute_path(astar_path, kb, astar_metrics)
            else:
                print("A* failed to find a path.") 
             ## ~~~--------- Head-to-Head Evaluation Execution ---------~~~ ##     
if __name__ == "__main__":
    main()