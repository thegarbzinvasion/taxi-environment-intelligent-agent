class KnowledgeBase():
     ## ~~~--------- !! State Labels !! --------~~~ ##
    def __init__(self):
        ## Passenger State represented as a list of 5 elements, each element can be 0, 1, 2, 3, or 4
        self.passenger_states = {
            0: 'Red',
            1: 'Green',
            2: 'Yellow',
            3: 'Blue',
            4: 'In taxi',
        }

        ## Drop-off Locations represented as a list of 4 elements, each element can be 0, 1, 2, or 3
        self.drop_off_locations = {
            0: 'Red',
            1: 'Green',
            2: 'Yellow',
            3: 'Blue', 
        }

        ## Available Actions for the Taxi represented as a list of 6 elements, each element can be 0, 1, 2, 3, 4, or 5
        self.available_actions = {
            0: 'Move south (down)',
            1: 'Move north (up)',
            2: 'Move east (right)',
            3: 'Move west (left)',
            4: 'Pick up passenger',
            5: 'Drop off passenger',
        }

        ## Rewards & Penalties for the Taxi Environment
        self.goal_rewards = {
            'action_cost': -1,
            'successful_drop_off': +20,
            'successful_pick_up': 0,
            'illegal_pick_up': -10,
            'illegal_drop_off': -10,
        }
     ## ~~~--------- !! State Labels !! --------~~~ ##

     ## ~~~--------- !! Domain Rules !! --------~~~ ##

    def is_valid_pickup(self, taxi_position, passenger_position):
        return taxi_position == passenger_position 
    
    def is_valid_dropoff(self, taxi_position, passenger_state, destination_state):
        return passenger_state == "In taxi" and taxi_position == destination_state
    
    def is_at_goal_state(self, passenger_state, destination_state):
        return passenger_state == destination_state 
    
     ## ~~~--------- !! Domain Rules !! --------~~~ ##


        
        

