import queue 

class KnowledgeBase():
    def __init__(self):
        ## Passanger State represented as a list of 5 elements, each element can be 0, 1, 2, 3, or 4
        self.state_labels = {
            '0': 'Red',
            '1': 'Green',
            '2': 'Yellow',
            '3': 'Blue',
            '4': 'In taxi',
        }

        self.state_labels = {
            ## Drop-off Locations represented as a list of 4 elements, each element can be 0, 1, 2, or 3
            '0': 'Red',
            '1': 'Green',
            '2': 'Yellow',
            '3': 'Blue', 
        }

        self.state_labels = {
            ## Available Actions for the Taxi represented as a list of 6 elements, each element can be 0, 1, 2, 3, 4, or 5
            '0': 'Move south (down)',
            '1': 'Move north (up)',
            '2': 'Move east (right)',
            '3': 'Move west (left)',
            '4': 'Pick up passenger',
            '5': 'Drop off passenger',
        }

