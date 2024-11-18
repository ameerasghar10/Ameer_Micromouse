class PositionTracker:
    
# This a function that defines the micromouse's beginning position in terms of (x,y). A beginning direction is also set to know which way that the micromouse is facing (E.g. North) 
    def __init__(self, initial_position, initial_direction):
        self.current_cell_x, self.current_cell_y = initial_position
        self.direction = initial_direction
    
    def update_position(self, movement):

# This is just for moving forward at the moment but updates the micromouse's position depending on where it's facing forward North, South etc...  
        if movement == 'forward':
            if self.direction == 'N':
                self.current_cell_y += 1
            elif self.direction == 'S':
                self.current_cell_y -= 1
            elif self.direction == 'E':
                self.current_cell_x += 1
            elif self.direction == 'W':
                self.current_cell_x -= 1

# This function returns the micromouse's new position now     
    def get_position(self):
        return (self.current_cell_x, self.current_cell_y)

# This is for setting a new direction    
    def set_direction(self, new_direction):
        self.direction = new_direction

# This is for returning the new facing direction
    def get_direction(self):
        return self.direction
