class Cup:
    def __init__(self, cup_string):
        # Format: "CUP_TYPE CUP_NAME CUP_POS CLEAN? EMPTY? FULL?"
        cup_info = cup_string.split(" ")
        self.cup_type = cup_info[0]
        self.cup_name = cup_info[1]
        self.cup_loc = cup_info[2]
        self.cup_clean = cup_info[3] == "True"
        self.cup_empty = cup_info[4] == "True"
        self.cup_full = cup_info[5] == "True"

    def __str__(self):
        return f"{self.cup_type} {self.cup_name} {self.cup_loc}"

class Location:
    def __init__(self, loc_string):
        # Format: "LOCATION_NAME LOCATION_TYPE CLEAR?"
        loc_info = loc_string.split(" ")
        self.loc_name = loc_info[1]
        self.loc_type = loc_info[0]  # dirty-area, clean-area, coffee-machine, or serving-counter

    def __str__(self):
        return f"{self.loc_name}"

class CoffeeEnv:
    def __init__(self, baseenv, cups, locations, robot_loc, hands_free):
        self.cups = {Cup(cup).cup_name: Cup(cup) for cup in cups}
        self.locations = {Location(loc).loc_name: Location(loc) for loc in locations}
        self.robot_loc = robot_loc
        self.hands_free = hands_free
        self.holding = None

        self.baseenv = baseenv
        #need to make object list a list of cups: "CUP_TYPE CUP_NAME CUP_POS" and locations
        self.object_list = [str(cup) for cup in self.cups.values()] + [str(loc) for loc in self.locations.values()] + ["esp-btn", "cap-btn", "amer-btn"]
        _ = self.baseenv.reset(self.object_list)

    def at(self, cup_name, loc_name):
        """Check if a cup is at a specific location."""
        if cup_name in self.cups:
            return self.cups[cup_name].cup_loc == loc_name
        return False
    
    def hands_free(self):
        """Check if the robot's hand is free."""
        return self.hand_free
    
    def robot_at(self, loc_name):
        """Check if the robot is at a specific location."""
        return self.robot_loc == loc_name
    
    def pick(self, cup_name, loc_name):
        """Pick a cup if the robot is at the cup's location and hands are free."""

        # Preconditions
        if not self.at(cup_name, loc_name):
            raise ValueError(f"Cannot pick {cup_name} at {loc_name}: cup is not there.")
        if not self.robot_at(loc_name):
            raise ValueError(f"Cannot pick {cup_name}: robot is not at {loc_name}.")
        if not self.hands_free:
            raise ValueError(f"Cannot pick {cup_name}: robot's hands are not free.")
        
        # Effects
        cupobj = self.cups[cup_name]
        cupobj.cup_loc = "hand"
        self.hands_free = False
        self.holding = cupobj

        # Update the base environment
        self.baseenv.pick(cup_name)

    def place(self, cup_name, loc_name):
        """Place a cup at a specific location if the robot is holding it."""
        
        # Preconditions
        if self.holding is None or self.holding.cup_name != cup_name:
            raise ValueError(f"Cannot place {cup_name}: robot is not holding it.")
        if not self.robot_at(loc_name):
            raise ValueError(f"Cannot place {cup_name} at {loc_name}: robot is not there.")
        
        # Effects
        cupobj = self.holding
        cupobj.cup_loc = loc_name
        self.hands_free = True
        self.holding = None

        # Update the base environment
        if self.locations[loc_name].loc_type == "coffee-machine":
            self.baseenv.place(cup_name, loc_name, find_empty_pos=False)
        else:
            self.baseenv.place(cup_name, loc_name, find_empty_pos=True)

    def move(self, from_loc, to_loc):
        """Move the robot from one location to another."""
        
        # Preconditions
        if not self.robot_at(from_loc):
            raise ValueError(f"Cannot move: robot is not at {from_loc}.")
        
        # Effects
        self.robot_loc = to_loc

        # Update the base environment
        self.baseenv.move_to_location(to_loc)

    def press_espresso_button(self, button_name, cup_name, cm_location):
        """Press the espresso button on the coffee machine."""
        # Preconditions
        if button_name != "esp-btn":
            raise ValueError(f"Cannot press {button_name}: not the espresso button.")
        if self.cups[cup_name].cup_type != "espresso-cup":
            raise ValueError(f"Cannot press {button_name}: cup is not an espresso cup.")
        if self.locations[cm_location].loc_type != "coffee-machine":
            raise ValueError(f"Cannot press {button_name}: not at coffee machine location.")
        if not self.cups[cup_name].cup_clean:
            raise ValueError(f"Cannot press {button_name}: cup {cup_name} is not clean.")
        if self.robot_loc != cm_location:
            raise ValueError(f"Cannot press {button_name}: robot is not at coffee machine location {cm_location}.")

        # Effects
        self.cups[cup_name].cup_full = True
        self.cups[cup_name].cup_empty = False
        self.cups[cup_name].cup_clean = False

        # Update the base environment
        self.baseenv.press_button(button_name)

    def press_cappuccino_button(self, button_name, cup_name, cm_location):
        """Press the cappuccino button on the coffee machine."""
        # Preconditions
        if button_name != "cap-btn":
            raise ValueError(f"Cannot press {button_name}: not the cappuccino button.")
        if self.cups[cup_name].cup_type != "cappuccino-cup":
            raise ValueError(f"Cannot press {button_name}: cup is not a cappuccino cup.")
        if self.locations[cm_location].loc_type != "coffee-machine":
            raise ValueError(f"Cannot press {button_name}: not at coffee machine location.")
        if not self.cups[cup_name].cup_clean:
            raise ValueError(f"Cannot press {button_name}: cup {cup_name} is not clean.")
        if self.robot_loc != cm_location:
            raise ValueError(f"Cannot press {button_name}: robot is not at coffee machine location {cm_location}.")

        # Effects
        self.cups[cup_name].cup_full = True
        self.cups[cup_name].cup_empty = False
        self.cups[cup_name].cup_clean = False

        # Update the base environment
        self.baseenv.press_button(button_name)

    def press_americano_button(self, button_name, cup_name, cm_location):
        """Press the americano button on the coffee machine."""
        # Preconditions
        if button_name != "amer-btn":
            raise ValueError(f"Cannot press {button_name}: not the americano button.")
        if self.cups[cup_name].cup_type != "americano-cup":
            raise ValueError(f"Cannot press {button_name}: cup is not an americano cup.")
        if self.locations[cm_location].loc_type != "coffee-machine":
            raise ValueError(f"Cannot press {button_name}: not at coffee machine location.")
        if not self.cups[cup_name].cup_clean:
            raise ValueError(f"Cannot press {button_name}: cup {cup_name} is not clean.")
        if self.robot_loc != cm_location:
            raise ValueError(f"Cannot press {button_name}: robot is not at coffee machine location {cm_location}.")

        # Effects
        self.cups[cup_name].cup_full = True
        self.cups[cup_name].cup_empty = False
        self.cups[cup_name].cup_clean = False

        # Update the base environment
        self.baseenv.press_button(button_name)
