from .location import Location
from .cup import Cup
from .button import Button
from envs.sim.env import SimActions

class ActionExecutor:
    def __init__(self, cups, locations, buttons, robot_loc, hands_free=True, render=False, high_res=False, high_frame_rate=False, max_steps=5000):
        self.symbolic_state = SymbolicState(cups, 
                                            locations, 
                                            buttons, 
                                            robot_loc, 
                                            hands_free)
        self.sim_actions = SimActions(self.symbolic_state.cups, 
                                      self.symbolic_state.locations, 
                                      self.symbolic_state.buttons, 
                                      robot_loc, 
                                      render=render, 
                                      high_res=high_res, 
                                      high_frame_rate=high_frame_rate, 
                                      max_steps=max_steps)
        
    
    def execute(self, action_string):
        print("Executing action:", action_string)
        action_array = action_string.split(' ')
        action = action_array[0]
        self.validate_symbolic_state()
        if action == 'PICK':
            cup_to_pick = action_array[1].lower()
            cup_location = action_array[2].lower()
            self.symbolic_state.pick(cup_to_pick, cup_location)
            self.sim_actions.pick(cup_to_pick)
        if action == 'MOVE':
            move_from = action_array[1].lower()
            move_to = action_array[2].lower()
            self.symbolic_state.move(move_from, move_to)
            self.sim_actions.move_to_location(move_to)
        if action == 'PLACE':
            cup_to_place = action_array[1].lower()
            cup_location = action_array[2].lower()
            self.symbolic_state.place(cup_to_place, cup_location)
            if self.symbolic_state.locations[cup_location].loc_type == "coffee-machine":
                self.sim_actions.place(cup_to_place, cup_location, find_empty_pos=False)
            else:
                self.sim_actions.place(cup_to_place, cup_location, find_empty_pos=True)
        if action == 'PRESS-BUTTON-CAPPUCCINO':
            button_name = action_array[1].lower()
            cup_name = action_array[2].lower()
            cm_location = action_array[3].lower()
            self.symbolic_state.press_cappuccino_button(button_name, cup_name, cm_location)
            self.sim_actions.press_button(button_name)
        if action == 'PRESS-BUTTON-ESPRESSO':
            button_name = action_array[1].lower()
            cup_name = action_array[2].lower()
            cm_location = action_array[3].lower()
            self.symbolic_state.press_espresso_button(button_name, cup_name, cm_location)
            self.sim_actions.press_button(button_name)
        if action == 'PRESS-BUTTON-AMERICANO':
            button_name = action_array[1].lower()
            cup_name = action_array[2].lower()
            cm_location = action_array[3].lower()
            self.symbolic_state.press_americano_button(button_name, cup_name, cm_location)
            self.sim_actions.press_button(button_name)
        if action == 'PRESS-CLEANING-BUTTON':
            button_name = action_array[1].lower()
            cup_name = action_array[2].lower()
            cm_location = action_array[3].lower()
            self.symbolic_state.press_cleaning_button(button_name, cup_name, cm_location)
            self.sim_actions.press_button(button_name)
        
    
    def validate_symbolic_state(self):
        """Validate symbolic state against the base environment"""
        # Check that cups are at right locations
        for cup_name, cup in self.symbolic_state.cups.items():
            if cup.cup_loc != "hand" and not self.sim_actions.on_top_of(cup_name, cup.cup_loc):
                raise ValueError(f"Symbolic state mismatch: {cup_name} is not at {cup.cup_loc} in the base environment.")
        
        # Check if hand is empty that gripper is actually empty
        if self.symbolic_state.hands_free:
            if not self.sim_actions.hand_empty():
                raise ValueError("Symbolic state mismatch: hands should be free but gripper is not empty.")
            

class SymbolicState:
    def __init__(self, cups, locations, buttons, robot_loc, hands_free):
        self.cups = {Cup(cup).cup_name: Cup(cup) for cup in cups}
        self.locations = {Location(loc).loc_name: Location(loc) for loc in locations}
        self.buttons = {Button(btn).button_name: Button(btn) for btn in buttons}
        self.robot_loc = robot_loc
        self.hands_free = hands_free
        self.holding = None

        #need to make object list a list of cups: "CUP_TYPE CUP_NAME CUP_POS" and locations
        self.object_list = [str(cup) for cup in self.cups.values()] + [str(loc) for loc in self.locations.values()] + [str(btn) for btn in self.buttons.values()]

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

    def move(self, from_loc, to_loc):
        """Move the robot from one location to another."""
        
        # Preconditions
        if not self.robot_at(from_loc):
            raise ValueError(f"Cannot move: robot is not at {from_loc}.")
        
        # Effects
        self.robot_loc = to_loc

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
        # self.baseenv.press_button(button_name)
        # self.baseenv.change_color(cup_name, "brown")

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
        # self.baseenv.press_button(button_name)
        # self.baseenv.change_color(cup_name, "white")

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
        # self.baseenv.press_button(button_name)
        # self.baseenv.change_color(cup_name, "black")

    def press_cleaning_button(self, button_name, cup_name, location):
        """Press cleaning button"""
        # Preconditions
        if self.cups[cup_name].cup_loc != location:
            raise ValueError(f"Cannot press cleaning button: cup {cup_name} is not at {location}.")
        if self.robot_loc != location:
            raise ValueError(f"Cannot press cleaning button: robot is not at {location}.")
        if not self.cups[cup_name].cup_empty:
            raise ValueError(f"Cannot press cleaning button: cup {cup_name} is not empty.")
        if not self.cups[cup_name].dirty:
            raise ValueError(f"Cannot press cleaning button: cup {cup_name} is not dirty.")
        if button_name != "clean-btn":
            raise ValueError(f"Cannot press {button_name}: not the cleaning button.")
        # Effects
        self.cups[cup_name].cup_clean = True
        self.cups[cup_name].dirty = False

        # Update the base environment
        # self.baseenv.press_button(button_name)
        # self.baseenv.change_color(cup_name, "silver")  # Change cup color to indicate cleaning

    def get_groundings(self):
        """Get current symbolic state in object form for JSON serialization."""
        return {
            "cups": [{
                    "cup_type": cup.cup_type,
                    "cup_name": cup.cup_name,
                    "cup_loc": cup.cup_loc,
                    "cup_clean": cup.cup_clean,
                    "cup_empty": cup.cup_empty,
                    "cup_full": cup.cup_full,
                    "dirty": cup.dirty
                } for cup in self.cups.values()],
            "locations": [{
                    "loc_name": loc.loc_name,
                    "loc_type": loc.loc_type
                } for loc in self.locations.values()],
            "buttons": [{
                    "button_name": btn.button_name,
                    "button_type": btn.button_type
                } for btn in self.buttons.values()],
            "robot_location": self.robot_loc,
            "hands_free": self.hands_free,
            "holding": str(self.holding) if self.holding else None
        }
    
