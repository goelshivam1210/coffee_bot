import sys
import os

# Add the pddl-parser directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'scripts', 'pddl-parser'))

from pddl_parser.planner import Planner
from pddl_parser.PDDL import PDDL_Parser

def run_pddl_planner(domain_file, problem_file):
    """Run the PDDL parser planner and return the plan"""
    planner = Planner()
    plan = planner.solve(domain_file, problem_file)
    return plan

def format_plan_actions(plan):
    """Convert plan actions to the expected string format"""
    formatted_plan = []
    
    if plan is None:
        print("No plan found!")
        return []
    
    for action in plan:
        # Format action name and parameters
        action_str = action.name.upper()
        if action.parameters:
            action_str += " " + " ".join(action.parameters).upper()
        formatted_plan.append(action_str)
    
    return formatted_plan

def plan_and_parse(domain_file="pddl/domain.pddl", problem_file="pddl/problem.pddl"):
    """Run a PDDL planner on domain and problem files to produce a sequence of ground actions (aka a plan)"""    

    # Check if files exist
    if not os.path.exists(domain_file):
        raise FileNotFoundError(f"Domain file not found: {domain_file}")
    if not os.path.exists(problem_file):
        raise FileNotFoundError(f"Problem file not found: {problem_file}")

    # Run the planner
    plan = run_pddl_planner(domain_file, problem_file)
    print("PDDL Planner found plan:", plan)

    # Format the plan to match the expected format
    parsed_plan = format_plan_actions(plan)
    print("Formatted Plan:", parsed_plan)    
    return parsed_plan

def parse_pddl_objects(domain_file="pddl/domain.pddl", problem_file="pddl/problem.pddl"):
    """Parse PDDL files and return CUPS, LOCATIONS, and BUTTONS arrays"""
    # Create PDDL parser instance
    parser = PDDL_Parser()
    
    # Parse domain and problem files
    parser.parse_domain(domain_file)
    parser.parse_problem(problem_file)
    
    # Initialize arrays
    cups = []
    locations = []
    buttons = []
    robot_location = None
    
    # Get robot location from initial state
    for state_tuple in parser.state:
        if len(state_tuple) == 2 and state_tuple[0] == 'robot-at':
            robot_location = state_tuple[1]
            break
    
    # If no robot location found, default to a location
    if robot_location is None:
        print("Warning: No robot-at predicate found. Defaulting to 'loc1'.")
        robot_location = 'loc1'

    
    # Helper function to get initial state predicates for an object
    def get_object_predicates(obj_name):
        predicates = {}
        for state_tuple in parser.state:
            if len(state_tuple) >= 2:
                if state_tuple[0] == 'clean' and state_tuple[1] == obj_name:
                    predicates['clean'] = True
                elif state_tuple[0] == 'empty' and state_tuple[1] == obj_name:
                    predicates['empty'] = True
                elif state_tuple[0] == 'full' and state_tuple[1] == obj_name:
                    predicates['full'] = True
                elif state_tuple[0] == 'dirty' and state_tuple[1] == obj_name:
                    predicates['dirty'] = True
                elif state_tuple[0] == 'at' and len(state_tuple) == 3 and state_tuple[1] == obj_name:
                    predicates['location'] = state_tuple[2]
        return predicates
    
    # Process cups
    for cup_type in ['espresso-cup', 'cappuccino-cup', 'americano-cup']:
        if cup_type in parser.objects:
            for cup_name in parser.objects[cup_type]:
                predicates = get_object_predicates(cup_name)
                
                # Get location
                location = predicates.get('location', None)  # default to loc1
                if location is None:
                    print(f"Warning: No location found for {cup_name}. Defaulting to 'loc1'.")
                    location = 'loc1'
                
                # Get states (clean, empty, full)
                is_clean = predicates.get('clean', False)
                is_empty = predicates.get('empty', True)  # default to empty
                is_full = predicates.get('full', False)
                is_dirty = predicates.get('dirty', False)

                # Format: 'type name location clean empty full dirty'
                cup_entry = f"{cup_type} {cup_name} {location} {is_clean} {is_empty} {is_full} {is_dirty}"
                cups.append(cup_entry)
    
    # Process locations
    for location_type in ['dirty-area', 'clean-area', 'coffee-machine', 'serving-counter']:
        if location_type in parser.objects:
            for location_name in parser.objects[location_type]:
                # Format: 'type name'
                location_entry = f"{location_type} {location_name}"
                locations.append(location_entry)
    
    # Process buttons
    for button_type in ['espresso-button', 'cappuccino-button', 'americano-button']:
        if button_type in parser.objects:
            for button_name in parser.objects[button_type]:
                buttons.append(button_name)
    
    return cups, locations, buttons, robot_location