import sys
import os

# Add the pddl-parser directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'knowledge', 'pddl-parser'))

from pddl_parser.planner import Planner

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

def plan_and_parse():
    """Run a PDDL planner on domain and problem files to produce a sequence of ground actions (aka a plan)"""    

    domain_file = "knowledge/PDDL/domain.pddl"
    problem_file = "knowledge/PDDL/problem.pddl"

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