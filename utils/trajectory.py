class Trajectory:
    def __init__(self, groundings, image, action):
        self.groundings = groundings
        self.image = image
        self.action = action

def save_to_json(trajectories):
    # Save groundings, action, and image path for each trajectory to JSON
    import json 
    # Get datetime
    import datetime
    timestamp = datetime.datetime.now().timestamp()

    data = []
    for traj in trajectories:
        traj_data = {
            "groundings": traj.groundings,
            "action": traj.action,
            "image": traj.image
        }
        data.append(traj_data)

    with open("trajectories/trajectories.json", "w") as f:
        json.dump(data, f)