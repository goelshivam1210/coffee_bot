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
    import cv2
    timestamp = datetime.datetime.now().timestamp()
    import os
    if not os.path.exists("trajectories"):
        os.makedirs("trajectories")

    data = []
    for ind, traj in enumerate(trajectories):
        cv2.imwrite(f"trajectories/{timestamp}_{ind}.png", traj.image)
        traj_data = {
            "groundings": traj.groundings,
            "action": traj.action,
            "image": f"trajectories/{timestamp}_{ind}.png"
        }
        data.append(traj_data)

    with open("trajectories/trajectories.json", "w") as f:
        json.dump(data, f)