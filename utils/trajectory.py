class Trajectory:
    def __init__(self, groundings, image, bbs, action):
        self.groundings = groundings
        self.image = image
        self.bbs = bbs
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
    if not os.path.exists(f"trajectories/{timestamp}"):
        os.makedirs(f"trajectories/{timestamp}")

    data = []
    bounding_boxes = []
    for ind, traj in enumerate(trajectories):
        image_bgr = cv2.cvtColor(traj.image, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR for OpenCV
        cv2.imwrite(f"trajectories/{timestamp}/{ind}.png", image_bgr)
        traj_data = {
            "groundings": traj.groundings,
            "action": traj.action,
            "image": f"trajectories/{timestamp}/{ind}.png"
        }
        bbs_data = {
            "image": f"trajectories/{timestamp}/{ind}.png",
            "bounding_boxes": traj.bbs
        }
        data.append(traj_data)
        bounding_boxes.append(bbs_data)
    print(bounding_boxes)

    with open(f"trajectories/{timestamp}/trajectories.json", "w") as f:
        json.dump(data, f)

    with open(f"trajectories/{timestamp}/bounding_boxes.json", "w") as f:
        json.dump(bounding_boxes, f)