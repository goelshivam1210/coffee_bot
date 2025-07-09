import numpy as np

COLORS = {
    'red':    (255/255,  87/255,  89/255, 255/255),
    'blue1':  (65/255,  110/255, 180/255, 255/255),
    'purple1': (180/255, 115/255, 155/255, 255/255),
    'yellow1': (240/255, 190/255,  65/255, 255/255),
    'green1':  (89/255,  169/255,  79/255, 255/255),
    'blue2':   (78/255,  121/255, 167/255, 255/255),
    'yellow2': (237/255, 201/255,  72/255, 255/255),
    'purple2': (176/255, 122/255, 161/255, 255/255),
    'green2':  (80/255,  180/255,  70/255, 255/255),
    # Coffee-specific colors
    'brown':  (156/255, 117/255,  95/255, 255/255),
    'white':  (0.9, 0.9, 0.9, 1.0),
    'black':  (0.1, 0.1, 0.1, 1.0),
    'silver': (0.7, 0.7, 0.7, 1.0),
    'coffee_machine': (0.3, 0.3, 0.3, 1.0),
}

# Coffee-specific object configurations
CUP_CONFIGS = {
    'espresso-cup': {'radius': 0.025, 'height': 0.06, 'mass': 0.05, 'color': 'brown'},
    'cappuccino-cup': {'radius': 0.02, 'height': 0.05, 'mass': 0.04, 'color': 'white'},
    'americano-cup': {'radius': 0.03, 'height': 0.08, 'mass': 0.06, 'color': 'black'},
}

BUTTON_CONFIGS = {
    'esp-btn': {'type': 'espresso-button', 'color': 'brown'},
    'cap-btn': {'type': 'cappuccino-button', 'color': 'white'},
    'amer-btn': {'type': 'americano-button', 'color': 'black'},
}

LOCATION_CONFIGS = {
    'loc3': {'type': 'coffee-machine', 'position': (0, -0.7, 0), 'color': 'coffee_machine'},
    'loc1': {'type': 'dirty-area', 'position': (-0.2, -0.35, 0), 'color': 'red'},      # Left third
    'loc2': {'type': 'clean-area', 'position': (0, -0.35, 0), 'color': 'green1'},      # Middle third  
    'loc4': {'type': 'serving-counter', 'position': (0.2, -0.35, 0), 'color': 'silver'}, # Right third
}

CORNER_POS = {
  'top left corner':     (-0.3 + 0.05, -0.2 - 0.05, 0),
  'top side':            (0,           -0.2 - 0.05, 0),
  'top right corner':    (0.3 - 0.05,  -0.2 - 0.05, 0),
  'left side':           (-0.3 + 0.05, -0.5,        0),
  'middle':              (0,           -0.5,        0),
  'right side':          (0.3 - 0.05,  -0.5,        0),
  'bottom left corner':  (-0.3 + 0.05, -0.8 + 0.05, 0),
  'bottom side':         (0,           -0.8 + 0.05, 0),
  'bottom right corner': (0.3 - 0.05,  -0.8 + 0.05, 0),
}

PIXEL_SIZE = 0.00267857
BOUNDS = np.float32([[-0.3, 0.3], [-0.8, -0.2], [0, 0.15]])  # X Y Z