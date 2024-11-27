import random
import math

def random_pose():
    """Generate random x, y within bounds."""
    return random.uniform(1.5, 9.5), random.uniform(1.5, 9.5)

def adjust_for_boundary(x, y):
    """Keeps coordinates within boundaries."""
    if x < 1.0:
        x = 1.0
    elif x > 10.0:
        x = 10.0
    if y < 1.0:
        y = 1.0
    elif y > 10.0:
        y = 10.0
    return x, y

def calculate_distance(pos1, pos2):
    """Calculate Euclidean distance."""
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

