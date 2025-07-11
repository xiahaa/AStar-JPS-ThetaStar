import sys
import os

# Add the build directory to the Python path so it can find planner_cpp.
# This assumes the script is run from the root of the project and the build
# directory is 'build' or 'build/Debug' or 'build/Release' etc.
# A more robust solution for distribution would be a proper setup.py install.

# Try to find the module in common build directory locations
module_found = False
build_dir_candidates = [
    'build',
    'build/Debug',
    'build/Release',
    'build/RelWithDebInfo',
    'build/MinSizeRel',
    # For multi-config generators like Visual Studio, pybind11 often places
    # the .pyd file in a subdirectory matching the configuration within the
    # main Python extension directory (e.g., build/lib.win-amd64-3.8/Debug)
    # This simple path addition might not be enough for all CMake generators/platforms.
    # Users might need to set PYTHONPATH or copy the module manually.
]

# Add current directory in case the module is placed there by some build systems
# or for manual placement.
sys.path.insert(0, os.path.abspath("."))


for candidate_dir in build_dir_candidates:
    # Construct path relative to this script's location (project root)
    abs_candidate_dir = os.path.abspath(candidate_dir)
    if os.path.isdir(abs_candidate_dir):
        sys.path.insert(0, abs_candidate_dir)
        # Also check common pybind11 output subdirectories (e.g., for Windows)
        # Example: build/Debug/planner_cpp.pyd or build/planner_cpp.pyd
        # pybind11 might also place it in a subfolder like 'Release' inside the build dir.
        # This is a simplified search.

try:
    import ThetaStarPlanner
    module_found = True
except ImportError as e:
    print(f"Error importing ThetaStarPlanner: {e}")
    print("Please ensure that ThetaStarPlanner module is built and either:")
    print("1. The script is run from a directory where Python can find it (e.g., build directory after CMake).")
    print("2. The build directory (e.g., 'build' or 'build/Release') is in your PYTHONPATH.")
    print(f"Current sys.path: {sys.path}")
    exit(1)

def main():
    import cv2
    import numpy as np
    filename = "./data/image.png"
    print(f"Loading image from {filename}...")
    img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)  # Load as grayscale

    img = cv2.resize(img, (400, 300))  # Resize to 400x300 pixels for testing

    origin = [0,0]  # x, y of the map origin
    dim = [img.shape[1], img.shape[0]]  # width, height in pixels
    resolution = 1  # meters per pixel (assumed)
    print(dim)

    map_data = []
    # transform image to map data, for each pixel, if the value is over 200, it is marked as 100 occipied, otherwise 0 free
    for r in range(dim[1]):  # rows (y)
        row_data = []
        for c in range(dim[0]):  # cols (x)
            pixel_value = img[r, c]
            if isinstance(pixel_value, np.ndarray):
                # If pixel is multi-channel (e.g., RGB), take the first channel
                pixel_value = pixel_value[0]
            if pixel_value > 200:  # Assuming a threshold for occupied
                row_data.append(1)  # Occupied
            else:
                row_data.append(0)
        map_data.extend(row_data)

    # random pick start and goal points that are free and at least 1000 pixels apart
    free_indices = [i for i, v in enumerate(map_data) if v == 0]
    print(f"Number of free cells: {len(free_indices)}")
    if len(free_indices) < 2:
        print("Not enough free cells to pick start and goal points.")
        return
    import random
    start_index = random.choice(free_indices)
    goal_index = random.choice(free_indices)
    while abs(start_index - goal_index) < 100:  # Ensure at least 1000 pixels apart
        goal_index = random.choice(free_indices)
    print(f"Start index: {start_index}, Goal index: {goal_index}")
    print(f"Start cell value: {map_data[start_index]}, Goal cell value: {map_data[goal_index]}")
    # Print a small region around start and goal for debugging
    def print_region(idx, label):
        x = idx % dim[0]
        y = idx // dim[0]
        print(f"{label} region (centered at {x},{y}):")
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                nx, ny = x+dx, y+dy
                if 0 <= nx < dim[0] and 0 <= ny < dim[1]:
                    print(f"{map_data[ny*dim[0]+nx]:3}", end=" ")
                else:
                    print("  X", end=" ")
            print()
    print_region(start_index, "Start")
    print_region(goal_index, "Goal")

    # Swap start_w and goal_w to (row, col) if needed
    start_w = [start_index % dim[0] * resolution, start_index // dim[0] * resolution]  # [row, col]
    goal_w = [goal_index % dim[0] * resolution, goal_index // dim[0] * resolution]    # [row, col]

    print(f"Map Origin: {origin}")
    print(f"Map Dimensions (cells): {dim}")
    print(f"Map Resolution: {resolution}")
    print(f"Start (world): {start_w}")
    print(f"Goal (world): {goal_w}")

    print("Successfully imported ThetaStarPlanner module.")

    # Plan using A*
    print("\nPlanning with A*...")
    use_theta_star_false = False
    print("Calling ThetaStarPlanner.plan_2d with:")
    print(f"  origin={origin}")
    print(f"  dim={dim}")
    print(f"  start_w={start_w}")
    print(f"  goal_w={goal_w}")
    print(f"  resolution={resolution}")
    print(f"  use_theta_star={use_theta_star_false}")
    status_astar, path_astar, time_astar = ThetaStarPlanner.plan_2d(
        origin, dim, map_data, start_w, goal_w, resolution, use_theta_star_false
    )

    if status_astar == 0:
        print("A* Path Found Successfully!")
        print(f"  Path: {path_astar}")
        print(f"  Time spent: {time_astar:.4f} ms")
    else:
        print("A* Failed to find a path.")
        print(f"  Status code: {status_astar}")
        print(f"  Time spent: {time_astar:.4f} ms")

    # Plan using Theta*
    print("\nPlanning with Theta*...")
    use_theta_star_true = True
    print("Calling ThetaStarPlanner.plan_2d with:")
    print(f"  origin={origin}")
    print(f"  dim={dim}")
    print(f"  start_w={start_w}")
    print(f"  goal_w={goal_w}")
    print(f"  map value of start and goal: {map_data[start_w[1] * dim[0] + start_w[0]]}, {map_data[goal_w[1] * dim[0] + goal_w[0]]}")
    print(f"  resolution={resolution}")
    print(f"  use_theta_star={use_theta_star_true}")
    status_theta, path_theta, time_theta = ThetaStarPlanner.plan_2d(
        origin, dim, map_data, start_w, goal_w, resolution, use_theta_star_true
    )

    if status_theta == 0:
        print("Theta* Path Found Successfully!")
        print(f"  Path: {path_theta}")
        print(f"  Time spent: {time_theta:.4f} ms")
    else:
        print("Theta* Failed to find a path.")
        print(f"  Status code: {status_theta}")
        print(f"  Time spent: {time_theta:.4f} ms")

    # Visualization using matplotlib
    import matplotlib.pyplot as plt

    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img_color)

    # Draw start and goal
    ax.scatter([start_w[1]], [start_w[0]], c='lime', s=80, marker='o', label='Start')
    ax.scatter([goal_w[1]], [goal_w[0]], c='red', s=80, marker='o', label='Goal')

    # Draw A* path (blue)
    if status_astar == 0 and path_astar:
        path_astar_y = [pt[0] for pt in path_astar]
        path_astar_x = [pt[1] for pt in path_astar]
        ax.plot(path_astar_x, path_astar_y, color='blue', linewidth=2, label="A* Path")

    # Draw Theta* path (magenta)
    if status_theta == 0 and path_theta:
        path_theta_y = [pt[0] for pt in path_theta]
        path_theta_x = [pt[1] for pt in path_theta]
        ax.plot(path_theta_x, path_theta_y, color='magenta', linewidth=2, label="Theta* Path")

    ax.legend()
    ax.set_title("Planned Path")
    plt.axis('off')
    plt.savefig("./data/planned_path.png", bbox_inches='tight', pad_inches=0.1)
    print("Saved planned path visualization to ./data/planned_path.png")
    plt.show()


if __name__ == "__main__":
    main()
