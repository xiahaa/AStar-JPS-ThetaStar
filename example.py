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
    import planner_cpp
    module_found = True
except ImportError as e:
    print(f"Error importing planner_cpp: {e}")
    print("Please ensure that planner_cpp module is built and either:")
    print("1. The script is run from a directory where Python can find it (e.g., build directory after CMake).")
    print("2. The build directory (e.g., 'build' or 'build/Release') is in your PYTHONPATH.")
    print(f"Current sys.path: {sys.path}")
    exit(1)

def main():
    print("Successfully imported planner_cpp module.")

    # Example Usage
    origin = [0.0, 0.0] # Map origin (meters)
    dim = [10, 10]      # Map dimensions [height, width] (cells)
    resolution = 0.1    # Map resolution (meters/cell)

    # Create a simple map: 10x10 grid
    # 0 = free, 1 = obstacle
    # Flattened row-major order
    map_data = [0] * (dim[0] * dim[1])

    # Add some obstacles
    # map_data[index] where index = row * width + col
    # Example: place an obstacle at (row=1, col=2) -> 1 * 10 + 2 = 12
    if dim[0] > 1 and dim[1] > 2: map_data[1 * dim[1] + 2] = 1
    if dim[0] > 2 and dim[1] > 2: map_data[2 * dim[1] + 2] = 1
    if dim[0] > 3 and dim[1] > 2: map_data[3 * dim[1] + 2] = 1
    if dim[0] > 4 and dim[1] > 2: map_data[4 * dim[1] + 2] = 1
    if dim[0] > 5 and dim[1] > 2: map_data[5 * dim[1] + 2] = 1 # Wall

    # Start and Goal in meters
    start_m = [0.05, 0.05] # Cell (0,0)
    goal_m = [0.85, 0.85]   # Cell (8,8)

    print(f"Map Origin: {origin}")
    print(f"Map Dimensions (cells): {dim}")
    print(f"Map Resolution: {resolution} m/cell")
    print(f"Start (meters): {start_m}")
    print(f"Goal (meters): {goal_m}")
    # print(f"Map Data (first 20 elements): {map_data[:20]}...") # Uncomment to see part of the map

    # Plan using A*
    print("\nPlanning with A*...")
    use_theta_star_false = False
    status_astar, path_astar, time_astar = planner_cpp.plan_2d(
        origin, dim, map_data, start_m, goal_m, resolution, use_theta_star_false
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
    status_theta, path_theta, time_theta = planner_cpp.plan_2d(
        origin, dim, map_data, start_m, goal_m, resolution, use_theta_star_true
    )

    if status_theta == 0:
        print("Theta* Path Found Successfully!")
        print(f"  Path: {path_theta}")
        print(f"  Time spent: {time_theta:.4f} ms")
    else:
        print("Theta* Failed to find a path.")
        print(f"  Status code: {status_theta}")
        print(f"  Time spent: {time_theta:.4f} ms")


if __name__ == "__main__":
    main()
