# AStar-JPS-ThetaStar
Basic algorithms for single-shot grid-based 2D path finding.

Single-shot means that the algorithms are tailored to answering a single pathfinding query (as opposed to other pathfinders that are suited to solving sequences of alike queries, e.g. queries on the same map but with different start and goal locations).

Grid-based means that a (regular square) grid is an essential part of the input. We follow center-based assumption which implies that distinct agent's locations are tied to the centers of the traversable cells (not to the corners). 

Description
==========
This project contains implementations of the following algorithms:
- Breadth First Search
- Dijkstra's algorithm
- A*
- Theta*
- Jump Point Search (JPS)

Build and Launch
================
To build the project you can use QMake (part of QT toolchain) or CMake. Both .pro and CMakeLists files are available in the repo.
Please note that the code relies on C++11 standart. Make sure that your compiler supports it.
The project does not use any external libraries (except tinyXML which is linked at the source level) and is meant to be cross-platform.

To launch the compiled file and get a result you need to pass a correct input XML-file (see below) as the first (command line) argument. This file encodes all the information about path finding instance (map, start, goal, algorithm's options etc.).

If the input is correct you'll see the result in the console. Detailed XML log-file will also be created by default. If you choose not to create output XML you can alter the input XML following the instructions given below.

Input and Output files
======================
Both files are XML files of a specific structure. 
Input file should contain:
>- Mandatory tag <b>\<map></b>. It describes the environment.
  * **\<height>** and **\<width>** - mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width*-1, *height*-1) is lower right.
  * **\<startx>** and **\<starty>** - mandatory tags that define horizontal (X) and vertical (Y) offset of the start location from the upper left corner. Legal values for *startx* are [0, .., *width*-1], for *starty* - [0, .., *height*-1].
  * **\<finishx>** and **\<finishy>** - mandatory tags that define horizontal (X) and vertical (Y) offset of the goal location.
  * **\<grid>** - mandatory tag that describes the square grid constituting the map. It consists of **\<row>** tags. Each **\<row>** contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" - for untraversable (actually any other figure but "0" can be used instead of "1").
  * **\<cellsize>** - optional tag that defines the size of one cell. One might add it to calculate scaled length of the path.
  * **\<title>**, **\<URL>**, **\<coordinates>**, etc - optional tags containing additional information on the map.
>- Mandatory tag <b>\<algorithm></b>. It describes the parameters of the algorithm.
  * **\<searchtype>** - mandatory tag that defines which algorithm will be used for pathfinding. Possible values - "astar", "theta", "jp_search", "bfs", "dijkstra".
  * **\<metrictype>** - defines the type of metric for heuristic function. Possible values - "euclid", "diagonal", "manhattan", "chebyshev". Default value is "euclid". Please note that the lenghs of the actual moves (not the heuristic to goal) are always counted with Euclid metrics, e.g. the length of the move between two cardinally adjacent cells is 1, the length of the move between two diagonally adjacent cells is sqrt(2) etc.
  * **\<hweight>** - defines the weight of the heuristic function. Should be real number greater or equal 1. Default value is "1".
  * **\<breakingties>** - defines the priority in OPEN list for nodes with equal f-values. Possible values - "g-min" (break ties in favor of the node with smaller g-value), "g-max" (break ties in favor of the node with greater g-value). Default value is "g-max".
  * **\<allowdiagonal>** - boolean tag that allows/prohibits diagonal moves. Default value is "true". Setting it to "false" restricts the agent to make cardinal (horizonal, vertical) moves only. If Theta* algorithm is used only cardinal successors will be generated during expansion of the current node, but after resetting parent the resultant move will probably violate this restriction. 
  * **\<cutcorners>** - boolean tag that defines the possibilty to make diagonal moves when one adjacent cell is untraversable. The tag is ignored if diagonal moves are not allowed. Default value is "false".
  * **\<allowsqueeze>** - boolean tag that defines the possibility to make diagonal moves when both adjacent cells are untraversable. The tag is ignored if cutting corners is not allowed. Default value is "false".
  * **\<postsmoothing>** - boolean tag that allows to start the path smoothing procedure after pathfinding was ended. The tag is ignored if Theta* algorithm was chosen. Default value is "false".
>- Optional tag <b>\<options></b>. Options that are not related to search.
  * **\<loglevel>** - defines the level of detalization of log-file. Default value is "1". Possible values:
    * "0" or "none" - log-file is not created.
    * "0.5" or "tiny" - All the input data is copied to the log-file plus short **\<summary>** is appended. **\<summary>** contains info of the path length, number of steps, elapsed time, etc.
    * "1" or "short" - *0.5*-log plus **\<path>** is appended. It looks like **\<grid>** but cells forming the path are marked by "\*" instead of "0". The following tags are also appended: **\<hplevel>** and **\<lplevel>**. **\<lplevel>** is the sequence of coordinates of cells forming the path (in case Theta* planner is used, this sequence is formed at post-processing step by invoking sequantually line-of-sight procedure on path's segments). **\<hplevel>** is the sequence of sections forming the path (in case planner other from Theta* is used, sections are formed at post-processing step using naive procedure).
    * "1.5" or "medium" - *1*-log plus the information (explicit enumeration) on last iteration's OPEN and CLOSE lists.
    * "2" or "full" - *1*-log plus OPEN and CLOSE lists are written into the log-file after each step of the algorithm. Can make log-files really huge.
  * **\<logpath>** - defines the directory where the log-file should be written. If not specified directory of the input file is used. 
  * **\<logname>** - defines the name of log-file. If not specified the name of the log file is: "input file name"+"_log"+input file extension.

Python Wrapper
==============
This project also includes a Python wrapper allowing you to use the A* and Theta* path planning functionalities from Python.

Prerequisites
-------------
-   **CMake** (version 3.1 or higher recommended)
-   A C++ compiler that supports C++11 (e.g., GCC, Clang, MSVC)
-   **Python** (version 3.6 or higher recommended)
-   **pybind11**: This will be used for creating the Python bindings. You can install it via pip:
    ```bash
    pip install pybind11
    ```
-   **setuptools** (usually comes with Python/pip):
    ```bash
    pip install setuptools
    ```

Building the Python Module
--------------------------
The Python module `planner_cpp` is built using CMake.

1.  **Navigate to the project root directory** (the one containing this README).
2.  **Create a build directory and navigate into it:**
    ```bash
    mkdir build
    cd build
    ```
3.  **Run CMake to configure the project:**
    *   On Linux/macOS:
        ```bash
        cmake ../src
        ```
    *   On Windows (if using Visual Studio, you might need to specify a generator, e.g., `cmake ../src -G "Visual Studio 16 2019" -A x64`). Simpler for command line (like MinGW):
        ```bash
        cmake ../src
        ```
    This command tells CMake to look for the `CMakeLists.txt` in the `../src` directory.
4.  **Compile the project:**
    *   On Linux/macOS:
        ```bash
        make
        ```
    *   On Windows (with MSVC):
        ```bash
        cmake --build . --config Release
        ```
        (Or open the generated solution file in Visual Studio and build).
        If using MinGW makefiles:
        ```bash
        mingw32-make
        ```

    After a successful build, you should find the Python module file (e.g., `planner_cpp.cpython-38-x86_64-linux-gnu.so` on Linux, `planner_cpp.cp38-win_amd64.pyd` on Windows) inside the `build` directory (or a subdirectory like `build/Release` depending on your CMake generator and build type).

Running the Example
-------------------
An example Python script `example.py` is provided in the root directory.

1.  **Ensure the Python module is built** as described above.
2.  **Run the example script from the project root directory:**
    ```bash
    python example.py
    ```
    The `example.py` script attempts to locate the compiled module in common build directory locations (e.g., `build/`, `build/Release/`).

    If you encounter an `ImportError`, ensure that:
    *   The module (e.g., `planner_cpp...so` or `planner_cpp...pyd`) exists in your build directory.
    *   The build directory is in your `PYTHONPATH` environment variable, or you are running the script in a way that Python can find the module (the script tries to add the build directory to `sys.path` automatically, but this might not cover all build configurations). For example, you can copy the `.so`/`.pyd` file from the build directory to the project root directory next to `example.py`.

Using the Wrapper in Your Own Python Code
-----------------------------------------
1.  Make sure the compiled `planner_cpp` module is in your Python path.
2.  Import the module and use the `plan_2d` function:

```python
import planner_cpp # Or ensure the .so/.pyd file is in your PYTHONPATH

origin = [0.0, 0.0]
dim = [100, 100] # height, width
# Map data: flat list, row-major, 0=free, 1=obstacle
map_data = [0] * (dim[0] * dim[1])
# Populate map_data with obstacles as needed, e.g.:
# map_data[row * dim[1] + col] = 1

start_coords = [0.5, 0.5] # meters
goal_coords = [9.5, 9.5] # meters
map_resolution = 0.1 # meters/cell
use_theta_star = False # False for A*, True for Theta*

status, path, time_ms = planner_cpp.plan_2d(
    origin,
    dim,
    map_data,
    start_coords,
    goal_coords,
    map_resolution,
    use_theta_star
)

if status == 0:
    print(f"Path found: {path}")
    print(f"Time taken: {time_ms} ms")
else:
    print("Failed to find path.")

```
The `map_data` list should contain integers that can be safely converted to `signed char` by the C++ backend (typically values like 0 for free, 1 for obstacle).
The path is returned as a list of `[x, y]` coordinate pairs in meters.
Time spent is returned in milliseconds.
