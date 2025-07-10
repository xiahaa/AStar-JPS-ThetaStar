#include "mission.h" // Includes map.h (old Map), config.h, etc.
#include <iostream>
#include <vector>
#include <exception>
#include <iomanip> // For std::fixed and std::setprecision

// Headers for the pure Theta* test and new XML data test
#include "map_types.h" // Defines pure_theta_star::GridMap
#include "theta.h"     // Theta algorithm
#include "environmentoptions.h"
#include "searchresult.h"
#include "gl_const.h"   // For constants

// Function to run the pure Theta* test with hardcoded map
void runPureThetaStarTest() {
    std::cout << "\n--- Running Pure Theta* Test (Hardcoded Map) ---" << std::endl;

    std::vector<std::vector<int>> complex_map_data = {
        {0, 0, 0, 0, 0, 0, 0, 0},
        {0, 100, 0, 0, 0, 100, 0, 0}, // Start(1,0)
        {0, 100, 0, 0, 0, 100, 0, 0},
        {0, 0, 0, 100, 0, 0, 0, 0},
        {0, 0, 0, 100, 0, 0, 100, 0}, // Goal(4,7)
        {0, 100, 0, 0, 0, 100, 0, 0},
        {0, 100, 0, 0, 0, 100, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0}
    };
    int start_r = 1; int start_c = 0;
    int goal_r = 4; int goal_c = 7;

    try {
        pure_theta_star::GridMap gridMap(complex_map_data, start_r, start_c, goal_r, goal_c);
        std::cout << "GridMap created: " << gridMap.getWidth() << "x" << gridMap.getHeight() << std::endl;
        std::cout << "Start: (" << gridMap.getStartRow() << "," << gridMap.getStartCol() << ")" << std::endl;
        std::cout << "Goal: (" << gridMap.getGoalRow() << "," << gridMap.getGoalCol() << ")" << std::endl;

        EnvironmentOptions options(false, true, true, CN_SP_MT_EUCL); // allow diag, CUT CORNERS = TRUE for Theta*, Euclidean
                                                                    // Note: cutcorners=true is often preferred for Theta* LoS

        double h_weight = 1.0;
        bool breaking_ties_g_max = (CN_SP_BT_GMAX == 1); // Assuming CN_SP_BT_GMAX is 1 for true
        Theta theta_search(h_weight, breaking_ties_g_max);

        std::cout << "Starting Theta* search..." << std::endl;
        SearchResult result = theta_search.startSearch(nullptr, gridMap, options);

        std::cout << "Search finished." << std::endl;
        std::cout << "Path found: " << (result.pathfound ? "Yes" : "No") << std::endl;
        std::cout << "Nodes created (closed+open): " << result.nodescreated << std::endl;
        std::cout << "Number of steps (expanded nodes): " << result.numberofsteps << std::endl;
        std::cout << "Path length (g-cost of goal): " << std::fixed << std::setprecision(5) << result.pathlength << std::endl;
        std::cout << "Time (s): " << std::fixed << std::setprecision(5) << result.time << std::endl;

        if (result.pathfound && result.hppath && !result.hppath->empty()) {
            std::cout << "High-precision path (waypoints):" << std::endl;
            for (const auto& node : *result.hppath) {
                std::cout << "(" << node.i << "," << node.j << ") ";
            }
            std::cout << std::endl;
        }
        if (result.pathfound && result.lppath && !result.lppath->empty()) {
            std::cout << "Low-precision path (cells):" << std::endl;
            for (const auto& node : *result.lppath) {
                std::cout << "(" << node.i << "," << node.j << ") ";
            }
            std::cout << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error during Pure Theta* Test: " << e.what() << std::endl;
    }
    std::cout << "--- Pure Theta* Test (Hardcoded Map) Finished ---" << std::endl;
}

// Function to test the new Theta* implementation with data loaded from an XML file
void testNewThetaWithXmlData(const char* xmlFilePath) {
    std::cout << "\n--- Testing New Theta* with Data from XML: " << xmlFilePath << " ---" << std::endl;

    Map oldMap; // Using the original Map class to load XML
    if (!oldMap.getMap(xmlFilePath)) {
        std::cerr << "Error: Could not load map from XML file: " << xmlFilePath << std::endl;
        return;
    }
    std::cout << "XML Map loaded successfully via old Map class." << std::endl;
    std::cout << "Old Map dimensions: " << oldMap.width << "x" << oldMap.height << std::endl;
    std::cout << "Old Map Start: (" << oldMap.start_i << "," << oldMap.start_j << ")" << std::endl;
    std::cout << "Old Map Goal: (" << oldMap.goal_i << "," << oldMap.goal_j << ")" << std::endl;


    // Convert oldMap.Grid (int**) to std::vector<std::vector<int>>
    // And adjust obstacle representation: old map uses CN_GC_NOOBS (0) for traversable, 1 for obstacle.
    // New GridMap expects 0 for traversable, 100 for obstacle.
    std::vector<std::vector<int>> map_data(oldMap.height, std::vector<int>(oldMap.width));
    for (int r = 0; r < oldMap.height; ++r) {
        for (int c = 0; c < oldMap.width; ++c) {
            if (oldMap.Grid[r][c] == CN_GC_NOOBS) { // CN_GC_NOOBS is 0
                map_data[r][c] = 0;   // Traversable for new GridMap
            } else {
                map_data[r][c] = 100; // Obstacle for new GridMap
            }
        }
    }

    int start_r = oldMap.start_i;
    int start_c = oldMap.start_j;
    int goal_r = oldMap.goal_i;
    int goal_c = oldMap.goal_j;

    try {
        pure_theta_star::GridMap gridMap(map_data, start_r, start_c, goal_r, goal_c);
        std::cout << "Converted GridMap created: " << gridMap.getWidth() << "x" << gridMap.getHeight() << std::endl;
        std::cout << "Converted Start: (" << gridMap.getStartRow() << "," << gridMap.getStartCol() << ")" << std::endl;
        std::cout << "Converted Goal: (" << gridMap.getGoalRow() << "," << gridMap.getGoalCol() << ")" << std::endl;

        // Use consistent EnvironmentOptions, e.g., from the pure test or typical Theta* settings
        EnvironmentOptions options(false, true, true, CN_SP_MT_EUCL); // allow diag, CUT CORNERS = TRUE, Euclidean
                                                                    // Note: cutcorners=true is often preferred for Theta* LoS

        double h_weight = 1.0;
        bool breaking_ties_g_max = (CN_SP_BT_GMAX == 1);
        Theta theta_search(h_weight, breaking_ties_g_max);

        std::cout << "Starting Theta* search with XML-derived data..." << std::endl;
        SearchResult result = theta_search.startSearch(nullptr, gridMap, options);

        std::cout << "Search finished." << std::endl;
        std::cout << "Path found: " << (result.pathfound ? "Yes" : "No") << std::endl;
        std::cout << "Nodes created: " << result.nodescreated << std::endl;
        std::cout << "Number of steps: " << result.numberofsteps << std::endl;
        std::cout << "Path length: " << std::fixed << std::setprecision(5) << result.pathlength << std::endl;
        std::cout << "Time (s): " << std::fixed << std::setprecision(5) << result.time << std::endl;

        if (result.pathfound && result.hppath && !result.hppath->empty()) {
            std::cout << "High-precision path (waypoints) for XML-derived data:" << std::endl;
            for (const auto& node : *result.hppath) {
                std::cout << "(" << node.i << "," << node.j << ") ";
            }
            std::cout << std::endl;
        }
         if (result.pathfound && result.lppath && !result.lppath->empty()) {
            std::cout << "Low-precision path (cells) for XML-derived data:" << std::endl;
            for (const auto& node : *result.lppath) {
                std::cout << "(" << node.i << "," << node.j << ") ";
            }
            std::cout << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error during Theta* Test with XML-derived data: " << e.what() << std::endl;
    }
    std::cout << "--- Test New Theta* with XML Data Finished ---" << std::endl;
}


int main(int argc, char* argv[])
{
    // Run the pure Theta* test with a hardcoded map first
    runPureThetaStarTest();

    // Then, if an XML file is provided, run the test using data from that XML
    if (argc >= 2) {
        testNewThetaWithXmlData(argv[1]);
    } else {
        std::cout << "\nNo XML file specified as command line argument." << std::endl;
        std::cout << "To test with an XML map, provide the path as an argument." << std::endl;
        std::cout << "Example: ./asearch path/to/your/map.xml" << std::endl;
    }

    // The original Mission-based logic is effectively disabled/broken by the ISearch interface changes.
    // Keeping it commented out or removed to avoid confusion.
    /*
    if (argc >= 2) {
        // ... original Mission logic ...
    }
    */

    return 0;
}
