#ifndef ISEARCH_H
#define ISEARCH_H

#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "map_types.h" // Added for GridMap. The old map.h should not be included by this file anymore.

#include <list>
#include <vector>
#include <unordered_map>
#include <cmath>        // For sqrt, pow, fabs
#include <functional>   // For std::hash

// Forward declaration of Node might be useful if Node.h isn't included by all users of ISearch.
// However, Node is used in signatures, so its definition needs to be available.
// Assuming node.h is included by files that include isearch.h, or by a common header.
#include "node.h"


class ISearch
{
    public:
        ISearch();
        virtual ~ISearch(void);

        // Main search function. Changed 'const Map&' to 'const प्योर_थीटा_स्टार::GridMap&'.
        SearchResult startSearch(ILogger *Logger, const प्योर_थीटा_स्टार::GridMap &gridMap, const EnvironmentOptions &options);

    protected:
        // Open list operations
        Node findMin(); // Finds the node with the minimum F-value in the open list.
        virtual void addOpen(Node newNode); // Adds a new node to the open list.

        // Abstract heuristic function. Must be implemented by derived classes (A*, Theta*).
        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options) = 0;

        // Finds valid neighboring cells (successors) of a given node.
        // Changed 'const Map&' to 'const प्योर_थीटा_स्टार::GridMap&'.
        virtual std::list<Node> findSuccessors(Node curNode, const प्योर_थीटा_स्टार::GridMap &gridMap, const EnvironmentOptions &options);

        // Path reconstruction methods.
        virtual void makePrimaryPath(Node curNode); // Constructs hppath (high-level path or waypoints).
        virtual void makeSecondaryPath();           // Constructs lppath (low-level, detailed cell path from hppath).

        // Theta* specific: Updates a node's parent if a direct line of sight to grandparent is better.
        // Changed 'const Map&' to 'const प्योर_थीटा_स्टार::GridMap&'.
        virtual Node resetParent(Node current, Node parent, const प्योर_थीटा_स्टार::GridMap &gridMap, const EnvironmentOptions &options) { return current; } // Default for non-Theta*

        // Condition to terminate the search (e.g., goal found or open list empty).
        virtual bool stopCriterion();

        // Search process variables
        SearchResult                    sresult;    // Stores results like path length, nodes created, time.
        std::list<Node>                 lppath;     // Detailed path (list of cells).
        std::list<Node>                 hppath;     // Waypoint path (used by Theta*).

        // Closed set: stores nodes that have already been expanded.
        // Using a hash of coordinates for the key for efficient lookup.
        // Example key: (i << 16) | j for a map up to 2^16 width/height.
        // A struct that combines i and j and has a hash function is also possible.
        // The original used `std::unordered_map<int, Node> close;`
        // We need a robust way to generate the int key from (i,j) or use a Node hasher.
        // For now, stick to the original int key, assuming it's `i * mapWidth + j` or similar,
        // which needs mapWidth available or a different hashing.
        // Let's define a simple struct for keys in unordered_map for clarity.
        struct PairHash {
            template <class T1, class T2>
            std::size_t operator () (const std::pair<T1,T2> &p) const {
                auto h1 = std::hash<T1>{}(p.first);
                auto h2 = std::hash<T2>{}(p.second);
                // Simple hash combining technique
                return h1 ^ (h2 << 1);
            }
        };
        std::unordered_map<std::pair<int,int>, Node, PairHash>    close; // (i,j) pair as key

        // Open list: a common implementation is a priority queue.
        // The original `std::vector<std::list<Node>> open;` suggests a bucket-based approach (e.g., F-value buckets).
        std::vector<std::list<Node>>    open;       // Open list structure (e.g., buckets for F-values).
        int                             openSize;   // Total number of nodes in the open list.

        // Algorithm tuning parameters
        double                          hweight;        // Heuristic weight (for A*, Theta*).
        bool                            breakingties;   // Tie-breaking policy (e.g., g-min or g-max).

        // Goal coordinates, useful for computeH and findSuccessors.
        // These will be initialized in startSearch from the gridMap.
        int current_goal_i;
        int current_goal_j;
        // Current map dimensions, useful for hashing or boundary checks if not using GridMap methods always
        int current_map_height;
        int current_map_width;

};
#endif
