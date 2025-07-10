#ifndef THETA_H
#define THETA_H
#include "astar.h"
#include "map_types.h" // Added for GridMap
// EnvironmentOptions is included via astar.h -> isearch.h -> environmentoptions.h

class Theta: public Astar
{
    public:
        Theta(double hweight, bool breakingties):Astar(hweight, breakingties){}
        ~Theta(void);
        // Changed Map to GridMap
        static bool lineOfSight(int i1, int j1, int i2, int j2, const प्योर_थीटा_स्टार::GridMap &gridMap, bool cutcorners);
        static double distance(int i1, int j1, int i2, int j2); // Stays the same
    protected:


        // Changed Map to GridMap
        Node resetParent(Node current, Node parent, const प्योर_थीटा_स्टार::GridMap &gridMap, const EnvironmentOptions &options);
        // makePrimaryPath and makeSecondaryPath are virtual methods from ISearch,
        // their signatures in the base class will be updated later if they depend on Map.
        // For Theta's direct override, the signature doesn't take Map.
        // These are inherited and called by the A* search process.
        // void makePrimaryPath(Node curNode); // Definition in ISearch or Astar
        // void makeSecondaryPath();           // Definition in ISearch or Astar

};

#endif // THETA_H
