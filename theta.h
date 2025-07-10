#ifndef THETA_H
#define THETA_H
#include "astar.h"
#include "map_types.h" // Added for GridMap (now uses pure_theta_star namespace)
// EnvironmentOptions is included via astar.h -> isearch.h -> environmentoptions.h

class Theta: public Astar
{
    public:
        Theta(double hweight, bool breakingties):Astar(hweight, breakingties){}
        ~Theta(void);
        // Changed Map to GridMap and updated namespace
        static bool lineOfSight(int i1, int j1, int i2, int j2, const pure_theta_star::GridMap &gridMap, bool cutcorners);
        static double distance(int i1, int j1, int i2, int j2); // Stays the same
    protected:


        // Changed Map to GridMap and updated namespace
        Node resetParent(Node current, Node parent, const pure_theta_star::GridMap &gridMap, const EnvironmentOptions &options);
        // makePrimaryPath and makeSecondaryPath are virtual methods from ISearch.
        // Their definitions are in ISearch/AStar or overridden by Theta if needed.
        // Their signatures in ISearch were already updated if they took the map object.
};

#endif // THETA_H
