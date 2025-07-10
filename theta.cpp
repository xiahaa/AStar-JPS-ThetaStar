#include "theta.h"
#include <cmath> // For std::abs, std::sqrt, std::pow
// map_types.h is included via theta.h

Theta::~Theta()
{
}

// Corrected and simplified Line of Sight
bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const pure_theta_star::GridMap &gridMap, bool cutcorners) // Namespace corrected
{
    // Check if start or end points are out of bounds first.
    // Obstacle checks for these specific points (i1,j1) and (i2,j2) are done by CellIsObstacle.
    if (!gridMap.CellOnGrid(i1, j1)) return false; // Start point off grid
    if (!gridMap.CellOnGrid(i2, j2)) return false; // End point off grid

    if (gridMap.CellIsObstacle(i1, j1)) return false;
    if (gridMap.CellIsObstacle(i2, j2)) return false;


    if (i1 == i2 && j1 == j2) return true;

    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);

    int current_i = i1;
    int current_j = j1;
    int error;

    if (delta_i == 0) {
        while (current_j != j2 - step_j) {
            current_j += step_j;
            if (gridMap.CellIsObstacle(current_i, current_j)) return false;
        }
        return true;
    } else if (delta_j == 0) {
        while (current_i != i2 - step_i) {
            current_i += step_i;
            if (gridMap.CellIsObstacle(current_i, current_j)) return false;
        }
        return true;
    }

    if (cutcorners) {
        if (delta_i > delta_j) {
            error = (delta_j << 1) - delta_i;
            for (int k = 0; k < delta_i -1; ++k) {
                current_i += step_i;
                if (error >= 0) {
                    current_j += step_j;
                    error -= (delta_i << 1);
                }
                error += (delta_j << 1);
                if (gridMap.CellIsObstacle(current_i, current_j)) return false;
            }
        } else {
            error = (delta_i << 1) - delta_j;
            for (int k = 0; k < delta_j -1; ++k) {
                current_j += step_j;
                if (error >= 0) {
                    current_i += step_i;
                    error -= (delta_j << 1);
                }
                error += (delta_i << 1);
                if (gridMap.CellIsObstacle(current_i, current_j)) return false;
            }
        }
    } else {
        // Reset current_i, current_j for this block if they were modified above
        // (they are not, as they are only modified in horizontal/vertical cases which return early)
        // However, to be safe, or if logic changes:
        current_i = i1;
        current_j = j1;
        if (delta_i > delta_j) {
            error = (delta_j << 1) - delta_i;
            // Iterate from first point up to point before the last along major axis
            for (int iter_count = 0; iter_count < delta_i; ++iter_count) {
                 if (iter_count > 0 && gridMap.CellIsObstacle(current_i, current_j)) return false;

                if (error >= 0) {
                    // Check corner cells before diagonal step
                    if (gridMap.CellIsObstacle(current_i + step_i, current_j) &&
                        gridMap.CellIsObstacle(current_i, current_j + step_j)) {
                        return false;
                    }
                    current_j += step_j;
                    error -= (delta_i << 1);
                }
                error += (delta_j << 1);
                if (iter_count < delta_i) current_i += step_i; // Move along major axis
            }
        } else {
            error = (delta_i << 1) - delta_j;
            for (int iter_count = 0; iter_count < delta_j; ++iter_count) {
                if (iter_count > 0 && gridMap.CellIsObstacle(current_i, current_j)) return false;

                if (error >= 0) {
                     if (gridMap.CellIsObstacle(current_i + step_i, current_j) &&
                         gridMap.CellIsObstacle(current_i, current_j + step_j)) {
                         return false;
                     }
                    current_i += step_i;
                    error -= (delta_j << 1);
                }
                error += (delta_i << 1);
                 if (iter_count < delta_j) current_j += step_j; // Move along major axis
            }
        }
    }
    return true;
}


Node Theta::resetParent(Node current, Node parent, const pure_theta_star::GridMap &gridMap, const EnvironmentOptions &options ) // Namespace corrected
{
    if (parent.parent == nullptr) {
        return current;
    }

    if (lineOfSight(parent.parent->i, parent.parent->j, current.i, current.j, gridMap, options.cutcorners)) {
        current.g = parent.parent->g + distance(parent.parent->i, parent.parent->j, current.i, current.j);
        current.parent = parent.parent;
    }
    return current;
}

double Theta::distance(int i1, int j1, int i2, int j2)
{
    return std::sqrt(std::pow(static_cast<double>(i1) - i2, 2) + std::pow(static_cast<double>(j1) - j2, 2));
}

void Theta::makePrimaryPath(Node curNode)
{
    hppath.clear();
    Node current = curNode;
    while(current.parent != nullptr) {
        hppath.push_front(current);
        current = *current.parent;
    }
    hppath.push_front(current);
}

void Theta::makeSecondaryPath()
{
    if(hppath.empty()) {
        lppath.clear();
        return;
    }

    lppath.clear();

    std::list<Node>::const_iterator it = hppath.begin();
    Node prevNode = *it;
    lppath.push_back(prevNode);

    ++it;
    while(it != hppath.end())
    {
        Node curNode = *it;
        int i1 = prevNode.i;
        int j1 = prevNode.j;
        int i2 = curNode.i;
        int j2 = curNode.j;

        if (i1 == i2 && j1 == j2) {
            prevNode = curNode;
            ++it;
            continue;
        }

        int delta_i = std::abs(i1 - i2);
        int delta_j = std::abs(j1 - j2);
        int step_i = (i1 < i2 ? 1 : -1);
        int step_j = (j1 < j2 ? 1 : -1);

        int current_i = i1;
        int current_j = j1;
        int error;
        Node inpathNode;
        inpathNode.parent = nullptr;
        inpathNode.F = inpathNode.g = inpathNode.H = 0;


        if (delta_i > delta_j) {
            error = (delta_j << 1) - delta_i;
            while(current_i != i2) {
                current_i += step_i;
                if (error >= 0) {
                    current_j += step_j;
                    error -= (delta_i << 1);
                }
                error += (delta_j << 1);
                inpathNode.i = current_i;
                inpathNode.j = current_j;
                lppath.push_back(inpathNode);
            }
        } else {
            error = (delta_i << 1) - delta_j;
            while(current_j != j2) {
                current_j += step_j;
                if (error >= 0) {
                    current_i += step_i;
                    error -= (delta_j << 1);
                }
                error += (delta_i << 1);
                inpathNode.i = current_i;
                inpathNode.j = current_j;
                lppath.push_back(inpathNode);
            }
        }
        prevNode = curNode;
        ++it;
    }
}
