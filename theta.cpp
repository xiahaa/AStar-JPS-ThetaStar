#include "theta.h"
#include <cmath> // For std::abs, std::sqrt, std::pow
// map_types.h is included via theta.h

Theta::~Theta()
{
}

// Corrected and simplified Line of Sight
bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const प्योर_थीटा_स्टार::GridMap &gridMap, bool cutcorners)
{
    // Check if start or end points are out of bounds first.
    // Obstacle checks for these specific points (i1,j1) and (i2,j2) are done by CellIsObstacle.
    if (!gridMap.CellOnGrid(i1, j1)) return false; // Start point off grid
    if (!gridMap.CellOnGrid(i2, j2)) return false; // End point off grid

    // If either start or end is an obstacle, no LoS (unless they are the same point and it's an obstacle)
    // Theta* typically assumes LoS is between valid, traversable points.
    // If the algorithm can select an obstacle as a parent or current node, that's a different issue.
    // For LoS itself, if (i1,j1) or (i2,j2) are obstacles, path is blocked.
    if (gridMap.CellIsObstacle(i1, j1)) return false;
    if (gridMap.CellIsObstacle(i2, j2)) return false;


    if (i1 == i2 && j1 == j2) return true; // LoS to self is true if valid (checked above)

    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);

    int current_i = i1;
    int current_j = j1;
    int error;

    if (delta_i == 0) { // Horizontal line
        // Iterate through intermediate cells
        while (current_j != j2 - step_j) { // Stop before the last cell
            current_j += step_j;
            if (gridMap.CellIsObstacle(current_i, current_j)) return false;
        }
        return true; // All intermediate cells are clear
    } else if (delta_j == 0) { // Vertical line
        while (current_i != i2 - step_i) { // Stop before the last cell
            current_i += step_i;
            if (gridMap.CellIsObstacle(current_i, current_j)) return false;
        }
        return true; // All intermediate cells are clear
    }

    // Bresenham's line algorithm for intermediate cells
    // The start (i1,j1) and end (i2,j2) points are already confirmed traversable.
    // We only need to check cells strictly between them.
    if (cutcorners) { // Standard Bresenham: checks the direct line of cells
        if (delta_i > delta_j) {
            error = (delta_j << 1) - delta_i;
            for (int k = 0; k < delta_i -1; ++k) { // Iterate delta_i-1 times for intermediate cells
                current_i += step_i;
                if (error >= 0) {
                    current_j += step_j;
                    error -= (delta_i << 1);
                }
                error += (delta_j << 1);
                if (gridMap.CellIsObstacle(current_i, current_j)) return false;
            }
        } else { // delta_j >= delta_i
            error = (delta_i << 1) - delta_j;
            for (int k = 0; k < delta_j -1; ++k) { // Iterate delta_j-1 times
                current_j += step_j;
                if (error >= 0) {
                    current_i += step_i;
                    error -= (delta_j << 1);
                }
                error += (delta_i << 1);
                if (gridMap.CellIsObstacle(current_i, current_j)) return false;
            }
        }
    } else { // No cutcorners: checks additional cells to prevent corner cutting
        // This version is more complex and needs careful implementation.
        // A common way is to check 2x2 blocks or ensure that diagonal steps
        // don't pass between two diagonal obstacles.
        // The original code's "no cutcorners" was very specific.
        // For simplicity and robustness, a standard "thick" Bresenham or checking
        // the two cells that a diagonal would pass "between" is common.
        // Example: if moving from (0,0) to (1,1) (SE), check (1,0) AND (0,1). If both obstacles, LoS blocked.
        // If only one is, LoS might be allowed. If neither, LoS allowed.

        // Reverting to a structure closer to the original for "no cutcorners"
        // while ensuring it uses GridMap correctly.
        // The original's 'no cutcorners' also had complex conditions.
        // Let's use a fairly standard interpretation of no-cutcorners Bresenham.
        // This means when moving diagonally, both cells forming the "corner" must be clear.
        i = i1; j = j1; // reset current_i, current_j to i,j for clarity
        if (delta_i > delta_j) {
            error = (delta_j << 1) - delta_i;
            for (current_i = i1; current_i != i2; current_i += step_i) {
                if (current_i != i1 && gridMap.CellIsObstacle(current_i, current_j)) return false; // Check current point on line (pixel)

                if (error >= 0) { // Potential diagonal step
                    // Check cells that would be "cut" if we moved from (current_i, current_j) to (current_i+step_i, current_j+step_j)
                    // These are (current_i+step_i, current_j) and (current_i, current_j+step_j)
                    if (gridMap.CellIsObstacle(current_i + step_i, current_j) &&
                        gridMap.CellIsObstacle(current_i, current_j + step_j)) {
                        return false; // Blocked if both "corner" cells are obstacles
                    }
                    current_j += step_j;
                    error -= (delta_i << 1);
                }
                error += (delta_j << 1);
                // The point (current_i+step_i, current_j) after this loop iteration is the next point.
                // The very last point (i2,j2) is not checked by this loop structure if it's the main axis.
            }
        } else { // delta_j >= delta_i
            error = (delta_i << 1) - delta_j;
            for (current_j = j1; current_j != j2; current_j += step_j) {
                if (current_j != j1 && gridMap.CellIsObstacle(current_i, current_j)) return false;

                if (error >= 0) {
                     if (gridMap.CellIsObstacle(current_i + step_i, current_j) &&
                         gridMap.CellIsObstacle(current_i, current_j + step_j)) {
                         return false;
                     }
                    current_i += step_i;
                    error -= (delta_j << 1);
                }
                error += (delta_i << 1);
            }
        }
        // The loops above check intermediate cells. (i2,j2) is confirmed traversable at start.
    }
    return true; // All intermediate cells are clear according to the mode
}


Node Theta::resetParent(Node current, Node parent, const प्योर_थीटा_स्टार::GridMap &gridMap, const EnvironmentOptions &options )
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
        Node inpathNode; // To store points for lppath

        // Standard Bresenham to fill cells between (i1,j1) and (i2,j2)
        // Add cells from (i1,j1)+1 up to (i2,j2)
        if (delta_i > delta_j) {
            error = (delta_j << 1) - delta_i;
            while(current_i != i2) {
                current_i += step_i; // Move first
                if (error >= 0) {
                    current_j += step_j;
                    error -= (delta_i << 1);
                }
                error += (delta_j << 1);
                inpathNode.i = current_i;
                inpathNode.j = current_j;
                inpathNode.parent = nullptr; // lppath nodes don't need parent for this purpose
                inpathNode.F = inpathNode.g = inpathNode.H = 0;
                lppath.push_back(inpathNode);
            }
        } else { // delta_j >= delta_i
            error = (delta_i << 1) - delta_j;
            while(current_j != j2) {
                current_j += step_j; // Move first
                if (error >= 0) {
                    current_i += step_i;
                    error -= (delta_j << 1);
                }
                error += (delta_i << 1);
                inpathNode.i = current_i;
                inpathNode.j = current_j;
                inpathNode.parent = nullptr;
                inpathNode.F = inpathNode.g = inpathNode.H = 0;
                lppath.push_back(inpathNode);
            }
        }
        prevNode = curNode;
        ++it;
    }
}
