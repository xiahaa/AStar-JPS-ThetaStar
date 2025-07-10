#include "isearch.h"
#include <vector>
#include <cmath> // For std::sqrt, std::abs, std::pow
#include <limits> // For std::numeric_limits
#include <chrono> // For timing
#include <iostream> // For std::cout (used in stopCriterion)
#include <algorithm> // For std::find_if if needed for list removal

// map_types.h is included via isearch.h
// map.h should not be included

ISearch::ISearch()
{
    hweight = 1.0;
    breakingties = CN_SP_BT_GMAX;
    openSize = 0;
    current_goal_i = -1;
    current_goal_j = -1;
    current_map_width = -1;
    current_map_height = -1;
}

ISearch::~ISearch(void) {}

bool ISearch::stopCriterion()
{
    if (openSize == 0) {
        return true;
    }
    return false;
}

SearchResult ISearch::startSearch(ILogger *Logger, const प्योर_थीटा_स्टार::GridMap &gridMap, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    start_time = std::chrono::system_clock::now();

    current_map_width = gridMap.getWidth();
    current_map_height = gridMap.getHeight();
    current_goal_i = gridMap.getGoalRow();
    current_goal_j = gridMap.getGoalCol();

    open.assign(current_map_height, std::list<Node>());
    openSize = 0;
    close.clear();
    hppath.clear();
    lppath.clear();


    Node startNode;
    startNode.i = gridMap.getStartRow();
    startNode.j = gridMap.getStartCol();
    startNode.g = 0;
    startNode.H = computeHFromCellToCell(startNode.i, startNode.j, current_goal_i, current_goal_j, options);
    startNode.F = startNode.g + hweight * startNode.H;
    startNode.parent = nullptr;

    addOpen(startNode);

    int expanded_nodes = 0;
    bool pathfound = false;

    while (!stopCriterion()) {
        Node curNode = findMin();

        // Robust removal of curNode from open list
        bool removed = false;
        std::list<Node>& row_list = open[curNode.i];
        for(auto it_list = row_list.begin(); it_list != row_list.end(); ++it_list) {
            if(it_list->i == curNode.i && it_list->j == curNode.j && it_list->F == curNode.F && it_list->g == curNode.g) { // More specific match
                row_list.erase(it_list);
                removed = true;
                break;
            }
        }
        if (removed) {
            openSize--;
        } else {
            // This can happen if findMin returns a node that was already processed or
            // if the open list structure / findMin logic has inconsistencies.
            // For safety, we can check if it's already in close.
            if (close.count(std::make_pair(curNode.i, curNode.j))) continue; // Already processed
            // Otherwise, this is an unexpected state.
        }

        close[std::make_pair(curNode.i, curNode.j)] = curNode;
        expanded_nodes++;

        if (curNode.i == current_goal_i && curNode.j == current_goal_j) {
            pathfound = true;
            break;
        }

        std::list<Node> successors = findSuccessors(curNode, gridMap, options);
        Node* parentNodeInClose = &(close.at(std::make_pair(curNode.i, curNode.j)));

        for (Node successor_candidate : successors) { // Iterate by value, as resetParent returns a new/modified node
            successor_candidate.parent = parentNodeInClose;

            // Theta* resetParent logic:
            // successor_candidate is the node S' (neighbor of N)
            // parentNodeInClose is N (parent of S')
            // resetParent(S', N, ...) will check if S' can be parented by N.parent (grandparent of S')
            Node updated_successor = resetParent(successor_candidate, *parentNodeInClose, gridMap, options);
            // updated_successor now has potentially new parent and g-cost.

            updated_successor.H = computeHFromCellToCell(updated_successor.i, updated_successor.j, current_goal_i, current_goal_j, options);
            updated_successor.F = updated_successor.g + hweight * updated_successor.H;

            addOpen(updated_successor);
        }
        if(Logger) Logger->writeToLogOpenClose(open, close, false);
    }

    if(Logger) Logger->writeToLogOpenClose(open, close, true);

    sresult.pathfound = pathfound;
    sresult.nodescreated = close.size() + openSize;
    sresult.numberofsteps = close.size();

    if (pathfound) {
        // Retrieve the goal node from the closed set to ensure it has the correct parent pointers and g-cost
        Node goalNodeInClose = close.at(std::make_pair(current_goal_i, current_goal_j));
        makePrimaryPath(goalNodeInClose);
        sresult.pathlength = goalNodeInClose.g;
        makeSecondaryPath();
    } else {
        sresult.pathlength = 0;
    }
    
    end_time = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count()) / 1000000.0;

    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

Node ISearch::findMin()
{
    Node minNode;
    minNode.F = std::numeric_limits<double>::infinity();
    bool node_found = false;

    for (int i = 0; i < open.size(); ++i) {
        if (!open[i].empty()) {
            for (const auto& currentNode : open[i]) { // Iterate through all nodes in the current row's list
                if (!node_found || currentNode.F < minNode.F) {
                    minNode = currentNode;
                    node_found = true;
                } else if (currentNode.F == minNode.F) { // Tie-breaking
                    if ((breakingties == CN_SP_BT_GMAX && currentNode.g >= minNode.g) ||
                        (breakingties == CN_SP_BT_GMIN && currentNode.g < minNode.g)) {
                        minNode = currentNode;
                    }
                }
            }
        }
    }
    // If !node_found here and openSize > 0, it means open list might be corrupted or empty.
    // stopCriterion() should handle openSize == 0.
    return minNode;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const प्योर_थीटा_स्टार::GridMap &gridMap, const EnvironmentOptions &options)
{
    std::list<Node> successors;
    for (int di = -1; di <= +1; ++di) {
        for (int dj = -1; dj <= +1; ++dj) {
            if (di == 0 && dj == 0) continue;

            int ni = curNode.i + di;
            int nj = curNode.j + dj;

            if (gridMap.CellOnGrid(ni, nj) && !gridMap.CellIsObstacle(ni, nj)) {
                bool is_diagonal = (di != 0 && dj != 0);
                if (is_diagonal) {
                    if (!options.allowdiagonal) continue;
                    if (!options.cutcorners) {
                        if (gridMap.CellIsObstacle(curNode.i, nj) || gridMap.CellIsObstacle(ni, curNode.j)) {
                            continue;
                        }
                    }
                    // The 'allowsqueeze' logic from original was:
                    // else if (!options.allowsqueeze) {
                    //    if (map.CellIsObstacle(curNode.i, curNode.j + j) && map.CellIsObstacle(curNode.i + i, curNode.j))
                    //        continue;
                    // }
                    // This is identical to the !cutcorners check. If allowsqueeze is true, this check is skipped.
                    // If options.allowsqueeze is true, it means even if both corners are obstacles, we can pass.
                    // If options.cutcorners is true, it also means we can pass.
                    // The original logic for !allowsqueeze is effectively: if (!cutcorners && is_diagonal && (corner1_obs && corner2_obs)) then block.
                    // This seems like 'allowsqueeze' might be inverted or related to a specific interpretation.
                    // Standard: cutcorners=true means ignore corner obstacles. cutcorners=false means check them.
                    // Let's stick to the 'cutcorners' flag as the primary determinant for this behavior.
                }

                if (close.find(std::make_pair(ni, nj)) == close.end()) {
                    Node successorNode;
                    successorNode.i = ni;
                    successorNode.j = nj;
                    successorNode.g = curNode.g + (is_diagonal ? CN_SQRT_TWO : 1.0);
                    successors.push_back(successorNode);
                }
            }
        }
    }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode)
{
    hppath.clear();
    Node current = curNode;
    while (current.parent != nullptr) {
        hppath.push_front(current);
        current = *current.parent;
    }
    hppath.push_front(current);
}

void ISearch::makeSecondaryPath()
{
    lppath.clear();
    if (hppath.empty()) return;

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
        } else {  // delta_j >= delta_i, includes delta_j == 0 if delta_i == 0 (handled by i1==i2,j1==j2 check)
                 // or delta_j > 0 if delta_i == 0 (vertical line)
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


void ISearch::addOpen(Node newNode)
{
    // Check if already in CLOSE set
    if (close.count(std::make_pair(newNode.i, newNode.j))) {
        // Some algorithms might reopen nodes from CLOSE if a much better path is found.
        // Standard A* typically does not. Let's assume if it's in close, we ignore.
        // Node closed_node = close.at(std::make_pair(newNode.i, newNode.j));
        // if (newNode.g >= closed_node.g) return; // If new path not better, definetely ignore
        // else { /* reopen: remove from close, proceed to add to open */ close.erase(...); }
        return;
    }

    std::list<Node>& nodeListForRow = open[newNode.i]; // Get the list for the specific row

    // Check if a node with same coordinates exists in OPEN
    for (auto it = nodeListForRow.begin(); it != nodeListForRow.end(); ++it) {
        if (it->j == newNode.j) { // Node with same (i,j) found in open list for this row
            if (it->g <= newNode.g) { // Existing node has a better or equal g-cost
                                      // (F cost check was in original, but g is more fundamental for path cost)
                return; // Do nothing, existing path is better or equal
            } else { // New node offers a better g-cost
                nodeListForRow.erase(it); // Remove the old, worse node
                openSize--;
                break; // Important: exit loop as iterator is now invalid
            }
        }
    }

    // Insert newNode into the sorted list for its row (maintaining sorted order by F, then by g for tie-breaking)
    std::list<Node>::iterator iter_insert_pos = nodeListForRow.begin();
    while(iter_insert_pos != nodeListForRow.end()) {
        if (newNode.F < iter_insert_pos->F) {
            break;
        }
        if (newNode.F == iter_insert_pos->F) { // Tie-breaking on F
            if (breakingties == CN_SP_BT_GMAX && newNode.g >= iter_insert_pos->g) { // Prefer larger g
                break;
            } else if (breakingties == CN_SP_BT_GMIN && newNode.g < iter_insert_pos->g) { // Prefer smaller g
                break;
            }
        }
        ++iter_insert_pos;
    }
    nodeListForRow.insert(iter_insert_pos, newNode);
    openSize++;
}
