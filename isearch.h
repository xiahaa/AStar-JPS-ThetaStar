#ifndef ISEARCH_H
#define ISEARCH_H

#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "map_types.h" // Uses pure_theta_star namespace

#include <list>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <functional>

#include "node.h"


class ISearch
{
    public:
        ISearch();
        virtual ~ISearch(void);

        SearchResult startSearch(ILogger *Logger, const pure_theta_star::GridMap &gridMap, const EnvironmentOptions &options); // Namespace updated

    protected:
        Node findMin();
        virtual void addOpen(Node newNode);

        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j, const EnvironmentOptions &options) = 0;

        virtual std::list<Node> findSuccessors(Node curNode, const pure_theta_star::GridMap &gridMap, const EnvironmentOptions &options); // Namespace updated

        virtual void makePrimaryPath(Node curNode);
        virtual void makeSecondaryPath();

        virtual Node resetParent(Node current, Node parent, const pure_theta_star::GridMap &gridMap, const EnvironmentOptions &options) { return current; } // Namespace updated

        virtual bool stopCriterion();

        SearchResult                    sresult;
        std::list<Node>                 lppath;
        std::list<Node>                 hppath;

        struct PairHash {
            template <class T1, class T2>
            std::size_t operator () (const std::pair<T1,T2> &p) const {
                auto h1 = std::hash<T1>{}(p.first);
                auto h2 = std::hash<T2>{}(p.second);
                return h1 ^ (h2 << 1);
            }
        };
        std::unordered_map<std::pair<int,int>, Node, PairHash>    close;

        std::vector<std::list<Node>>    open;
        int                             openSize;

        double                          hweight;
        bool                            breakingties;

        int current_goal_i;
        int current_goal_j;
        int current_map_height;
        int current_map_width;

};
#endif
