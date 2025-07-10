#include "mission.h"
#include "astar.h"
#include "theta.h"
#include "gl_const.h"

Mission::Mission()
{
    search = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    search = nullptr;
}

Mission::~Mission()
{
    if (search)
        delete search;
}

bool Mission::getMap(int startX, int startY, int endX, int endY, int cellSize, std::vector<std::vector<int>> &mapData)
{
    return map.getMap(mapData, startX, startY, endX, endY, cellSize);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}


void Mission::createEnvironmentOptions()
{
    options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC], config.SearchParams[CN_SP_MT]);
}

void Mission::createSearch()
{
    if (search)
        delete search;
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
    {
        std::cout << "Using A* search algorithm." << std::endl;
        search = new Astar(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    }
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
    {
        std::cout << "Using Theta* search algorithm." << std::endl;
        search = new Theta(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    }
}

void Mission::startSearch()
{
    sr = search->startSearch(map, options);
    if (config.SearchParams[CN_SP_PS])
    {
        smooth_search_result(sr, map, options.cutcorners);
    }
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.cellSize << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}


const char *Mission::getAlgorithmName()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        return CNS_SP_ST_ASTAR;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        return CNS_SP_ST_TH;
    else
        return "";
}
