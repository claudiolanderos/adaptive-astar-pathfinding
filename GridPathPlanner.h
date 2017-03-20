#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H

#include "PartiallyKnownGrid.h"
#include <queue>
#include <map>
#include <set>
#include <vector>

struct Node
{
    xyLoc location;
    int gValue;
    int hValue;
    int fValue;
    std::shared_ptr<Node> parent;
};

class NodeComparator {
public:
    bool operator() (std::shared_ptr<Node> lhs, std::shared_ptr<Node> rhs)
    {
        if(lhs->fValue == rhs->fValue)
        {
            if(lhs->gValue == rhs->gValue)
            {
                return lhs->location < rhs->location;
            }
            return lhs->gValue < rhs->gValue;
        }
        return lhs->fValue < rhs->fValue;
    }
};

class GridPathPlanner{
public:
	GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star = false);
	~GridPathPlanner();

	xyLoc GetNextMove(PartiallyKnownGrid* grid);
	int GetNumExpansions();

private:
    xyLoc GetNextStep(std::shared_ptr<Node>);
    
    int mExpansions;
    int mGridHeight;
    int mGridWidth;
    int mGValue;
    bool mAdaptive;
    std::set<std::shared_ptr<Node>, NodeComparator> mOpenQueue;
    std::map<xyLoc, std::shared_ptr<Node> > mClosedList;
    std::map<xyLoc, std::shared_ptr<Node> > mOpenSet;
};

#endif
