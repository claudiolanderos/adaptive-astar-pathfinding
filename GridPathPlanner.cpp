#include "GridPathPlanner.h"
#include <limits>
#include <stdlib.h>
#include <iostream>

GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star) {
	// TODO
    mGridHeight = grid->GetHeight();
    mGridWidth = grid->GetWidth();
    mAdaptive = use_adaptive_a_star;
    mGValue = 0;
    mExpansions = 0;
}
GridPathPlanner::~GridPathPlanner(){
	// TODO
    mOpenQueue.clear();
    mClosedList.clear();
    mOpenSet.clear();
}

xyLoc GridPathPlanner::GetNextMove(PartiallyKnownGrid* grid) {
	// TODO
	// This is just a dummy implementation that returns a random neighbor.

    mClosedList.clear();
    mOpenSet.clear();
    mOpenQueue.clear();
    
	xyLoc curr = grid->GetCurrentLocation();
    
    Node starting;
    starting.parent = nullptr;
    starting.location = curr;
    starting.gValue = 0;
    if(mAdaptive && mGValue != 0)
    {
        starting.hValue = mGValue - starting.gValue;
    }
    else {
        starting.hValue = abs(curr.x - grid->GetGoalLocation().x) + abs(curr.y - grid->GetGoalLocation().y);
    }
    starting.fValue = starting.gValue + starting.hValue;
    std::shared_ptr<Node> startNode = std::make_shared<Node>(starting);
    //mOpenQueue.push(startNode);
    mOpenSet.insert(std::make_pair(startNode->location, startNode));
    mOpenQueue.insert(startNode);
    
    while(!mOpenQueue.empty())
    {
        auto queueIter = mOpenQueue.begin();
        std::shared_ptr<Node> currentNode = *queueIter;
        if(currentNode->location == grid->GetGoalLocation())
        {
            mGValue = currentNode->gValue;
            return GetNextStep(currentNode);
        }
        mOpenQueue.erase(queueIter);
        mOpenSet.erase(currentNode->location);
        
        std::vector<xyLoc> neighbors;
        neighbors.push_back(xyLoc(currentNode->location.x+1, currentNode->location.y));
        neighbors.push_back(xyLoc(currentNode->location.x-1, currentNode->location.y));
        neighbors.push_back(xyLoc(currentNode->location.x, currentNode->location.y+1));
        neighbors.push_back(xyLoc(currentNode->location.x, currentNode->location.y-1));
        
        for (int i = 0; i < neighbors.size(); i++) {
            xyLoc n = neighbors[i];
            if (grid->IsValidLocation(n) && !grid->IsBlocked(n) && (mClosedList.find(n) == mClosedList.end())) {
                auto neighborNode = mOpenSet.find(n);
                auto inClosedNode = mClosedList.find(n);
                
                if(neighborNode == mOpenSet.end() && inClosedNode == mClosedList.end())
                {
                    Node temp;
                    temp.location = n;
                    temp.gValue = currentNode->gValue + 1;
                    if(mAdaptive && mGValue != 0)
                    {
                        temp.hValue = mGValue - temp.gValue;
                    }
                    else {
                        temp.hValue = abs(n.x - grid->GetGoalLocation().x) + abs(n.y - grid->GetGoalLocation().y);
                    }
                    temp.fValue = temp.gValue + temp.hValue;
                    temp.parent = currentNode;
                    std::shared_ptr<Node> tempNode = std::make_shared<Node>(temp);
                    mOpenQueue.insert(tempNode);
                    mOpenSet.insert(std::make_pair(tempNode->location, tempNode));
                }
                else if (inClosedNode == mClosedList.end() && currentNode->gValue + 1 + currentNode->hValue <= neighborNode->second->fValue)
                {
                    mOpenQueue.erase(neighborNode->second);
                    neighborNode->second->location = n;
                    neighborNode->second->parent = currentNode;
                    neighborNode->second->gValue = currentNode->gValue + 1;
                    if(mAdaptive && mGValue != 0)
                    {
                        neighborNode->second->hValue = mGValue - neighborNode->second->gValue;
                    }
                    else {
                        neighborNode->second->hValue = abs(neighborNode->second->location.x - grid->GetGoalLocation().x) + abs(neighborNode->second->location.y - grid->GetGoalLocation().y);
                    }
                    neighborNode->second->fValue = neighborNode->second->gValue + neighborNode->second->hValue;
                    mOpenQueue.insert(neighborNode->second);
                }
            }
        }
        mClosedList.insert(std::make_pair(currentNode->location, currentNode));
        mExpansions++;
    }
    // return failure
    return kInvalidXYLoc;
}

xyLoc GridPathPlanner::GetNextStep(std::shared_ptr<Node> node)
{
    xyLoc nextTile;
    auto iter = node;
    auto next = node;
    while (iter->parent != nullptr) {
        next = iter;
        iter = iter->parent;
    }
    std::cout << "Expansions: " << GetNumExpansions() << std::endl;
    return next->location;
}

int GridPathPlanner::GetNumExpansions() {
    return mExpansions;
}
