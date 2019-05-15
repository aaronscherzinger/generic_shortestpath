#ifndef GENERIC_PATHFINDING_H
#define GENERIC_PATHFINDING_H

#include <assert.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <vector>

/**
 * Generic implementation of A* algorithm
 * - NODE_TYPE is the type of nodes that is used. If you have unique IDs (e.g., int) for your nodes, you may directly use those, alternatively you
 *      can also use const* for NODE_TYPE if you have an existing graph and/or a large node type to save memory by avoiding copy operations for
 *      node objects, in general NODE_TYPE must be an assignable data type (provide a default constructor, copy constructor, and assignment operator)
 *      and implement operator==
 * - for VALUE_TYPE, addition by using +, comparison by using <, and std::numeric_limits<VALUE_TYPE> must be available (e.g., float, double, int)
 */
template<typename NODE_TYPE = int, typename VALUE_TYPE = float>
class PathFinding
{
public:
    // Function types used by the path finding algorithm as well as for storing nodes in ordered map
    using HeuristicFunction = std::function<VALUE_TYPE(NODE_TYPE, NODE_TYPE)>;
    using EdgeCostFunction = std::function<VALUE_TYPE(NODE_TYPE, NODE_TYPE)>;
    using GetOutgoingNeighborsFunction = std::function<std::vector<NODE_TYPE>(NODE_TYPE)>;
    using NodeCompareFunction = std::function<bool(NODE_TYPE, NODE_TYPE)>;

    /** Constructor for generic path finding algorithm using A* algorithm
     \param hFunc heuristic function for a heuristic distance between nodes (must be lower bound for actual distance!)
     \param cFunc cost function for traversing edge between a node and its neighbor
     \param nFunc function for retrieving all reachable neighbors of a given node (i.e., end nodes of outgoing edges)
     \param nodeCompare used for ordering nodes internally, must be provided if NODE_TYPE is not comparable by < or provides a specialized implementation of std::less
    */
    PathFinding(HeuristicFunction hFunc, EdgeCostFunction cFunc, GetOutgoingNeighborsFunction nFunc, NodeCompareFunction nodeCompare = [](NODE_TYPE a, NODE_TYPE b){ static std::less<NODE_TYPE> f; return f(a,b); })
        : heuristicFunc_(hFunc)
        , costFunc_(cFunc)
        , getNeighbors_(nFunc)
        , queue_([](QueueElement a, QueueElement b) { return b.first < a.first; })
        , cost_(nodeCompare)
        , parent_(nodeCompare)
        , visited_(nodeCompare)
    { }

    PathFinding() = delete;

    /// Returns the shortest path from start to destination (excluding the start node, but including the destination node) using A* (returns an empty vector if no path exists)
    std::vector<NODE_TYPE>&& getShortestPath(NODE_TYPE start, NODE_TYPE destination);

private:
    /// Resets all data
    void reset() {
        queue_ = QueueType([](QueueElement a, QueueElement b) { return b.first < a.first; });
        cost_.clear();
        parent_.clear();
        visited_.clear();
    }

    /// Function for heuristic distance between nodes - needs to be lower bound for cost function in order for A* to find shortest path
    HeuristicFunction heuristicFunc_;

    /// Cost function for transition betwen nodes (only called for direct neighbors, i.e., outgoing edges of the first node)
    EdgeCostFunction costFunc_;

    /// Function for retrieving the neighbors of a specific node
    GetOutgoingNeighborsFunction getNeighbors_;

    /// Queue contains nodes and the heuristic for the priority which is used to compare the nodes by a lambda function
    using QueueElement = std::pair<VALUE_TYPE, NODE_TYPE>;
    using QueueElementCompareFunc = std::function<bool(QueueElement, QueueElement)>;
    using QueueType = std::priority_queue<QueueElement, std::vector<QueueElement>, QueueElementCompareFunc>;
    QueueType queue_;

    // maps for executing the algorithm
    std::map<NODE_TYPE, VALUE_TYPE, NodeCompareFunction> cost_;
    std::map<NODE_TYPE, NODE_TYPE, NodeCompareFunction> parent_;
    std::map<NODE_TYPE, bool, NodeCompareFunction> visited_;

    // result vector (moved when returning the result)
    std::vector<NODE_TYPE> shortestPath_;
};

// A* implementation
template<typename NODE_TYPE, typename VALUE_TYPE>
std::vector<NODE_TYPE>&& PathFinding<NODE_TYPE, VALUE_TYPE>::getShortestPath(NODE_TYPE start, NODE_TYPE destination) {

    bool destinationReached = false;
    // set start node as its own parent (used as invalid value / None type)
    cost_[start] = 0;
    parent_[start] = start;
    queue_.push(std::make_pair(cost_[start], start));

    while (!queue_.empty()) {
        // get next node from the queue
        NODE_TYPE currentNode = queue_.top().second;
        queue_.pop();

        VALUE_TYPE currentCost = cost_[currentNode];
        visited_[currentNode] = true;

        if (currentNode == destination) {
            // found the shortest path to the destination
            destinationReached = true;
            break;
        }

        // get the nodes of the outgoing edges from the node
        auto neighbors = getNeighbors_(currentNode);

        // for each outgoing edge
        for (auto& n : neighbors) {
            // skip nodes that we already visited
            if (visited_.find(n) != visited_.end())
                continue;

            // get current cost of the neighbor (or infinity if the node has not been seen yet)
            VALUE_TYPE currentNeighborCost = cost_.find(n) != cost_.end() ? cost_[n] : std::numeric_limits<VALUE_TYPE>::infinity();

            VALUE_TYPE newCost = currentCost + costFunc_(currentNode, n);
            if (newCost < currentNeighborCost) {
                // we found a shorter path to the current neighbor node than before
                cost_[n] = newCost;
                parent_[n] = currentNode;

                // std::priority_queue does not allow to change the priority of an existing node so even if n has already been inserted, we insert it again
                queue_.push(std::make_pair(newCost + heuristicFunc_(n, destination), n));
            }
        }
    }

    if (!destinationReached) {
        // no path exists from start to destination
        assert(queue_.empty());
    }
    else {
        // construct path from parents - does not contain the start node
        NODE_TYPE currentNode = destination;
        while(!(currentNode == start)) {
            shortestPath_.push_back(currentNode);
            currentNode = parent_[currentNode];
        }
        std::reverse(shortestPath_.begin(), shortestPath_.end());
    }

    // clear datastructures that are not required anymore and return path
    reset();
    return std::move(shortestPath_);
}

#endif // GENERIC_PATHFINDING_H

