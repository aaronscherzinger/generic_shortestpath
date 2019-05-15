#include <iostream>
#include <limits>
#include <vector>

#include "pathfinding.h"

//////////////////////////////////////////////////////////////////
// Example for using the generic pathfinding.                   //
// This example creates a graph with an explicit representation //
// (i.e., lists of nodes and edges).                            //
//////////////////////////////////////////////////////////////////

/// Simple example node class
class Node
{
public:
    /// Constructor automatically assigns the next ID to the node
    Node()
    {
        static int currentID = 0;
        id_ = currentID++;
    }

    int getID() const {
        return id_;
    }

private:
    int id_;    ///< unique ID for each node
};

/// Simple example edge class
class Edge
{
public:
    /// Construct edge with start and destination node and cost
    Edge(int startNode, int endNode, float cost)
    : start_(startNode)
    , dest_(endNode)
    , cost_(cost)
    { }

    Edge() = delete;

    int getStart() const {
        return start_;
    }

    int getDestination() const {
        return dest_;
    }

    float getCost() const {
        return cost_;
    }

private:
    int start_;
    int dest_;
    float cost_;
};

/// Simple example graph class
class Graph
{
public:
    /// Construct graph (moves nodes and edges to members of graph)
    Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges)
    : nodes_(nodes)
    , edges_(edges)
    { }

    const std::vector<Node>& getNodes() {
        return nodes_;
    }

    const std::vector<Edge>& getEdges() {
        return edges_;
    }

private:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
};

// main creates a few nodes and edges and will then find the shortest path between two nodes
int main() {

    // create 20 nodes (ID 0..19)
    std::vector<Node> nodes;
    for (int i = 0; i < 20; ++i)
        nodes.push_back(Node());

    // Create edges to all nodes with higher ID with a cost of the squared difference of the IDs
    std::vector<Edge> edges;
    for (int i = 0; i < 20; ++i)
        for (int j = i+1; j < 20; ++j)
            edges.push_back(Edge(i, j, static_cast<float>((j-i) * (j-i))));

    // Create a graph from the list of nodes and edges
    Graph graph(std::move(nodes), std::move(edges));

    ////// FIRST EXAMPLE: const Node* as NODE_TYPE //////

    // Create PathFinding algorithm with lambda functions - note: all of those lambda functions are incredibly inefficient :-)
    PathFinding<const Node*> pathFinder(
        // heuristic: lower bound for a connection is difference between indices (
        [](const Node* a, const Node* b) {
            return static_cast<float>(std::abs(a->getID() - b->getID()));
        },
        // cost function: find edge and return its cost
        [&](const Node* a, const Node* b) {
            // iterate over all edges until we found the right one
            for (auto& e : graph.getEdges())
                if (e.getStart() == a->getID() && e.getDestination() == b->getID())
                    return e.getCost();
            // should not get here
            return std::numeric_limits<float>::infinity();
        },
        // get neighbors of a node
        [&](const Node* node) {
            std::vector<const Node*> result;
            // iterate over all edges
            for (auto& e : graph.getEdges()) {
                if (e.getStart() == node->getID()) {
                    // we found an outgoing edge - get its destination node and and append it to the results list
                    int dst = e.getDestination();
                    for (auto& n : graph.getNodes()) {
                        if (n.getID() == dst)
                            result.push_back(&n);
                    }
                }
            }
            return result;
        },
        // compare nodes by their ID
        [](const Node* a, const Node* b) {
            return a->getID() < b->getID();
        }
    );

    std::vector<const Node*> path = pathFinder.getShortestPath(&graph.getNodes().at(0), &graph.getNodes().at(19));

    std::cout << "First example: using const Node* as NODE_TYPE: " << std::endl;
    // print path
    std::cout << "Shortest path found:" << std::endl;
    // start node is not part of the path
    std::cout << "Node " << nodes.at(0).getID();
    for (auto p : path)
        std::cout << " -> " << p->getID();
    std::cout << std::endl;

    ////// SECOND EXAMPLE: int as NODE_TYPE //////

    // same example, but using Node IDs instead of const Node*, does not need a compare function parameter
    PathFinding<> idPathFinder(
        // heuristic: lower bound for a connection is difference between indices (
        [](int a, int b) {
            return static_cast<float>(std::abs(a - b));
        },
        // cost function: find edge and return its cost
        [&](int a, int b) {
            // iterate over all edges until we found the right one
            for (auto& e : graph.getEdges())
                if (e.getStart() == a && e.getDestination() == b)
                    return e.getCost();
            // should not get here
            return std::numeric_limits<float>::infinity();
        },
        // get neighbors of a node
        [&](int node) {
            std::vector<int> result;
            // iterate over all edges
            for (auto& e : graph.getEdges()) {
                if (e.getStart() == node) {
                    // we found an outgoing edge - get its destination node and and append it to the results list
                    int dst = e.getDestination();
                    for (auto& n : graph.getNodes()) {
                        if (n.getID() == dst)
                            result.push_back(n.getID());
                    }
                }
            }
            return result;
        }
    );

    std::vector<int> secondPath = idPathFinder.getShortestPath(graph.getNodes().at(0).getID(), graph.getNodes().at(19).getID());

    std::cout << "Second example: using Node ID (i.e., int) as NODE_TYPE: " << std::endl;
    // print path
    std::cout << "Shortest path found:" << std::endl;
    // start node is not part of the path
    std::cout << "Node " << nodes.at(0).getID();
    for (auto p : secondPath)
        std::cout << " -> " << p;
    std::cout << std::endl;

    return 0;
}
