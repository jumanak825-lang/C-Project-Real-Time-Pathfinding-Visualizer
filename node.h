#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>
#include <cmath>
#include <utility> // For std::pair

enum class NodeState {
    Start,
    End,
    Wall,
    Path,
    Open,  // Node currently being evaluated
    Closed, // Node already evaluated
    Empty
};

/**
 * @brief Represents a single cell/node on the grid for pathfinding.
 */
class Node {
public:
    int x, y; // Grid coordinates
    double g_cost; // Cost from start node to this node (Actual cost)
    double h_cost; // Heuristic cost from this node to end node (Estimated cost)
    double f_cost; // Total cost (f = g + h)
    Node* parent;  // Pointer to the node that preceded this one on the shortest path
    NodeState state;

    Node(int x_coord, int y_coord, NodeState s = NodeState::Empty)
        : x(x_coord), y(y_coord), g_cost(INFINITY), h_cost(INFINITY), f_cost(INFINITY), parent(nullptr), state(s) {}

    // Method to calculate heuristic cost (Manhattan distance used here)
    void calculate_h_cost(const Node* end_node) {
        // H-cost is the distance in steps (ignoring walls)
        h_cost = std::abs(x - end_node->x) + std::abs(y - end_node->y);
    }

    // Overload the < operator for use in a priority queue (for the lowest F-cost)
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

#endif // NODE_H
