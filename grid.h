#ifndef GRID_H
#define GRID_H

#include "Node.h"
#include <queue>
#include <algorithm>
#include <list>

class Grid {
private:
    int rows, cols;
    std::vector<std::vector<Node>> grid_nodes;
    Node* start_node = nullptr;
    Node* end_node = nullptr;

public:
    Grid(int r, int c) : rows(r), cols(c) {
        // Initialize the 2D grid
        for (int i = 0; i < rows; ++i) {
            std::vector<Node> row;
            for (int j = 0; j < cols; ++j) {
                row.emplace_back(i, j);
            }
            grid_nodes.push_back(std::move(row));
        }
    }

    // Initialize the grid with a starting point, end point, and walls
    void setup_grid(int start_x, int start_y, int end_x, int end_y) {
        // Ensure coordinates are within bounds (Error handling omitted for brevity)
        start_node = &grid_nodes[start_x][start_y];
        end_node = &grid_nodes[end_x][end_y];

        start_node->state = NodeState::Start;
        end_node->state = NodeState::End;

        // --- Add Walls (Obstacles) ---
        // Simple walls pattern for demonstration
        for (int i = 2; i < rows - 2; ++i) {
            grid_nodes[i][5].state = NodeState::Wall;
            grid_nodes[i][10].state = NodeState::Wall;
        }
    }

    // The core A* Search Algorithm
    bool a_star_search() {
        if (!start_node || !end_node) return false;

        // 1. Initialize Open List (Priority Queue for fast access to lowest F-cost node)
        // We use a custom comparator with std::priority_queue
        auto comp = [](Node* a, Node* b) { return *a > *b; };
        std::priority_queue<Node*, std::vector<Node*>, decltype(comp)> open_list(comp);

        // 2. Setup Start Node
        start_node->g_cost = 0;
        start_node->calculate_h_cost(end_node);
        start_node->f_cost = start_node->g_cost + start_node->h_cost;
        open_list.push(start_node);

        // 3. Main Loop
        while (!open_list.empty()) {
            Node* current_node = open_list.top();
            open_list.pop();

            // Goal Check: Path found
            if (current_node == end_node) {
                retrace_path();
                return true;
            }

            // Mark the node as closed (evaluated)
            if (current_node->state != NodeState::Start && current_node->state != NodeState::End) {
                current_node->state = NodeState::Closed;
            }

            // 4. Check Neighbors (up, down, left, right)
            const int dx[] = {0, 0, 1, -1};
            const int dy[] = {1, -1, 0, 0};

            for (int i = 0; i < 4; ++i) {
                int neighbor_x = current_node->x + dx[i];
                int neighbor_y = current_node->y + dy[i];

                // Check bounds
                if (neighbor_x < 0 || neighbor_x >= rows || neighbor_y < 0 || neighbor_y >= cols) continue;

                Node* neighbor = &grid_nodes[neighbor_x][neighbor_y];

                // Check if neighbor is a wall or already closed
                if (neighbor->state == NodeState::Wall || neighbor->state == NodeState::Closed) continue;

                // Calculate potential new g_cost (movement cost is always 1 in this simple grid)
                double new_g_cost = current_node->g_cost + 1;

                // If a new, shorter path to the neighbor is found
                if (new_g_cost < neighbor->g_cost || neighbor->state == NodeState::Empty) {
                    neighbor->g_cost = new_g_cost;
                    neighbor->calculate_h_cost(end_node);
                    neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                    neighbor->parent = current_node;

                    // If neighbor is not in the open list, add it
                    if (neighbor->state == NodeState::Empty || neighbor->state == NodeState::Open) {
                        if (neighbor->state == NodeState::Empty) {
                             if (neighbor->state != NodeState::End) { // Don't mark the End node as Open
                                neighbor->state = NodeState::Open;
                            }
                            open_list.push(neighbor);
                        }
                        // Note: Re-adding to the priority queue is complex; 
                        // a simpler approach is to use a std::set/std::list for the Open List, 
                        // or to just re-insert the node (which is safe if the custom comparator works).
                    }
                }
            }
        }

        // Open list is empty, and target not reached
        return false;
    }

    // Retrace the path from the end node back to the start node
    void retrace_path() {
        Node* current = end_node->parent;
        while (current != start_node) {
            current->state = NodeState::Path;
            current = current->parent;
        }
    }

    // Display the current state of the grid
    void display() const {
        std::cout << "\nPathfinding Grid:\n";
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                char symbol;
                switch (grid_nodes[i][j].state) {
                    case NodeState::Start:  symbol = 'S'; break;
                    case NodeState::End:    symbol = 'E'; break;
                    case NodeState::Wall:   symbol = '#'; break;
                    case NodeState::Path:   symbol = '@'; break;
                    case NodeState::Open:   symbol = 'o'; break;
                    case NodeState::Closed: symbol = 'c'; break;
                    case NodeState::Empty:  symbol = '.'; break;
                    default:                symbol = '?'; break;
                }
                std::cout << symbol << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
};

#endif // GRID_H
