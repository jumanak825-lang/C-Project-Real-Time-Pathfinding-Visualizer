#include "Grid.h"
#include <iostream>

int main() {
    std::cout << "--------------------------------------------------------" << std::endl;
    std::cout << "C++ A* Pathfinding Visualizer (Console Simulation)" << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;

    const int GRID_ROWS = 15;
    const int GRID_COLS = 20;

    // --- 1. Setup the Grid ---
    Grid pathfinding_grid(GRID_ROWS, GRID_COLS);
    
    // Define Start (S) and End (E) coordinates
    // Start at top-left, End at bottom-right
    int start_x = 1, start_y = 1;
    int end_x = GRID_ROWS - 2, end_y = GRID_COLS - 2;

    pathfinding_grid.setup_grid(start_x, start_y, end_x, end_y);
    
    std::cout << "Initial Grid State (S=Start, E=End, #=Wall, .=Empty):\n";
    pathfinding_grid.display();

    // --- 2. Run the A* Algorithm ---
    std::cout << "Running A* Search...\n";
    bool path_found = pathfinding_grid.a_star_search();

    // --- 3. Display Results ---
    if (path_found) {
        std::cout << "Path found successfully! (@ = Path)\n";
        pathfinding_grid.display();
    } else {
        std::cout << "Path not found. The target is inaccessible.\n";
        // Optionally display the fully explored map to show why
        pathfinding_grid.display(); 
    }

    return 0;
}
