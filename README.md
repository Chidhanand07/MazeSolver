# Advanced Maze Solver

A powerful and interactive maze solving application with guaranteed solvable maze generation, featuring multiple pathfinding algorithms and real-time visualization. Built with Python and Tkinter.

## Features

### Guaranteed Solvable Maze Generation
- **Recursive Backtracking Algorithm**: Creates perfect mazes with a single path
- **Prim's Algorithm**: Generates mazes with a different structural pattern
- **Automatic Verification**: Every generated maze is verified to be solvable
- **Customizable Dimensions**: Generate mazes from 11×11 to 51×51

### Multiple Search Algorithms
- **BFS (Breadth-First Search)**: Guarantees shortest path
- **DFS (Depth-First Search)**: Memory efficient, explores deeply
- **A* (A-Star)**: Optimal pathfinding with Manhattan distance heuristic
- **Dijkstra's Algorithm**: Explores uniformly, finds shortest path

### Interactive Visualization
- **Step-by-Step Animation**: Watch algorithms explore the maze in real-time
- **Pause/Resume**: Control the visualization at any point
- **Step Mode**: Advance through the search one step at a time
- **Adjustable Speed**: Control animation speed from fast to slow
- **Color-Coded Visualization**:
  - Red: Start position (A)
  - Green: Goal position (G)
  - Orange: Explored nodes
  - Blue: Frontier (nodes being considered)
  - Purple: Solution path
  - Dark Gray: Walls
  - Light Gray: Open paths

### Additional Features
- **Load/Save Mazes**: Import and export maze files
- **Zoom Controls**: Zoom in/out for better visualization
- **Real-Time Statistics**: Track nodes explored, frontier size, and path length
- **Solvability Verification**: Verify any maze before solving

## Requirements

- Python 3.7 or higher
- Tkinter (usually comes with Python)

## Installation

1. Ensure Python 3.7+ is installed:
```bash
python --version
```

2. Tkinter is typically included with Python. If not installed:

**Windows/Mac**: Tkinter usually comes pre-installed

**Linux (Ubuntu/Debian)**:
```bash
sudo apt-get install python3-tk
```

**Linux (Fedora)**:
```bash
sudo dnf install python3-tkinter
```

3. Download the application:
```bash
# Clone or download the maze_solver.py file
```

## How to Run

```bash
python maze_solver.py
```

## Usage Guide

### Generating a Solvable Maze

1. **Select Generation Algorithm**:
   - Recursive Backtrack (default): Creates perfect mazes
   - Prim's Algorithm: Alternative generation method

2. **Set Maze Dimensions**:
   - Width: 11-51 (odd numbers recommended)
   - Height: 11-51 (odd numbers recommended)

3. **Generate**:
   - Click "Generate Solvable Maze"
   - The maze is automatically verified as solvable

4. **Verify** (optional):
   - Click "Verify Solvability" to confirm the maze has a solution

### Solving a Maze

1. **Choose Algorithm**:
   - Select BFS, DFS, A*, or Dijkstra from the radio buttons

2. **Start Solving**:
   - Click "Start Search" to begin visualization
   - Watch the algorithm explore the maze in real-time

3. **Control Execution**:
   - **Pause**: Freeze the animation
   - **Step**: Advance one step at a time (when paused)
   - **Reset**: Clear the search and start over

4. **Adjust Speed**:
   - Use the slider to control animation speed
   - Fast (left) to Slow (right)

### Loading Custom Mazes

1. Click **File → Open Maze**
2. Select a text file with maze format:
   - `A` = Start position
   - `G` = Goal position
   - `█` or `#` = Wall
   - ` ` (space) = Open path

Example maze file:
```
████████████
█A         █
█ ███ ███ █
█     █   █
█████ ███ █
█         G█
████████████
```

### Saving Mazes

1. Click **File → Save Maze**
2. Choose location and filename
3. Maze is saved in text format

## Keyboard Shortcuts

- **File Menu**: Load/Save mazes
- **Generate Menu**: Quick access to generation algorithms
- **View Menu**: Zoom controls
- **Help Menu**: Instructions and about information

## Algorithm Comparison

| Algorithm | Completeness | Optimality | Time Complexity | Space Complexity |
|-----------|-------------|------------|----------------|------------------|
| BFS | Yes | Yes (shortest path) | O(b^d) | O(b^d) |
| DFS | Yes | No | O(b^m) | O(bm) |
| A* | Yes | Yes (optimal) | O(b^d) | O(b^d) |
| Dijkstra | Yes | Yes | O(b^d) | O(b^d) |

*b = branching factor, d = depth of solution, m = maximum depth*

## Maze Generation Algorithms

### Recursive Backtracking
- Creates perfect mazes (exactly one path between any two points)
- Uses depth-first traversal
- Always generates solvable mazes
- Tends to create longer, winding corridors

### Prim's Algorithm
- Also creates perfect mazes
- Uses random weighted minimum spanning tree
- More balanced maze structure
- Creates mazes with shorter, more direct paths

## Statistics Panel

The statistics panel displays real-time information:
- **Algorithm**: Currently selected search algorithm
- **Nodes Explored**: Number of cells examined
- **Frontier Size**: Cells in the search queue
- **Maze Size**: Width × Height dimensions
- **Solvability**: Verification status
- **Path Length**: Number of steps in solution (when found)
- **Generation**: Algorithm used to create the maze

## Color Legend

- 🔴 **Red**: Start position (A)
- 🟢 **Green**: Goal position (G)
- 🟠 **Orange**: Explored cells
- 🔵 **Blue**: Frontier cells (being considered)
- 🟣 **Purple**: Solution path
- ⬛ **Dark Gray**: Walls
- ⬜ **Light Gray**: Open paths

## Technical Details

### Maze Generation
The application uses two main algorithms to guarantee solvable mazes:

1. **Recursive Backtracking**:
   - Starts from a random cell
   - Carves paths by visiting unvisited neighbors
   - Backtracks when stuck
   - Ensures all reachable cells are connected

2. **Prim's Algorithm**:
   - Maintains a frontier of walls
   - Randomly adds walls to the maze
   - Connects cells to create a spanning tree
   - Guarantees connectivity

### Verification System
Before starting any search, the application can verify maze solvability using BFS to ensure a path exists from start to goal.

## Project Structure

```
maze_solver/
│
├── maze_solver.py          # Main application
├── README.md              # This file
└── sample_mazes/          # Optional folder for maze files
    ├── simple.txt
    ├── medium.txt
    └── complex.txt
```

## Troubleshooting

### Application won't start
- Ensure Python 3.7+ is installed
- Verify Tkinter is installed: `python -c "import tkinter"`

### Maze appears unsolvable
- Use "Verify Solvability" to check
- Try regenerating the maze
- Ensure start (A) and goal (G) are properly placed

### Animation is too fast/slow
- Adjust the speed slider
- Range: 10ms (very fast) to 500ms (very slow)

### Can't see the entire maze
- Use the scrollbars to navigate
- Try "View → Zoom Out"
- Generate a smaller maze

## Advanced Usage

### Creating Custom Maze Files

Create a text file with the following format:

```
████████████
█A    █   █
█ ███ █ █ █
█ █   █ █ █
█ █ ███ █ █
█       █ G█
████████████
```

Rules:
- Use `█` (or `#`) for walls
- Use `A` for start position (exactly one)
- Use `G` for goal position (exactly one)
- Use spaces for open paths
- Rectangular shape recommended

### Comparing Algorithms

1. Generate or load a maze
2. Select first algorithm (e.g., BFS)
3. Click "Start Search" and note statistics
4. Click "Reset"
5. Select second algorithm (e.g., A*)
6. Click "Start Search" and compare results

### Best Practices

- Use BFS for guaranteed shortest path
- Use A* for faster solving with optimality
- Use DFS for memory-constrained situations
- Generate odd-dimensioned mazes (21×21, 31×31) for best results

## Future Enhancements

Potential features for future versions:
- Bidirectional search algorithms
- Wall-following algorithms
- 3D maze visualization
- Custom heuristics for A*
- Maze difficulty ratings
- Animation export (GIF/video)
- Multi-goal pathfinding
- Weighted graphs support

## Credits

**Algorithms**: Classic computer science pathfinding algorithms
**Maze Generation**: Recursive Backtracking and Prim's algorithms
**Framework**: Python Tkinter

## License

This project is open source and available for educational purposes.

## Contributing

Contributions are welcome! Areas for improvement:
- Additional maze generation algorithms
- New search algorithms
- Performance optimizations
- UI/UX enhancements
- Bug fixes

## Version History

- **v1.0**: Initial release with guaranteed solvable maze generation
  - Four search algorithms (BFS, DFS, A*, Dijkstra)
  - Two generation algorithms
  - Solvability verification
  - Interactive visualization

---

**Note**: All generated mazes are guaranteed to be solvable. If you encounter an unsolvable maze, please verify it using the verification feature or report it as a bug.
