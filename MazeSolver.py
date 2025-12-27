
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, Menu
import heapq
import time
import random
from collections import deque
import os

class Node:
    """Represents a node in the search tree"""
    def __init__(self, state, parent=None, action=None, cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

class Maze:
    """Enhanced maze class with guaranteed solvable maze generation"""
    def __init__(self):
        self.walls = []
        self.start = None
        self.goal = None
        self.width = 0
        self.height = 0
        self.solution = None
        self.explored = set()
        self.num_explored = 0
        self.frontier_states = set()

    def load_from_file(self, filename):
        """Load maze from text file"""
        try:
            with open(filename, 'r') as f:
                contents = f.read().strip()

            if contents.count("A") != 1:
                raise Exception("Maze must have exactly one starting point (A)")
            if contents.count("G") != 1:
                raise Exception("Maze must have exactly one goal point (G)")

            lines = contents.splitlines()
            self.height = len(lines)
            self.width = max(len(line) for line in lines) if lines else 0

            self.walls = []

            for i in range(self.height):
                row = []
                for j in range(self.width):
                    try:
                        char = lines[i][j] if j < len(lines[i]) else ' '
                        if char == 'A':
                            self.start = (i, j)
                            row.append(False)
                        elif char == 'G':
                            self.goal = (i, j)
                            row.append(False)
                        elif char == ' ':
                            row.append(False)
                        else:
                            row.append(True)
                    except IndexError:
                        row.append(False)
                self.walls.append(row)

            return True
        except Exception as e:
            raise Exception(f"Error loading maze: {str(e)}")

    def generate_random_solvable(self, width, height):
        """Generate a random maze guaranteed to have a path from start to goal"""
        # Ensure odd dimensions for proper maze generation
        if width % 2 == 0:
            width += 1
        if height % 2 == 0:
            height += 1

        self.width = width
        self.height = height

        # Initialize maze with all walls
        self.walls = [[True for _ in range(width)] for _ in range(height)]

        # Set start and goal positions (ensure they're on odd coordinates)
        self.start = (1, 1)  # Top-left corner (odd coordinates)
        self.goal = (height - 2, width - 2)  # Bottom-right corner (odd coordinates)

        # Use recursive backtracking to generate a perfect maze
        self._recursive_backtrack_generation()

        # Ensure start and goal are accessible
        self.walls[self.start[0]][self.start[1]] = False
        self.walls[self.goal[0]][self.goal[1]] = False

    def _recursive_backtrack_generation(self):
        """Generate maze using recursive backtracking algorithm"""
        visited = set()
        stack = []

        # Start from the starting position
        current = self.start
        visited.add(current)
        self.walls[current[0]][current[1]] = False

        while True:
            # Get unvisited neighbors (2 cells away to maintain wall structure)
            neighbors = self._get_unvisited_neighbors(current, visited)

            if neighbors:
                # Choose a random neighbor
                next_cell = random.choice(neighbors)

                # Add current cell to stack for backtracking
                stack.append(current)

                # Remove wall between current and next cell
                wall_row = (current[0] + next_cell[0]) // 2
                wall_col = (current[1] + next_cell[1]) // 2
                self.walls[wall_row][wall_col] = False

                # Mark next cell as visited and make it current
                self.walls[next_cell[0]][next_cell[1]] = False
                visited.add(next_cell)
                current = next_cell

            elif stack:
                # Backtrack if no unvisited neighbors
                current = stack.pop()
            else:
                # Algorithm complete
                break

        # Additional step: ensure goal is always reachable by creating path if needed
        self._ensure_goal_reachable(visited)

    def _get_unvisited_neighbors(self, cell, visited):
        """Get unvisited neighbors at distance 2 (for maze generation)"""
        row, col = cell
        neighbors = []

        # Check all four directions at distance 2
        directions = [(-2, 0), (2, 0), (0, -2), (0, 2)]  # up, down, left, right

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc

            # Check bounds and if cell hasn't been visited
            if (1 <= new_row < self.height - 1 and
                1 <= new_col < self.width - 1 and
                (new_row, new_col) not in visited):
                neighbors.append((new_row, new_col))

        return neighbors

    def _ensure_goal_reachable(self, visited):
        """Ensure the goal is reachable from the generated maze"""
        if self.goal not in visited:
            # Find the nearest visited cell to the goal
            min_distance = float('inf')
            nearest_cell = None

            for cell in visited:
                distance = abs(cell[0] - self.goal[0]) + abs(cell[1] - self.goal[1])
                if distance < min_distance:
                    min_distance = distance
                    nearest_cell = cell

            # Create a direct path from nearest cell to goal
            if nearest_cell:
                self._carve_path(nearest_cell, self.goal)

    def _carve_path(self, start_cell, end_cell):
        """Carve a direct path between two cells"""
        current_row, current_col = start_cell
        goal_row, goal_col = end_cell

        # Move horizontally first
        while current_col != goal_col:
            if current_col < goal_col:
                current_col += 1
            else:
                current_col -= 1
            self.walls[current_row][current_col] = False

        # Then move vertically
        while current_row != goal_row:
            if current_row < goal_row:
                current_row += 1
            else:
                current_row -= 1
            self.walls[current_row][current_col] = False

    def generate_prim_maze(self, width, height):
        """Generate maze using Prim's algorithm for variety"""
        # Ensure odd dimensions
        if width % 2 == 0:
            width += 1
        if height % 2 == 0:
            height += 1

        self.width = width
        self.height = height

        # Initialize maze with all walls
        self.walls = [[True for _ in range(width)] for _ in range(height)]

        # Set start and goal
        self.start = (1, 1)
        self.goal = (height - 2, width - 2)

        # Prim's algorithm implementation
        visited = set()
        frontier = []

        # Start with a random cell
        start_cell = self.start
        visited.add(start_cell)
        self.walls[start_cell[0]][start_cell[1]] = False

        # Add all neighbors of start cell to frontier
        self._add_neighbors_to_frontier(start_cell, frontier, visited)

        while frontier:
            # Pick random frontier cell
            frontier_cell = random.choice(frontier)
            frontier.remove(frontier_cell)

            # Find visited neighbors
            visited_neighbors = self._get_visited_neighbors(frontier_cell, visited)

            if visited_neighbors:
                # Connect to random visited neighbor
                neighbor = random.choice(visited_neighbors)

                # Carve path
                wall_row = (frontier_cell[0] + neighbor[0]) // 2
                wall_col = (frontier_cell[1] + neighbor[1]) // 2
                self.walls[wall_row][wall_col] = False
                self.walls[frontier_cell[0]][frontier_cell[1]] = False

                visited.add(frontier_cell)

                # Add new frontier cells
                self._add_neighbors_to_frontier(frontier_cell, frontier, visited)

        # Ensure goal is accessible
        self.walls[self.goal[0]][self.goal[1]] = False
        self._ensure_goal_reachable(visited)

    def _add_neighbors_to_frontier(self, cell, frontier, visited):
        """Add unvisited neighbors to frontier list"""
        row, col = cell
        directions = [(-2, 0), (2, 0), (0, -2), (0, 2)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc

            if (1 <= new_row < self.height - 1 and
                1 <= new_col < self.width - 1 and
                (new_row, new_col) not in visited and
                (new_row, new_col) not in frontier):
                frontier.append((new_row, new_col))

    def _get_visited_neighbors(self, cell, visited):
        """Get visited neighbors at distance 2"""
        row, col = cell
        neighbors = []
        directions = [(-2, 0), (2, 0), (0, -2), (0, 2)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc

            if (0 <= new_row < self.height and
                0 <= new_col < self.width and
                (new_row, new_col) in visited):
                neighbors.append((new_row, new_col))

        return neighbors

    def neighbors(self, state):
        """Get valid neighbors of a state"""
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1))
        ]

        result = []
        for action, (r, c) in candidates:
            if (0 <= r < self.height and
                0 <= c < self.width and
                not self.walls[r][c]):
                result.append((action, (r, c)))

        return result

    def heuristic(self, state):
        """Manhattan distance heuristic for A*"""
        if not self.goal:
            return 0
        return abs(state[0] - self.goal[0]) + abs(state[1] - self.goal[1])

    def reset(self):
        """Reset maze state for new search"""
        self.explored = set()
        self.frontier_states = set()
        self.num_explored = 0
        self.solution = None

    def verify_solvability(self):
        """Verify that the maze has a path from start to goal using BFS"""
        if not self.start or not self.goal:
            return False

        visited = set()
        queue = deque([self.start])
        visited.add(self.start)

        while queue:
            current = queue.popleft()

            if current == self.goal:
                return True

            for _, neighbor in self.neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        return False

class MazeSolverApp:
    """Enhanced application class with improved maze generation"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Advanced Maze Solver - Guaranteed Solvable Mazes")
        self.root.geometry("1200x800")

        self.maze = Maze()
        self.cell_size = 20
        self.animation_speed = 50  # milliseconds
        self.is_running = False
        self.is_paused = False
        self.step_mode = False

        # Colors
        self.colors = {
            'wall': '#2C3E50',
            'empty': '#ECF0F1',
            'start': '#E74C3C',
            'goal': '#27AE60',
            'explored': '#F39C12',
            'frontier': '#3498DB',
            'solution': '#9B59B6'
        }

        self.setup_ui()
        self.setup_menu()
        self.create_sample_maze()

    def setup_ui(self):
        """Setup the user interface"""
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel for controls
        left_panel = ttk.Frame(main_frame, width=250)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_panel.pack_propagate(False)

        # Algorithm selection
        algo_frame = ttk.LabelFrame(left_panel, text="Algorithm", padding=10)
        algo_frame.pack(fill=tk.X, pady=(0, 10))

        self.algorithm_var = tk.StringVar(value="BFS")
        algorithms = ["BFS", "DFS", "A*", "Dijkstra"]

        for algo in algorithms:
            ttk.Radiobutton(algo_frame, text=algo, variable=self.algorithm_var,
                          value=algo).pack(anchor=tk.W)

        # Control buttons
        control_frame = ttk.LabelFrame(left_panel, text="Controls", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_btn = ttk.Button(control_frame, text="Start Search",
                                  command=self.start_search)
        self.start_btn.pack(fill=tk.X, pady=2)

        self.pause_btn = ttk.Button(control_frame, text="Pause",
                                  command=self.toggle_pause, state=tk.DISABLED)
        self.pause_btn.pack(fill=tk.X, pady=2)

        self.step_btn = ttk.Button(control_frame, text="Step",
                                 command=self.step_search, state=tk.DISABLED)
        self.step_btn.pack(fill=tk.X, pady=2)

        self.reset_btn = ttk.Button(control_frame, text="Reset",
                                  command=self.reset_search)
        self.reset_btn.pack(fill=tk.X, pady=2)

        # Speed control
        speed_frame = ttk.LabelFrame(left_panel, text="Animation Speed", padding=10)
        speed_frame.pack(fill=tk.X, pady=(0, 10))

        self.speed_var = tk.IntVar(value=50)
        self.speed_scale = ttk.Scale(speed_frame, from_=10, to=500,
                                   variable=self.speed_var, orient=tk.HORIZONTAL)
        self.speed_scale.pack(fill=tk.X)

        ttk.Label(speed_frame, text="Fast ← → Slow").pack()

        # Statistics
        stats_frame = ttk.LabelFrame(left_panel, text="Statistics", padding=10)
        stats_frame.pack(fill=tk.X, pady=(0, 10))

        self.stats_text = tk.Text(stats_frame, height=8, width=25)
        self.stats_text.pack(fill=tk.BOTH, expand=True)

        # Enhanced maze generation
        gen_frame = ttk.LabelFrame(left_panel, text="Generate Solvable Maze", padding=10)
        gen_frame.pack(fill=tk.X, pady=(0, 10))

        # Generation algorithm selection
        ttk.Label(gen_frame, text="Algorithm:").pack(anchor=tk.W)
        self.gen_algorithm_var = tk.StringVar(value="Recursive Backtrack")
        gen_algorithms = ["Recursive Backtrack", "Prim's Algorithm"]

        for algo in gen_algorithms:
            ttk.Radiobutton(gen_frame, text=algo, variable=self.gen_algorithm_var,
                          value=algo).pack(anchor=tk.W)

        # Size controls
        size_frame = ttk.Frame(gen_frame)
        size_frame.pack(fill=tk.X, pady=5)

        ttk.Label(size_frame, text="Width:").pack(side=tk.LEFT)
        self.width_var = tk.IntVar(value=21)
        width_spin = ttk.Spinbox(size_frame, from_=11, to=51, increment=2,
                               textvariable=self.width_var, width=8)
        width_spin.pack(side=tk.RIGHT)

        size_frame2 = ttk.Frame(gen_frame)
        size_frame2.pack(fill=tk.X, pady=2)

        ttk.Label(size_frame2, text="Height:").pack(side=tk.LEFT)
        self.height_var = tk.IntVar(value=21)
        height_spin = ttk.Spinbox(size_frame2, from_=11, to=51, increment=2,
                                textvariable=self.height_var, width=8)
        height_spin.pack(side=tk.RIGHT)

        ttk.Button(gen_frame, text="Generate Solvable Maze",
                 command=self.generate_guaranteed_solvable_maze).pack(fill=tk.X, pady=5)

        ttk.Button(gen_frame, text="Verify Solvability",
                 command=self.verify_maze_solvability).pack(fill=tk.X, pady=2)

        # Right panel for canvas
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Canvas with scrollbars
        canvas_frame = ttk.Frame(right_panel)
        canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(canvas_frame, bg=self.colors['empty'])

        v_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        h_scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)

        self.canvas.configure(yscrollcommand=v_scrollbar.set, xscrollcommand=h_scrollbar.set)

        v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Status bar
        self.status_var = tk.StringVar(value="Ready - Generated mazes are guaranteed to be solvable!")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def setup_menu(self):
        """Setup menu bar"""
        menubar = Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Open Maze", command=self.load_maze)
        file_menu.add_command(label="Save Maze", command=self.save_maze)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Generate menu
        generate_menu = Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Generate", menu=generate_menu)
        generate_menu.add_command(label="Recursive Backtrack Maze",
                                command=lambda: self.generate_specific_maze("Recursive Backtrack"))
        generate_menu.add_command(label="Prim's Algorithm Maze",
                                command=lambda: self.generate_specific_maze("Prim's Algorithm"))
        generate_menu.add_separator()
        generate_menu.add_command(label="Verify Current Maze", command=self.verify_maze_solvability)

        # View menu
        view_menu = Menu(menubar, tearoff=0)
        menubar.add_cascade(label="View", menu=view_menu)
        view_menu.add_command(label="Zoom In", command=self.zoom_in)
        view_menu.add_command(label="Zoom Out", command=self.zoom_out)
        view_menu.add_separator()
        view_menu.add_command(label="Reset Zoom", command=self.reset_zoom)

        # Help menu
        help_menu = Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
        help_menu.add_command(label="Instructions", command=self.show_instructions)

    def generate_guaranteed_solvable_maze(self):
        """Generate a maze guaranteed to be solvable"""
        width = self.width_var.get()
        height = self.height_var.get()
        algorithm = self.gen_algorithm_var.get()

        # Ensure odd dimensions for proper maze generation
        if width % 2 == 0:
            width += 1
            self.width_var.set(width)
        if height % 2 == 0:
            height += 1
            self.height_var.set(height)

        if algorithm == "Recursive Backtrack":
            self.maze.generate_random_solvable(width, height)
        elif algorithm == "Prim's Algorithm":
            self.maze.generate_prim_maze(width, height)

        # Verify the maze is actually solvable
        is_solvable = self.maze.verify_solvability()

        self.draw_maze()
        self.update_stats()

        if is_solvable:
            self.status_var.set(f"Generated solvable {algorithm} maze ({width}×{height}) ✓")
        else:
            self.status_var.set(f"Warning: Generated maze may not be solvable!")
            messagebox.showwarning("Warning", "The generated maze may not be solvable. Please try again.")

    def generate_specific_maze(self, algorithm):
        """Generate a specific type of maze"""
        self.gen_algorithm_var.set(algorithm)
        self.generate_guaranteed_solvable_maze()

    def verify_maze_solvability(self):
        """Verify that the current maze is solvable"""
        if not self.maze.start or not self.maze.goal:
            messagebox.showerror("Error", "Maze must have start (A) and goal (G) points")
            return

        is_solvable = self.maze.verify_solvability()

        if is_solvable:
            messagebox.showinfo("Verification Result",
                              "✓ Maze is solvable!\n\nThere is a path from start (A) to goal (G).")
            self.status_var.set("Maze verified: SOLVABLE ✓")
        else:
            messagebox.showerror("Verification Result",
                               "✗ Maze is NOT solvable!\n\nThere is no path from start (A) to goal (G).")
            self.status_var.set("Maze verified: NOT SOLVABLE ✗")

    def create_sample_maze(self):
        """Create a guaranteed solvable sample maze"""
        # Generate a small solvable maze using recursive backtracking
        self.maze.generate_random_solvable(21, 15)
        self.draw_maze()
        self.update_stats()
        self.status_var.set("Sample solvable maze loaded ✓")

    # ... (rest of the methods remain the same as in the original code)
    # I'll include the essential drawing and algorithm methods

    def draw_maze(self):
        """Draw the maze on canvas"""
        self.canvas.delete("all")

        if not self.maze.walls:
            return

        canvas_width = self.maze.width * self.cell_size
        canvas_height = self.maze.height * self.cell_size

        self.canvas.configure(scrollregion=(0, 0, canvas_width, canvas_height))

        for i in range(self.maze.height):
            for j in range(self.maze.width):
                x1 = j * self.cell_size
                y1 = i * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size

                color = self.get_cell_color(i, j)

                self.canvas.create_rectangle(x1, y1, x2, y2,
                                           fill=color, outline="#BDC3C7", width=1)

    def get_cell_color(self, row, col):
        """Get color for a specific cell"""
        if self.maze.walls[row][col]:
            return self.colors['wall']
        elif (row, col) == self.maze.start:
            return self.colors['start']
        elif (row, col) == self.maze.goal:
            return self.colors['goal']
        elif (self.maze.solution and
              hasattr(self.maze.solution, '__iter__') and
              (row, col) in self.maze.solution):
            return self.colors['solution']
        elif (row, col) in self.maze.explored:
            return self.colors['explored']
        elif (row, col) in self.maze.frontier_states:
            return self.colors['frontier']
        else:
            return self.colors['empty']

    def update_stats(self):
        """Update statistics display"""
        solvable_status = "✓ Verified Solvable" if self.maze.verify_solvability() else "✗ Not Solvable"

        stats = f"""Algorithm: {self.algorithm_var.get()}
Nodes explored: {self.maze.num_explored}
Frontier size: {len(self.maze.frontier_states)}
Maze size: {self.maze.width}×{self.maze.height}
Solvability: {solvable_status}

Path length: {len(self.maze.solution) if self.maze.solution else 'N/A'}

Generation: {self.gen_algorithm_var.get()}

Colors:
• Red: Start (A)
• Green: Goal (G)
• Orange: Explored
• Blue: Frontier  
• Purple: Solution
• Dark: Walls
• Light: Open paths"""

        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats)

    # Add essential methods for algorithm functionality
    def start_search(self):
        """Start the maze solving process"""
        if not self.maze.start or not self.maze.goal:
            messagebox.showerror("Error", "Maze must have start (A) and goal (G) points")
            return

        # Verify maze is solvable before starting
        if not self.maze.verify_solvability():
            result = messagebox.askyesno("Unsolvable Maze",
                                       "This maze appears to be unsolvable. Continue anyway?")
            if not result:
                return

        self.is_running = True
        self.is_paused = False
        self.step_mode = False

        self.start_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL)
        self.step_btn.config(state=tk.DISABLED)

        self.maze.reset()
        self.animation_speed = self.speed_var.get()

        algorithm = self.algorithm_var.get()
        self.status_var.set(f"Running {algorithm} search on verified solvable maze...")

        if algorithm == "BFS":
            self.search_generator = self.bfs_search()
        elif algorithm == "DFS":
            self.search_generator = self.dfs_search()
        elif algorithm == "A*":
            self.search_generator = self.astar_search()
        elif algorithm == "Dijkstra":
            self.search_generator = self.dijkstra_search()

        self.continue_search()

    def bfs_search(self):
        """Breadth-First Search generator"""
        frontier = deque([Node(self.maze.start)])
        self.maze.frontier_states = {self.maze.start}

        while frontier:
            if not self.is_running:
                return

            node = frontier.popleft()
            self.maze.frontier_states.discard(node.state)

            if node.state == self.maze.goal:
                self.reconstruct_path(node)
                yield "FOUND"
                return

            self.maze.explored.add(node.state)
            self.maze.num_explored += 1

            for action, state in self.maze.neighbors(node.state):
                if (state not in self.maze.explored and
                    state not in self.maze.frontier_states):
                    child = Node(state, node, action)
                    frontier.append(child)
                    self.maze.frontier_states.add(state)

            yield "CONTINUE"

        yield "NO_SOLUTION"

    def reconstruct_path(self, node):
        """Reconstruct solution path from goal node"""
        path = []
        while node.parent is not None:
            path.append(node.state)
            node = node.parent
        path.reverse()
        self.maze.solution = path

    def continue_search(self):
        """Continue the search animation"""
        if not self.is_running or self.is_paused:
            return

        try:
            result = next(self.search_generator)
            self.draw_maze()
            self.update_stats()

            if result == "FOUND":
                self.search_completed(True)
            elif result == "NO_SOLUTION":
                self.search_completed(False)
            else:
                self.root.after(self.animation_speed, self.continue_search)

        except StopIteration:
            self.search_completed(False)

    def search_completed(self, found_solution):
        """Handle search completion"""
        self.is_running = False

        self.start_btn.config(state=tk.NORMAL)
        self.pause_btn.config(state=tk.DISABLED)
        self.step_btn.config(state=tk.DISABLED)

        if found_solution:
            path_length = len(self.maze.solution) if self.maze.solution else 0
            self.status_var.set(f"✓ Solution found! Path length: {path_length}")
        else:
            self.status_var.set("✗ No solution found (this shouldn't happen with guaranteed solvable mazes!)")

        self.draw_maze()
        self.update_stats()

    def reset_search(self):
        """Reset the search"""
        self.is_running = False
        self.is_paused = False

        self.start_btn.config(state=tk.NORMAL)
        self.pause_btn.config(state=tk.DISABLED, text="Pause")
        self.step_btn.config(state=tk.DISABLED)

        self.maze.reset()
        self.draw_maze()
        self.update_stats()
        self.status_var.set("Ready - Generated mazes are guaranteed to be solvable!")

    def toggle_pause(self):
        """Toggle pause/resume"""
        self.is_paused = not self.is_paused

        if self.is_paused:
            self.pause_btn.config(text="Resume")
            self.step_btn.config(state=tk.NORMAL)
            self.status_var.set("Paused")
        else:
            self.pause_btn.config(text="Pause")
            self.step_btn.config(state=tk.DISABLED)
            self.status_var.set(f"Running {self.algorithm_var.get()} search...")
            self.continue_search()

    def step_search(self):
        """Perform one step of search"""
        if not hasattr(self, 'search_generator'):
            return

        try:
            result = next(self.search_generator)
            self.draw_maze()
            self.update_stats()

            if result == "FOUND":
                self.search_completed(True)
            elif result == "NO_SOLUTION":
                self.search_completed(False)

        except StopIteration:
            self.search_completed(False)

    # Additional algorithm implementations (DFS, A*, Dijkstra)
    def dfs_search(self):
        """Depth-First Search generator"""
        frontier = [Node(self.maze.start)]
        self.maze.frontier_states = {self.maze.start}

        while frontier:
            if not self.is_running:
                return

            node = frontier.pop()
            self.maze.frontier_states.discard(node.state)

            if node.state == self.maze.goal:
                self.reconstruct_path(node)
                yield "FOUND"
                return

            if node.state not in self.maze.explored:
                self.maze.explored.add(node.state)
                self.maze.num_explored += 1

                for action, state in self.maze.neighbors(node.state):
                    if (state not in self.maze.explored and
                        state not in self.maze.frontier_states):
                        child = Node(state, node, action)
                        frontier.append(child)
                        self.maze.frontier_states.add(state)

                yield "CONTINUE"

        yield "NO_SOLUTION"

    def astar_search(self):
        """A* Search generator"""
        frontier = []
        heapq.heappush(frontier, (0, 0, Node(self.maze.start)))
        self.maze.frontier_states = {self.maze.start}
        cost_so_far = {self.maze.start: 0}
        counter = 1

        while frontier:
            if not self.is_running:
                return

            _, _, node = heapq.heappop(frontier)
            self.maze.frontier_states.discard(node.state)

            if node.state == self.maze.goal:
                self.reconstruct_path(node)
                yield "FOUND"
                return

            if node.state in self.maze.explored:
                continue

            self.maze.explored.add(node.state)
            self.maze.num_explored += 1

            for action, state in self.maze.neighbors(node.state):
                new_cost = cost_so_far[node.state] + 1

                if (state not in cost_so_far or new_cost < cost_so_far[state]):
                    cost_so_far[state] = new_cost
                    priority = new_cost + self.maze.heuristic(state)
                    child = Node(state, node, action, new_cost)
                    heapq.heappush(frontier, (priority, counter, child))
                    self.maze.frontier_states.add(state)
                    counter += 1

            yield "CONTINUE"

        yield "NO_SOLUTION"

    def dijkstra_search(self):
        """Dijkstra's Search generator - same as A* but without heuristic"""
        frontier = []
        heapq.heappush(frontier, (0, 0, Node(self.maze.start)))
        self.maze.frontier_states = {self.maze.start}
        cost_so_far = {self.maze.start: 0}
        counter = 1

        while frontier:
            if not self.is_running:
                return

            _, _, node = heapq.heappop(frontier)
            self.maze.frontier_states.discard(node.state)

            if node.state == self.maze.goal:
                self.reconstruct_path(node)
                yield "FOUND"
                return

            if node.state in self.maze.explored:
                continue

            self.maze.explored.add(node.state)
            self.maze.num_explored += 1

            for action, state in self.maze.neighbors(node.state):
                new_cost = cost_so_far[node.state] + 1

                if (state not in cost_so_far or new_cost < cost_so_far[state]):
                    cost_so_far[state] = new_cost
                    child = Node(state, node, action, new_cost)
                    heapq.heappush(frontier, (new_cost, counter, child))
                    self.maze.frontier_states.add(state)
                    counter += 1

            yield "CONTINUE"

        yield "NO_SOLUTION"

    # UI utility methods
    def load_maze(self):
        """Load maze from file"""
        filename = filedialog.askopenfilename(
            title="Select Maze File",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )

        if filename:
            try:
                self.maze.load_from_file(filename)
                self.draw_maze()
                self.update_stats()

                solvable = self.maze.verify_solvability()
                status = "✓ SOLVABLE" if solvable else "✗ NOT SOLVABLE"
                self.status_var.set(f"Loaded maze: {status}")

            except Exception as e:
                messagebox.showerror("Error", str(e))

    def save_maze(self):
        """Save current maze to file"""
        filename = filedialog.asksaveasfilename(
            title="Save Maze As",
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )

        if filename:
            try:
                with open(filename, 'w') as f:
                    for i in range(self.maze.height):
                        for j in range(self.maze.width):
                            if (i, j) == self.maze.start:
                                f.write('A')
                            elif (i, j) == self.maze.goal:
                                f.write('G')
                            elif self.maze.walls[i][j]:
                                f.write('█')
                            else:
                                f.write(' ')
                        f.write('\n')

                self.status_var.set(f"Saved maze to {os.path.basename(filename)}")
            except Exception as e:
                messagebox.showerror("Error", f"Could not save maze: {str(e)}")

    def zoom_in(self):
        """Increase cell size"""
        if self.cell_size < 50:
            self.cell_size += 5
            self.draw_maze()

    def zoom_out(self):
        """Decrease cell size"""
        if self.cell_size > 10:
            self.cell_size -= 5
            self.draw_maze()

    def reset_zoom(self):
        """Reset zoom to default"""
        self.cell_size = 20
        self.draw_maze()

    def show_about(self):
        """Show about dialog"""
        messagebox.showinfo("About",
            "Advanced Maze Solver with Guaranteed Solvable Generation\n\n"
            "Features:\n"
            "• Multiple search algorithms (BFS, DFS, A*, Dijkstra)\n"
            "• Guaranteed solvable maze generation\n"
            "• Recursive backtracking algorithm\n"
            "• Prim's algorithm for maze generation\n"
            "• Solvability verification\n"
            "• Interactive step-by-step visualization\n\n"
            "All generated mazes are guaranteed to have a path from start to goal!")

    def show_instructions(self):
        """Show instructions dialog"""
        instructions = """Advanced Maze Solver Instructions:

GUARANTEED SOLVABLE MAZES:
• All generated mazes are guaranteed to be solvable
• Choose between Recursive Backtracking and Prim's Algorithm
• Verify any maze's solvability with the verification button

MAZE GENERATION:
1. Select generation algorithm (Recursive Backtrack or Prim's)
2. Set desired width and height (odd numbers work best)
3. Click "Generate Solvable Maze"
4. The maze is automatically verified as solvable

SOLVING MAZES:
1. Choose a search algorithm (BFS, DFS, A*, Dijkstra)
2. Click "Start Search" to begin visualization
3. Use "Pause" and "Step" for detailed analysis
4. Adjust animation speed with the slider

FEATURES:
• BFS: Guarantees shortest path
• DFS: Memory efficient, explores deeply
• A*: Optimal with heuristic guidance  
• Dijkstra: Explores uniformly

MAZE FORMAT:
• A = Start position
• G = Goal position
• █ = Wall
• (space) = Open path

The application ensures all generated mazes have exactly one path 
from start to goal, making them perfect for algorithm comparison!"""

        messagebox.showinfo("Instructions", instructions)

    def run(self):
        """Start the application"""
        self.root.mainloop()

if __name__ == "__main__":
    app = MazeSolverApp()
    app.run()
