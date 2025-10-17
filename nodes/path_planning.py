import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq

GRID_SIZE = 20


class AStarVisualizer:
    def __init__(self):
        self.grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
        self.start = (1, 1)
        self.goal = (18, 18)  # Moved goal to avoid bottom row

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.obstacle_patches = []
        self.explored_patches = []

        self.init_grid()
        self.setup_plot()

    def init_grid(self):
        self.grid.fill(0)
        self.obstacle_patches.clear()
        self.explored_patches.clear()

        # Place obstacles along the bottom row, except goal cell
        bottom_row = GRID_SIZE - 2
        for col in range(GRID_SIZE):
            if col != self.goal[1]:  # Don't block the goal
                self.grid[bottom_row, col] = 1

    def setup_plot(self):
        self.ax.clear()
        self.ax.set_title("A* Pathfinding: Cannot Pass Through Obstacles")
        self.ax.set_xlim(0, GRID_SIZE)
        self.ax.set_ylim(0, GRID_SIZE)
        self.ax.set_aspect('equal')
        self.ax.set_xticks(np.arange(0, GRID_SIZE + 1, 1))
        self.ax.set_yticks(np.arange(0, GRID_SIZE + 1, 1))
        self.ax.grid(True, linestyle='-', linewidth=0.3)

        # Draw obstacles first
        for x in range(GRID_SIZE):
            for y in range(GRID_SIZE):
                if self.grid[x, y] == 1:
                    rect = patches.Rectangle((y, x), 1, 1, color='black', alpha=0.8, label='Obstacle' if x == GRID_SIZE-1 and y == 0 else "")
                    self.ax.add_patch(rect)

        # Draw start and goal
        self.ax.scatter(self.start[1] + 0.5, self.start[0] + 0.5, s=300, c='green', marker='s', label="Start")
        self.ax.scatter(self.goal[1] + 0.5, self.goal[0] + 0.5, s=300, c='red', marker='s', label="Goal")
        self.ax.legend()
        self.fig.canvas.draw()
        plt.pause(0.01)

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_valid_cell(self, x, y):
        """Check if cell is within bounds and not an obstacle"""
        return (0 <= x < GRID_SIZE and 
                0 <= y < GRID_SIZE and 
                self.grid[x, y] == 0)

    def astar(self):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(self.start, self.goal), 0, self.start, [self.start]))
        visited = set()
        explored = []

        while open_set:
            f, cost, current, path = heapq.heappop(open_set)
            
            if current in visited:
                continue
                
            visited.add(current)
            explored.append(current)

            # Visualize exploration (skip start and goal for cleaner view)
            if current != self.start and current != self.goal:
                rect = patches.Rectangle((current[1] + 0.1, current[0] + 0.1), 0.8, 0.8,
                                         color='lightblue', alpha=0.5)
                self.ax.add_patch(rect)
                self.explored_patches.append(rect)
                self.fig.canvas.draw_idle()
                plt.pause(0.03)

            if current == self.goal:
                return path, explored

            # Explore neighbors
            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                
                if self.is_valid_cell(nx, ny) and (nx, ny) not in visited:
                    new_cost = cost + 1
                    priority = new_cost + self.heuristic((nx, ny), self.goal)
                    heapq.heappush(open_set, (priority, new_cost, (nx, ny), path + [(nx, ny)]))

        return None, explored

    def draw_path(self, path):
        if hasattr(self, "path_line") and self.path_line:
            self.path_line.remove()

        if path:
            # Verify path doesn't go through obstacles
            for x, y in path:
                if self.grid[x, y] == 1:
                    print(f"ERROR: Path goes through obstacle at ({x}, {y})")
                    return

            # Draw path
            px, py = zip(*[(y + 0.5, x + 0.5) for (x, y) in path])
            self.path_line, = self.ax.plot(py, px, color='blue', linewidth=4, alpha=0.8, label="Optimal Path")
            
            # Mark path cells
            for i, (x, y) in enumerate(path):
                if (x, y) != self.start and (x, y) != self.goal:
                    circle = patches.Circle((y + 0.5, x + 0.5), 0.15, color='darkblue', alpha=0.7)
                    self.ax.add_patch(circle)

            self.ax.legend()
            self.fig.canvas.draw_idle()
            plt.pause(0.1)
            
            print(f"Path found with {len(path)} steps")
            print(f"Path length: {len(path) - 1}")

    def run(self):
        print("Starting A* pathfinding demonstration...")
        print("Obstacles are placed on bottom row (black squares)")
        print("Algorithm must find path around obstacles, not through them")
        
        while True:
            self.init_grid()
            self.setup_plot()
            
            print("\nRunning A* algorithm...")
            path, explored = self.astar()
            
            if path:
                print(f"Success! Explored {len(explored)} cells")
                self.draw_path(path)
            else:
                print("No path found!")
                
            input("Press Enter to run again or Ctrl+C to exit...")


if __name__ == "__main__":
    visualizer = AStarVisualizer()
    visualizer.run()
