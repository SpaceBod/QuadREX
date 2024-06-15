import pygame
import random
import heapq
import math
import numpy as np

# Define constants
GRID_WIDTH = 50
GRID_HEIGHT = 25
CELL_SIZE = 20
WINDOW_WIDTH = GRID_WIDTH * CELL_SIZE
WINDOW_HEIGHT = GRID_HEIGHT * CELL_SIZE
LEGEND_WIDTH = 200
BUTTON_WIDTH = 100
BUTTON_HEIGHT = 30
INITIAL_OBSTACLES = 30
CLUSTER_SIZE = 15
COST_INTENSITY = 0.7
DETECTION_RADIUS = 3

# Initialize Pygame
pygame.init()
window = pygame.display.set_mode((WINDOW_WIDTH + LEGEND_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("A* Pathfinding with Wavefront Exploration")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREY = (240, 240, 240)
GREY_DARK = (200, 200, 200)
FRONTIER_COLOR = (255, 165, 0)  # Orange
YELLOW = (255, 255, 0)
BUTTON_COLOR = (150, 150, 150)
BUTTON_HOVER_COLOR = (0, 200, 200)
BUTTON_TEXT_COLOR = (255, 255, 255)
INPUT_BOX_COLOR = (255, 255, 255)
INPUT_BOX_ACTIVE_COLOR = (200, 200, 200)
INPUT_BOX_TEXT_COLOR = (0, 0, 0)

original_grid = None
original_cost_map = None
obstacles_input_text = str(INITIAL_OBSTACLES)
cluster_size_input_text = str(CLUSTER_SIZE)
cost_intensity_input_text = str(COST_INTENSITY)
obstacles_active = False
cluster_active = False
cost_intensity_active = False
total_cost = 0
keep_paths = False  # Variable to toggle keeping paths
all_paths = []  # List to store all paths


def flood_fill(grid, start):
    stack = [start]
    visited = set()
    visited.add(start)
    while stack:
        row, col = stack.pop()
        for d_row, d_col in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_row, new_col = row + d_row, col + d_col
            if (
                0 <= new_row < GRID_HEIGHT
                and 0 <= new_col < GRID_WIDTH
                and grid[new_row][new_col] == 1
                and (new_row, new_col) not in visited
            ):
                visited.add((new_row, new_col))
                stack.append((new_row, new_col))
    return visited


def generate_grid():
    grid = np.ones((GRID_HEIGHT, GRID_WIDTH), dtype=int)
    cost_map = np.ones((GRID_HEIGHT, GRID_WIDTH), dtype=int)

    num_obstacles = int(obstacles_input_text)
    cluster_size = int(cluster_size_input_text)
    cost_intensity = float(cost_intensity_input_text)

    obstacles = []

    for _ in range(num_obstacles):
        row = random.randint(0, GRID_HEIGHT - 1)
        col = random.randint(0, GRID_WIDTH - 1)
        if grid[row, col] == 0:
            continue

        grid[row, col] = 0
        cost_map[row, col] = 255
        temp_obstacles = [(row, col)]

        for _ in range(cluster_size):
            direction = random.choice([(0, 1), (1, 0), (0, -1), (-1, 0)])
            new_row = row + direction[0]
            new_col = col + direction[1]
            if 0 <= new_row < GRID_HEIGHT and 0 <= new_col < GRID_WIDTH:
                if grid[new_row, new_col] == 0:
                    continue
                grid[new_row, new_col] = 0
                cost_map[new_row, new_col] = 255
                temp_obstacles.append((new_row, new_col))
                row, col = new_row, new_col

        start = None
        for r in range(GRID_HEIGHT):
            for c in range(GRID_WIDTH):
                if grid[r, c] == 1:
                    start = (r, c)
                    break
            if start:
                break

        visited = flood_fill(grid, start)
        open_cells = [
            (r, c)
            for r in range(GRID_HEIGHT)
            for c in range(GRID_WIDTH)
            if grid[r, c] == 1
        ]

        if len(visited) == len(open_cells):
            obstacles.extend(temp_obstacles)
        else:
            for r, c in temp_obstacles:
                grid[r, c] = 1
                cost_map[r, c] = 1

    obstacle_positions = np.array(obstacles)
    for row in range(GRID_HEIGHT):
        for col in range(GRID_WIDTH):
            if grid[row, col] == 0:
                continue
            distances = np.hypot(
                obstacle_positions[:, 0] - row, obstacle_positions[:, 1] - col
            )
            min_distance = np.min(distances)
            cost_map[row, col] = max(
                1, int(255 * math.exp(-min_distance / cost_intensity))
            )

    while True:
        start_row, start_col = random.randint(0, GRID_HEIGHT - 1), random.randint(
            0, GRID_WIDTH - 1
        )
        if grid[start_row, start_col] == 1:
            start = (start_row, start_col)
            break

    return grid, cost_map, start


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(grid, cost_map, start, end):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            total_cost = g_score[end]
            return path, total_cost

        neighbors = [
            (current[0] + i, current[1] + j)
            for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]
        for neighbor in neighbors:
            if (
                0 <= neighbor[0] < GRID_HEIGHT
                and 0 <= neighbor[1] < GRID_WIDTH
                and grid[neighbor[0], neighbor[1]] != 0
            ):
                tentative_g_score = (
                    g_score[current] + cost_map[neighbor[0], neighbor[1]]
                )
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, float("inf")


def interpolate_points(p1, p2, t):
    return (p1[0] * (1 - t) + p2[0] * t, p1[1] * (1 - t) + p2[1] * t)


def bezier_curve(points, n=100):
    curve = []
    for t in range(n + 1):
        t = t / n
        p = points[:]
        while len(p) > 1:
            p = [interpolate_points(p[i], p[i + 1], t) for i in range(len(p) - 1)]
        curve.append(p[0])
    return curve


def bresenham_line(x1, y1, x2, y2):
    """Bresenham's Line Algorithm to determine visibility."""
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return points


def is_visible(grid, start, end):
    """Check if end is visible from start using Bresenham's line algorithm with corner handling."""
    for x, y in bresenham_line(start[0], start[1], end[0], end[1]):
        if grid[x, y] == 0:
            return False
        if (
            x != start[0]
            and y != start[1]
            and (grid[x, start[1]] == 0 or grid[start[0], y] == 0)
        ):
            return False
    return True


def mark_explored(grid, start, detection_radius):
    for dx in range(-detection_radius, detection_radius + 1):
        for dy in range(-detection_radius, detection_radius + 1):
            if dx**2 + dy**2 <= detection_radius**2:  # Ensure the radius is circular
                new_x, new_y = start[0] + dx, start[1] + dy
                if 0 <= new_x < GRID_HEIGHT and 0 <= new_y < GRID_WIDTH:
                    if grid[new_x, new_y] != 0 and is_visible(
                        grid, start, (new_x, new_y)
                    ):
                        grid[new_x, new_y] = 2


def find_frontiers(grid, detection_radius):
    frontiers = []
    for row in range(GRID_HEIGHT):
        for col in range(GRID_WIDTH):
            if grid[row, col] == 1:  # Unexplored
                for dx in range(-detection_radius, detection_radius + 1):
                    for dy in range(-detection_radius, detection_radius + 1):
                        new_x, new_y = row + dx, col + dy
                        if (
                            0 <= new_x < GRID_HEIGHT
                            and 0 <= new_y < GRID_WIDTH
                            and grid[new_x, new_y] == 2
                            and is_visible(grid, (new_x, new_y), (row, col))
                        ):  # Adjacent to an explored cell and visible
                            frontiers.append((row, col))
                            break
    return frontiers


def draw_legend():
    legend_items = [
        ("A* Path", BLUE),
        ("Start", GREEN),
        ("End", RED),
        ("Obstacle", BLACK),
        ("Cost Gradient", GREY_DARK),
        ("Explored", YELLOW),
        ("Frontier", FRONTIER_COLOR),
    ]

    font = pygame.font.Font(None, 24)
    x_offset = WINDOW_WIDTH + 20
    y_offset = 24

    pygame.draw.rect(window, WHITE, (WINDOW_WIDTH, 0, LEGEND_WIDTH, WINDOW_HEIGHT))

    for text, color in legend_items:
        label = font.render(text, True, BLACK)
        window.blit(label, (x_offset + 24, y_offset + 4))
        pygame.draw.rect(window, color, (x_offset, y_offset, 20, 20))
        y_offset += 24


def draw_button():
    font = pygame.font.Font(None, 24)
    button_rect = pygame.Rect(
        WINDOW_WIDTH + (LEGEND_WIDTH - BUTTON_WIDTH) // 2,
        WINDOW_HEIGHT - BUTTON_HEIGHT - 20,
        BUTTON_WIDTH,
        BUTTON_HEIGHT,
    )

    mouse_pos = pygame.mouse.get_pos()
    if button_rect.collidepoint(mouse_pos):
        pygame.draw.rect(window, BUTTON_HOVER_COLOR, button_rect)
    else:
        pygame.draw.rect(window, BUTTON_COLOR, button_rect)

    pygame.draw.rect(window, BLACK, button_rect, 2)

    label = font.render("Regenerate", True, BUTTON_TEXT_COLOR)
    label_rect = label.get_rect(center=button_rect.center)
    window.blit(label, label_rect)

    return button_rect


def draw_keep_paths_button():
    font = pygame.font.Font(None, 24)
    button_rect = pygame.Rect(
        WINDOW_WIDTH + (LEGEND_WIDTH - BUTTON_WIDTH) // 2,
        WINDOW_HEIGHT - BUTTON_HEIGHT - 60,
        BUTTON_WIDTH,
        BUTTON_HEIGHT,
    )

    mouse_pos = pygame.mouse.get_pos()
    if button_rect.collidepoint(mouse_pos):
        pygame.draw.rect(window, BUTTON_HOVER_COLOR, button_rect)
    else:
        pygame.draw.rect(window, BUTTON_COLOR, button_rect)

    pygame.draw.rect(window, BLACK, button_rect, 2)

    label_text = "Keep Paths" if not keep_paths else "Clear Paths"
    label = font.render(label_text, True, BUTTON_TEXT_COLOR)
    label_rect = label.get_rect(center=button_rect.center)
    window.blit(label, label_rect)

    return button_rect


def draw_input_boxes(obstacles_active, cluster_active, cost_intensity_active):
    font = pygame.font.Font(None, 24)

    obstacles_label_text = "Obstacles:"
    obstacles_label = font.render(obstacles_label_text, True, BLACK)
    window.blit(obstacles_label, (WINDOW_WIDTH + 20, WINDOW_HEIGHT // 2 - 30))

    obstacles_rect = pygame.Rect(
        WINDOW_WIDTH + 20,
        WINDOW_HEIGHT // 2 - 10,
        LEGEND_WIDTH - 40,
        BUTTON_HEIGHT,
    )
    if obstacles_active:
        pygame.draw.rect(window, INPUT_BOX_ACTIVE_COLOR, obstacles_rect)
    else:
        pygame.draw.rect(window, INPUT_BOX_COLOR, obstacles_rect)
    pygame.draw.rect(window, BLACK, obstacles_rect, 2)
    obstacles_text_surface = font.render(
        obstacles_input_text, True, INPUT_BOX_TEXT_COLOR
    )
    window.blit(obstacles_text_surface, (obstacles_rect.x + 10, obstacles_rect.y + 5))

    cluster_label_text = "Cluster Size:"
    cluster_label = font.render(cluster_label_text, True, BLACK)
    window.blit(cluster_label, (WINDOW_WIDTH + 20, WINDOW_HEIGHT // 2 + 30))

    cluster_rect = pygame.Rect(
        WINDOW_WIDTH + 20,
        WINDOW_HEIGHT // 2 + 50,
        LEGEND_WIDTH - 40,
        BUTTON_HEIGHT,
    )
    if cluster_active:
        pygame.draw.rect(window, INPUT_BOX_ACTIVE_COLOR, cluster_rect)
    else:
        pygame.draw.rect(window, INPUT_BOX_COLOR, cluster_rect)
    pygame.draw.rect(window, BLACK, cluster_rect, 2)
    cluster_text_surface = font.render(
        cluster_size_input_text, True, INPUT_BOX_TEXT_COLOR
    )
    window.blit(cluster_text_surface, (cluster_rect.x + 10, cluster_rect.y + 5))

    cost_intensity_label_text = "Cost Intensity:"
    cost_intensity_label = font.render(cost_intensity_label_text, True, BLACK)
    window.blit(cost_intensity_label, (WINDOW_WIDTH + 20, WINDOW_HEIGHT // 2 + 90))

    cost_intensity_rect = pygame.Rect(
        WINDOW_WIDTH + 20,
        WINDOW_HEIGHT // 2 + 110,
        LEGEND_WIDTH - 40,
        BUTTON_HEIGHT,
    )
    if cost_intensity_active:
        pygame.draw.rect(window, INPUT_BOX_ACTIVE_COLOR, cost_intensity_rect)
    else:
        pygame.draw.rect(window, INPUT_BOX_COLOR, cost_intensity_rect)
    pygame.draw.rect(window, BLACK, cost_intensity_rect, 2)
    cost_intensity_text_surface = font.render(
        cost_intensity_input_text, True, INPUT_BOX_TEXT_COLOR
    )
    window.blit(
        cost_intensity_text_surface,
        (cost_intensity_rect.x + 10, cost_intensity_rect.y + 5),
    )

    return obstacles_rect, cluster_rect, cost_intensity_rect


def draw_grid(grid, cost_map, start, end, path=None, total_cost=0):
    frontiers = find_frontiers(grid, DETECTION_RADIUS)

    for row in range(GRID_HEIGHT):
        for col in range(GRID_WIDTH):
            if grid[row, col] == 2:
                color = YELLOW
            elif (row, col) in frontiers:
                color = FRONTIER_COLOR
            else:
                color = (
                    255 - cost_map[row, col],
                    255 - cost_map[row, col],
                    255 - cost_map[row, col],
                )

            pygame.draw.rect(
                window,
                color,
                (
                    int(col * CELL_SIZE),
                    int(row * CELL_SIZE),
                    int(CELL_SIZE),
                    int(CELL_SIZE),
                ),
            )
            pygame.draw.rect(
                window,
                GREY,
                (
                    int(col * CELL_SIZE),
                    int(row * CELL_SIZE),
                    int(CELL_SIZE),
                    int(CELL_SIZE),
                ),
                1,
            )

            if grid[row, col] == 0:
                shape_type = random.choice(["circle", "rectangle"])
                if shape_type == "circle":
                    pygame.draw.circle(
                        window,
                        BLACK,
                        (
                            int(col * CELL_SIZE + CELL_SIZE // 2),
                            int(row * CELL_SIZE + CELL_SIZE // 2),
                        ),
                        int(CELL_SIZE // 3),
                    )
                else:
                    rect_size = int(CELL_SIZE // 2)
                    pygame.draw.rect(
                        window,
                        BLACK,
                        (
                            int(col * CELL_SIZE + (CELL_SIZE - rect_size) // 2),
                            int(row * CELL_SIZE + (CELL_SIZE - rect_size) // 2),
                            rect_size,
                            rect_size,
                        ),
                    )

    if keep_paths:
        for path_points in all_paths:
            bezier_points = bezier_curve(path_points)
            for i in range(len(bezier_points) - 1):
                pygame.draw.line(
                    window, BLUE, bezier_points[i], bezier_points[i + 1], 3
                )

    if path:
        path_points = [
            (
                int(col * CELL_SIZE + CELL_SIZE // 2),
                int(row * CELL_SIZE + CELL_SIZE // 2),
            )
            for row, col in path
        ]
        bezier_points = bezier_curve(path_points)
        for i in range(len(bezier_points) - 1):
            pygame.draw.line(window, BLUE, bezier_points[i], bezier_points[i + 1], 3)

    pygame.draw.rect(
        window,
        GREEN,
        (
            int(start[1] * CELL_SIZE),
            int(start[0] * CELL_SIZE),
            int(CELL_SIZE),
            int(CELL_SIZE),
        ),
    )
    pygame.draw.rect(
        window,
        RED,
        (
            int(end[1] * CELL_SIZE),
            int(end[0] * CELL_SIZE),
            int(CELL_SIZE),
            int(CELL_SIZE),
        ),
    )

    draw_legend()
    button_rect = draw_button()
    keep_paths_button_rect = draw_keep_paths_button()
    obstacles_rect, cluster_rect, cost_intensity_rect = draw_input_boxes(
        obstacles_active, cluster_active, cost_intensity_active
    )
    pygame.display.update()
    return (
        button_rect,
        keep_paths_button_rect,
        obstacles_rect,
        cluster_rect,
        cost_intensity_rect,
    )


def main():
    global original_grid, original_cost_map, keep_paths, all_paths
    global obstacles_input_text, cluster_size_input_text, cost_intensity_input_text
    global obstacles_active, cluster_active, cost_intensity_active, total_cost

    grid, cost_map, start = generate_grid()
    original_grid = grid.copy()
    original_cost_map = cost_map.copy()

    mark_explored(grid, start, DETECTION_RADIUS)
    frontiers = find_frontiers(grid, DETECTION_RADIUS)
    if frontiers:
        end = random.choice(frontiers)
        path, total_cost = a_star_search(grid, cost_map, start, end)
    else:
        path, total_cost = None, 0

    running = True
    move_index = 0  # Index to track the robot's movement along the path
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                col = int(event.pos[0] // CELL_SIZE)
                row = int(event.pos[1] // CELL_SIZE)

                if 0 <= col < GRID_WIDTH and 0 <= row < GRID_HEIGHT:
                    if event.button == 1:
                        if original_grid[row, col] == 1:
                            grid = original_grid.copy()
                            cost_map = original_cost_map.copy()
                            end = (row, col)
                            path, total_cost = a_star_search(grid, cost_map, start, end)
                            move_index = 0  # Reset movement index
                    elif event.button == 3:
                        if original_grid[row, col] == 1:
                            grid = original_grid.copy()
                            cost_map = original_cost_map.copy()
                            start = (row, col)
                            path, total_cost = a_star_search(grid, cost_map, start, end)
                            move_index = 0  # Reset movement index
                elif button_rect.collidepoint(event.pos):
                    grid, cost_map, start = generate_grid()
                    original_grid = grid.copy()
                    original_cost_map = cost_map.copy()

                    mark_explored(grid, start, DETECTION_RADIUS)
                    frontiers = find_frontiers(grid, DETECTION_RADIUS)
                    if frontiers:
                        end = random.choice(frontiers)
                        path, total_cost = a_star_search(grid, cost_map, start, end)
                    else:
                        path, total_cost = None, 0

                    move_index = 0  # Reset movement index
                elif keep_paths_button_rect.collidepoint(event.pos):
                    keep_paths = not keep_paths
                    if not keep_paths:
                        all_paths = []
                elif obstacles_rect.collidepoint(event.pos):
                    obstacles_active = True
                    cluster_active = False
                    cost_intensity_active = False
                elif cluster_rect.collidepoint(event.pos):
                    obstacles_active = False
                    cluster_active = True
                    cost_intensity_active = False
                elif cost_intensity_rect.collidepoint(event.pos):
                    obstacles_active = False
                    cluster_active = False
                    cost_intensity_active = True
                else:
                    obstacles_active = False
                    cluster_active = False
                    cost_intensity_active = False
            elif event.type == pygame.KEYDOWN:
                if obstacles_active:
                    if event.key == pygame.K_BACKSPACE:
                        obstacles_input_text = obstacles_input_text[:-1]
                    elif event.key == pygame.K_RETURN:
                        obstacles_active = False
                    else:
                        obstacles_input_text += event.unicode
                elif cluster_active:
                    if event.key == pygame.K_BACKSPACE:
                        cluster_size_input_text = cluster_size_input_text[:-1]
                    elif event.key == pygame.K_RETURN:
                        cluster_active = False
                    else:
                        cluster_size_input_text += event.unicode
                elif cost_intensity_active:
                    if event.key == pygame.K_BACKSPACE:
                        cost_intensity_input_text = cost_intensity_input_text[:-1]
                    elif event.key == pygame.K_RETURN:
                        cost_intensity_active = False
                    else:
                        cost_intensity_input_text += event.unicode

        (
            button_rect,
            keep_paths_button_rect,
            obstacles_rect,
            cluster_rect,
            cost_intensity_rect,
        ) = draw_grid(grid, cost_map, start, end, path, total_cost)
        pygame.display.flip()

        # Move along the path step-by-step
        if path and move_index < len(path) - 1:
            start = path[move_index]
            move_index += 1
            mark_explored(grid, start, DETECTION_RADIUS)
            pygame.time.wait(10)  # Wait for a short time to visualize the movement
        else:
            if keep_paths and path:
                path_points = [
                    (
                        int(col * CELL_SIZE + CELL_SIZE // 2),
                        int(row * CELL_SIZE + CELL_SIZE // 2),
                    )
                    for row, col in path
                ]
                all_paths.append(path_points)
            frontiers = find_frontiers(grid, DETECTION_RADIUS)
            while frontiers:
                nearest_frontier = min(frontiers, key=lambda f: heuristic(start, f))
                path, total_cost = a_star_search(
                    grid, cost_map, start, nearest_frontier
                )
                if path:  # Path found to the nearest frontier
                    end = nearest_frontier  # Update the end position to the nearest frontier
                    move_index = 0  # Reset movement index
                    break
                else:
                    grid[nearest_frontier[0], nearest_frontier[1]] = 0
                    frontiers.remove(nearest_frontier)

    pygame.quit()


if __name__ == "__main__":
    main()
