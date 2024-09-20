import heapq
import time

# Define direction constants (in degrees)
NORTH = 0
EAST = 90
SOUTH = 180
WEST = 270

# Mock function to get current orientation (now takes user input)
def get_current_orientation():
    orientation = int(input("Enter current orientation (0 for North, 90 for East, 180 for South, 270 for West): "))
    return orientation

# Function to turn the wheelchair clockwise by the specified degrees
def turn_clockwise(degrees):
    print(f"Turning clockwise {degrees} degrees...")
    time.sleep(degrees / 90)  # Simulate turning time based on 90-degree intervals
    print("Turn completed.")

# Function to move the wheelchair forward
def move_forward(distance):
    print(f"Moving forward {distance} meters...")

# Function to correct orientation with only clockwise turns
def correct_direction_clockwise(current_orientation, desired_orientation):
    turn_angle = (desired_orientation - current_orientation) % 360
    if turn_angle != 0:
        print(f"Current: {current_orientation}°, Desired: {desired_orientation}°, Turning: {turn_angle}° clockwise")
        turn_clockwise(turn_angle)

# Dijkstra's algorithm for finding the shortest path in a graph with directions.
def dijkstra_with_directions(graph, start, goal):
    pq = [(0, start)]  # (distance, node)
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    predecessors = {start: None}

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = predecessors[current_node]
            return path[::-1]  # Return reversed path

        for neighbor, weight, direction in graph[current_node]:
            distance = current_dist + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))
    
    return None  # If no path is found

# Example graph based on your diagram
graph = {
    'A': [('B', 5, 'UP'), ('E', 5, 'RIGHT')],
    'B': [('A', 5, 'DOWN'), ('C', 10, 'RIGHT')],
    'C': [('B', 10, 'LEFT'), ('D', 5, 'DOWN')],
    'D': [('C', 5, 'UP'), ('E', 5, 'LEFT')],
    'E': [('A', 5, 'LEFT'), ('D', 5, 'RIGHT')]
}

# Function to navigate through the path with directions
def navigate_path_with_directions(path):
    current_orientation = get_current_orientation()

    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]

        # Find the direction to turn based on the graph
        for neighbor, _, direction in graph[current_node]:
            if neighbor == next_node:
                desired_orientation = {
                    'UP': NORTH,
                    'RIGHT': EAST,
                    'DOWN': SOUTH,
                    'LEFT': WEST
                }[direction]
                break

        # Correct orientation and move
        correct_direction_clockwise(current_orientation, desired_orientation)
        move_forward(5)  # Move forward 5 meters
        current_orientation = desired_orientation  # Update current orientation

# Start and goal nodes
start = 'B'
goal = 'D'

# Find the shortest path using Dijkstra with directions
path = dijkstra_with_directions(graph, start, goal)

# Output the result
if path:
    print("Shortest path:", path)
    navigate_path_with_directions(path)
else:
    print("No path found")
