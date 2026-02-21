import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# --- Simulation Parameters ---
ROOM_WIDTH = 4000  # mm
ROOM_HEIGHT = 3000 # mm
ROBOT_RADIUS = 150 # mm
COLLISION_THRESHOLD = 300 # mm
START_POS = np.array([1000.0, 1000.0])

# Define the target corner (top right)
TARGET_CORNER = np.array([ROOM_WIDTH, ROOM_HEIGHT])

# Set up the plot
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(0, ROOM_WIDTH)
ax.set_ylim(0, ROOM_HEIGHT)
ax.set_aspect('equal')
ax.set_title("MAXBOT LiDAR Simulation")
ax.grid(True, linestyle='--', alpha=0.6)

# Draw room boundaries
room = patches.Rectangle((0, 0), ROOM_WIDTH, ROOM_HEIGHT, linewidth=4, edgecolor='black', facecolor='none')
ax.add_patch(room)

# Robot components
robot_body = plt.Circle(START_POS, ROBOT_RADIUS, color='blue', zorder=3)
ax.add_patch(robot_body)
heading_line, = ax.plot([], [], 'r-', lw=2, zorder=4)  # Indicates forward vector
lidar_scan, = ax.plot([], [], 'g-', lw=1, alpha=0.5, zorder=2) # Simulates LiDAR beam
status_text = ax.text(150, ROOM_HEIGHT - 200, '', fontsize=12, bbox=dict(facecolor='white', alpha=0.8))

# State variables
robot_pos = np.copy(START_POS)
robot_angle = 0.0 # Radians
state = "INIT"
frame_count = 0
spins_completed = 0
distance_to_corner = 0

def get_heading_vector(angle):
    return np.array([np.cos(angle), np.sin(angle)])

def update(frame):
    global robot_pos, robot_angle, state, frame_count, spins_completed, distance_to_corner

    # Update LiDAR visual beam
    scan_end = robot_pos + get_heading_vector(robot_angle) * 2000
    lidar_scan.set_data([robot_pos[0], scan_end[0]], [robot_pos[1], scan_end[1]])

    if state == "INIT":
        status_text.set_text("STATE: CALIBRATING (Spinning)")
        robot_angle += 0.2  # Spin speed

        # Count full rotations
        if robot_angle >= 2 * np.pi:
            robot_angle -= 2 * np.pi
            spins_completed += 1

        if spins_completed >= 3:
            state = "FIND_CORNER"

    elif state == "FIND_CORNER":
        status_text.set_text("STATE: FINDING CORNER")
        # Calculate angle to top-right corner
        delta = TARGET_CORNER - robot_pos
        target_angle = np.arctan2(delta[1], delta[0])

        # Turn towards target
        angle_diff = target_angle - robot_angle
        # Normalize angle
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        if abs(angle_diff) > 0.05:
            robot_angle += np.sign(angle_diff) * 0.1
        else:
            state = "NAVIGATE"

    elif state == "NAVIGATE":
        status_text.set_text("STATE: NAVIGATING TO CORNER")
        # Move forward
        direction = get_heading_vector(robot_angle)
        distance_to_corner = np.linalg.norm(TARGET_CORNER - robot_pos)

        # Check collision threshold (simulated as distance to the physical corner)
        if distance_to_corner > (COLLISION_THRESHOLD + ROBOT_RADIUS):
            robot_pos += direction * 40 # Move speed
        else:
            state = "AVOID"

    elif state == "AVOID":
        status_text.set_text("STATE: WANDER (Obstacle Evaded)")
        # Perform 90 degree turn away from wall
        target_angle = robot_angle + (np.pi / 2)

        if robot_angle < target_angle:
             robot_angle += 0.1

    # Update visual positions
    robot_body.center = (robot_pos[0], robot_pos[1])
    heading_end = robot_pos + get_heading_vector(robot_angle) * (ROBOT_RADIUS * 1.5)
    heading_line.set_data([robot_pos[0], heading_end[0]], [robot_pos[1], heading_end[1]])

    return robot_body, heading_line, lidar_scan, status_text

# Create animation (approx 200 frames for the full sequence)
ani = animation.FuncAnimation(fig, update, frames=250, interval=40, blit=True)

# Save as GIF
print("Generating simulation GIF. This may take a moment...")
ani.save('tankbot_simulation.gif', writer='pillow', fps=25)
print("Saved as tankbot_simulation.gif")

plt.close()
