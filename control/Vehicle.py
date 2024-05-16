import numpy as np
import matplotlib.pyplot as plt

# Define trajectory (list of waypoints)
trajectory = [(2, 2), (15, 10), (17, 15), (10, 18), (5, 15), (2, 10)]

# Define vehicle initial pose
initial_pose = (1, 1, np.pi/4)  # (x, y, theta)

# Define Stanley controller
class StanleyController:
    def __init__(self, k_error=0.5, k_heading=0.5, k_cross_track=1.0):
        self.k_error = k_error
        self.k_heading = k_heading
        self.k_cross_track = k_cross_track
        self.last_cross_track_error = 0.0

    def calculate_steering(self, current_pose, target_pose, heading_error):
        dx = current_pose[0] - target_pose[0]
        dy = current_pose[1] - target_pose[1]
        cos_theta = np.cos(target_pose[2])
        sin_theta = np.sin(target_pose[2])
        cross_track_error = cos_theta * dy - sin_theta * dx

        heading_correction = np.arctan2(self.k_heading * cross_track_error, current_pose[2])
        steering_angle = heading_correction + heading_error

        if cross_track_error * self.last_cross_track_error < 0:
            steering_angle = np.clip(0.5 * steering_angle, -np.pi / 4, np.pi / 4)
        
        self.last_cross_track_error = cross_track_error

        return steering_angle

# Initialize Stanley controller
controller = StanleyController()

# Simulate vehicle motion
current_pose = initial_pose
trajectory_index = 0
num_steps = 100

trajectory_x = []
trajectory_y = []

for _ in range(num_steps):
    # Get target waypoint
    target_pose = trajectory[trajectory_index]

    # Calculate heading error
    heading_error = target_pose[2] - current_pose[2]

    # Calculate steering angle using Stanley controller
    steering_angle = controller.calculate_steering(current_pose, target_pose, heading_error)

    # Update vehicle pose
    current_pose = (
        current_pose[0] + np.cos(current_pose[2]),  # Update x position
        current_pose[1] + np.sin(current_pose[2]),  # Update y position
        current_pose[2] + np.tan(steering_angle)  # Update orientation
    )

    # Record vehicle position for plotting
    trajectory_x.append(current_pose[0])
    trajectory_y.append(current_pose[1])

    # Check if vehicle reached the current waypoint
    if np.linalg.norm(np.array(current_pose[:2]) - np.array(target_pose[:2])) < 0.5:
        trajectory_index = (trajectory_index + 1) % len(trajectory)  # Move to the next waypoint

# Plot trajectory and vehicle path
plt.plot(*zip(*trajectory), marker='o', color='b', linestyle='-', label='Trajectory')
# plt.plot(trajectory_x, trajectory_y, marker='.', color='r', linestyle='-', label='Vehicle Path')
plt.title('Trajectory Following with Stanley Controller')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
