import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp
from scipy.signal import butter, filtfilt

# Define filter classes
class OldFilter:
    def __init__(self, state, comp_ratio=0.7):
        self.pos_state = state[:3]
        self.ori_state = state[3:]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = np.array(self.pos_state) * self.comp_ratio + np.array(next_state[:3]) * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_rotvec(
            np.stack([self.ori_state, next_state[3:]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_rotvec()
        return np.concatenate([self.pos_state, self.ori_state])

class Filter:
    def __init__(self, state, comp_ratio=0.5, cutoff_freq=15.0, fs=90.0, order=2):
        self.pos_state = state[:3]
        self.ori_state = state[3:]
        self.comp_ratio = comp_ratio
        self.cutoff_freq = cutoff_freq
        self.fs = fs  # Sampling frequency
        self.order = order
        # Create buffers for filtered output to stabilize initial outputs
        self.filtered_pos = np.array([self.pos_state] * 10)
        self.filtered_ori = np.array([self.ori_state] * 10)

        # Create a Butterworth filter
        self.b, self.a = butter(self.order, self.cutoff_freq / (0.5 * fs), btype='low')

    def __call__(self, next_state):
        # Calculate new interpolated state
        new_pos = np.array(self.pos_state) * self.comp_ratio + np.array(next_state[:3]) * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_rotvec(
            np.stack([self.ori_state, next_state[3:]], axis=0)))
        new_ori = ori_interp([1 - self.comp_ratio])[0].as_rotvec()

        # Update buffers
        self.filtered_pos = np.vstack((self.filtered_pos[1:], new_pos))
        self.filtered_ori = np.vstack((self.filtered_ori[1:], new_ori))

        # Apply Butterworth filter
        filtered_pos = filtfilt(self.b, self.a, self.filtered_pos, axis=0)[-1]
        filtered_ori = filtfilt(self.b, self.a, self.filtered_ori, axis=0)[-1]

        # Update internal state
        self.pos_state = filtered_pos
        self.ori_state = filtered_ori

        return np.concatenate([filtered_pos, filtered_ori])

# Simulate noisy 3D position and orientation data
np.random.seed(0)
time = np.linspace(0, 10, 100)
true_positions = np.column_stack((np.sin(time), np.cos(time), time))
noisy_positions = true_positions + np.random.normal(0, 0.1, size=true_positions.shape)

# Simulate noisy orientations as rotation vectors
true_orientations = np.column_stack((0.1 * time, 0.2 * time, 0.3 * time))
noisy_orientations = true_orientations + np.random.normal(0, 0.05, size=true_orientations.shape)

# Initial state
initial_state = np.concatenate([true_positions[0], true_orientations[0]])

# Instantiate filters
old_filter = OldFilter(initial_state.copy())
new_filter = Filter(initial_state.copy())

# Apply filters
old_filtered_states = []
new_filtered_states = []
for pos, ori in zip(noisy_positions, noisy_orientations):
    state = np.concatenate([pos, ori])
    old_filtered_states.append(old_filter(state))
    new_filtered_states.append(new_filter(state))

old_filtered_states = np.array(old_filtered_states)
new_filtered_states = np.array(new_filtered_states)

# Plotting the 3D trajectory with orientations
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot true positions and orientations
ax.quiver(noisy_positions[:, 0], noisy_positions[:, 1], noisy_positions[:, 2],
          noisy_orientations[:, 0], noisy_orientations[:, 1], noisy_orientations[:, 2],
          length=0.1, normalize=True, color='k', label='True Orientation')
ax.plot(noisy_positions[:, 0], noisy_positions[:, 1], noisy_positions[:, 2], 'k', label='True Position')

# Plot old filter positions and orientations
ax.quiver(old_filtered_states[:, 0], old_filtered_states[:, 1], old_filtered_states[:, 2],
          old_filtered_states[:, 3], old_filtered_states[:, 4], old_filtered_states[:, 5],
          length=0.1, normalize=True, color='r', label='Old Filter Orientation')
ax.plot(old_filtered_states[:, 0], old_filtered_states[:, 1], old_filtered_states[:, 2], 'r', label='Old Filter Position')

# Plot new filter positions and orientations
ax.quiver(new_filtered_states[:, 0], new_filtered_states[:, 1], new_filtered_states[:, 2],
          new_filtered_states[:, 3], new_filtered_states[:, 4], new_filtered_states[:, 5],
          length=0.1, normalize=True, color='b', label='New Filter Orientation')
ax.plot(new_filtered_states[:, 0], new_filtered_states[:, 1], new_filtered_states[:, 2], 'b', label='New Filter Position')

ax.set_title('3d traj')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()